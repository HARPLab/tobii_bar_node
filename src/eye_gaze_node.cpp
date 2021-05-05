#include "EyeGazeNode.hpp"
#include "tobii_bar_node/CalibrationInfoSrv.h"

OffsetManager::OffsetManager(std::string const & service_name, ros::NodeHandle nh) :
    service_server(nh.advertiseService("set_offset", &OffsetManager::updateOffset, this)),
    _offset{0., 0.} {}

std::array<double, 2> OffsetManager::get() const {
    boost::lock_guard<boost::mutex>(this->mutex);
    return this->_offset;
}

bool OffsetManager::updateOffset(tobii_bar_node::SetOffset::Request & req, tobii_bar_node::SetOffset::Response & resp) {
    if (req.offset.x == this->_offset[0] && req.offset.y == this->_offset[1]) {
        // no-op
        resp.ok = true;
        ROS_INFO_STREAM("No offset change current=(" << this->_offset[0] << ", " << this->_offset[1] << "), "
                   << "req=(" << req.offset.x << ", " << req.offset.y << ")");
        return true;
    } else {
        boost::lock_guard<boost::mutex>(this->mutex);
        this->_offset[0] = req.offset.x;
        this->_offset[1] = req.offset.y;
        resp.ok = true;
        ROS_INFO_STREAM("Set offset to (" << this->_offset[0] << ", " << this->_offset[1] << ")");
        return true;
    }
}

BasicPublisher::BasicPublisher(std::string const & topic_name, TobiiConnection & connection, OffsetManager & offset_manager) :
            nh(),
            pub(nh.advertise<ibmmpy::GazeData>(topic_name, 1)),
            connection(connection),
            offset_manager(offset_manager) {}

void BasicPublisher::processData(ros::Time const & recv_time, tobii_gaze_point_t const & gaze_point) {
    OffsetManager::OffsetType offset = this->offset_manager.get();

    ibmmpy::GazeData msg;
    msg.header.stamp = recv_time;
    msg.applied_offset.x = offset[0];
    msg.applied_offset.y = offset[1];
    msg.world_data.push_back(ibmmpy::GazeDataPoint());
    msg.world_data[0].header.stamp = recv_time;
    msg.world_data[0].position.x = gaze_point.position_xy[0] + offset[0];
    msg.world_data[0].position.y = gaze_point.position_xy[1] + offset[1];
    msg.world_data[0].confidence = 1.0; // doesn't seem to be a way to get a confidence flag, and we're already filtering validity
    this->pub.publish(msg);
}

BatchingPublisher::BatchingPublisher(std::string const &topic_name, TobiiConnection & connection, OffsetManager & offset_manager, ros::Duration const & batch_rate) : nh(),
            pub(nh.advertise<ibmmpy::GazeData>(topic_name, 1)),
            connection(connection), offset_manager(offset_manager), 
            cache(), rate(batch_rate) {}

void BatchingPublisher::processData(ros::Time const &recv_time, tobii_gaze_point_t const &gaze_point) {
    boost::lock_guard<boost::mutex>(this->cache_mutex);
    this->cache.push_back(std::make_pair(recv_time, gaze_point));
}
void BatchingPublisher::sendMessage(ros::TimerEvent const & e) {
    // grab the current cache -- just transfer ownership to this thread and leave the other an empty queue
    QueueType current_cache;
    {
        boost::lock_guard<boost::mutex>(this->cache_mutex);
        current_cache.swap(this->cache);
    }

    // now actually set up the message
    OffsetManager::OffsetType offset = this->offset_manager.get();

    ibmmpy::GazeData msg;
    msg.header.stamp = e.current_real;
    msg.applied_offset.x = offset[0];
    msg.applied_offset.y = offset[1];
    std::transform(current_cache.begin(), current_cache.end(), std::back_inserter(msg.world_data), 
        [&offset] (MessageType const & msg) {
        ibmmpy::GazeDataPoint data_point;
        data_point.header.stamp = msg.first;
        data_point.position.x = msg.second.position_xy[0] + offset[0];
        data_point.position.y = msg.second.position_xy[1] + offset[1];
        data_point.confidence = 1.0;
        return data_point;
    });

    this->pub.publish(msg);
}



const ros::Duration TobiiConnection::SYNC_PERIOD(30);

TobiiConnection::TobiiConnection() :
    callback(callback), is_registered(false) {
    // connect to api
    tobii_error_t error = tobii_api_create( &this->api, NULL, NULL );
    if (error != TOBII_ERROR_NO_ERROR ) {
        ROS_FATAL_STREAM("failed to create tobii api: " << tobii_error_message(error));
        throw TobiiException(error);
    }

    std::vector<std::string> urls;
    error = tobii_enumerate_local_device_urls( this->api, receiveUrl, &urls );
    if (error != TOBII_ERROR_NO_ERROR) {
        ROS_FATAL_STREAM("failed to find local devices: " << tobii_error_message(error));
        tobii_api_destroy(this->api);
        throw TobiiException(error);
    } else if (urls.empty()) {
        ROS_FATAL_STREAM("failed to obtain any device urls");
        tobii_api_destroy(this->api);
        throw TobiiException(error);
    }

    // TODO: add some filtering code here, if we want
    // based on rosparams probably
    // only needed if we connect multiple devices to the same computer

    error = tobii_device_create( api, urls[0].c_str(), &device );
    if (error != TOBII_ERROR_NO_ERROR) {
        ROS_FATAL_STREAM("failed to connect to device " << urls[0] << ": " << tobii_error_message(error) );
        tobii_api_destroy(this->api);
        throw TobiiException(error);
    }

    ROS_INFO_STREAM("Connected to tobii device at " << urls[0]);
}


TobiiConnection::~TobiiConnection() {
    tobii_error_t error;

    error = tobii_gaze_point_unsubscribe( this->device );
    if ( error != TOBII_ERROR_NO_ERROR ) {
        // don't throw in a destructor!
        ROS_ERROR_STREAM("failed to unsubscribe from device: " << tobii_error_message(error));
    }

    error = tobii_device_destroy( this->device );
    if ( error != TOBII_ERROR_NO_ERROR ) {
        // don't throw in a destructor!
        ROS_ERROR_STREAM("failed to destroy device: " << tobii_error_message(error));
    }


    error = tobii_api_destroy( this->api );
    if ( error != TOBII_ERROR_NO_ERROR ) {
        // don't throw in a destructor!
        ROS_ERROR_STREAM("failed to destroy api: " << tobii_error_message(error));
    }
}


void TobiiConnection::setCallback(TobiiConnection::CallbackType cb) {
    this->callback = cb;

    if (!this->is_registered) {
        // set up the subscriber
        tobii_error_t error = tobii_gaze_point_subscribe( this->device, &TobiiConnection::dataCallbackCaller, this );
        
        if (error != TOBII_ERROR_NO_ERROR) {
            ROS_FATAL_STREAM("failed to subscribe to device: " << tobii_error_message(error));
            throw TobiiException(error);
        }
        this->is_registered = true;
    }
}

void TobiiConnection::runOnce() {
    tobii_error_t error;
    error = tobii_wait_for_callbacks( 1, &this->device );
    if ( error != TOBII_ERROR_NO_ERROR && error != TOBII_ERROR_TIMED_OUT ) {
        ROS_ERROR_STREAM("failed to wait for device callbacks: " << tobii_error_message(error));
        throw TobiiException(error);
    }

    error = tobii_device_process_callbacks( device );
    if ( error != TOBII_ERROR_NO_ERROR) {
        ROS_ERROR_STREAM("failed to process device callbacks: " << tobii_error_message(error));
        throw TobiiException(error);
    }

    // check if we need to synchronize the time
    // documentation says: every 30 s
    ros::Time cur_time = this->getSystemTime();
    if (  cur_time - this->last_sync_time > TobiiConnection::SYNC_PERIOD ) {
        ROS_DEBUG("syncing system time");
        error = tobii_update_timesync(this->device);
        if ( error != TOBII_ERROR_NO_ERROR) {
            ROS_ERROR_STREAM("failed to sync device clock: " << tobii_error_message(error));
            throw TobiiException(error);
        }
        this->last_sync_time = cur_time;

        // while we're at it, update the office between tobii system time and ros time
        ros::Time ros1 = ros::Time::now();
        ros::Time dev = this->getSystemTime();
        ros::Time ros2 = ros::Time::now();
        this->time_offset = (ros1 - dev) * 0.5 + (ros2 - dev) * 0.5;
    }
}

ros::Time TobiiConnection::getSystemTime() {
    int64_t time;
    tobii_error_t error = tobii_system_clock(this->api, &time);
    if ( error != TOBII_ERROR_NO_ERROR) {
        ROS_ERROR_STREAM("failed to get device time: " << tobii_error_message(error));
        throw TobiiException(error);
    }
    return tobiiTimeToRos(time);
}

tobii_bar_node::CalibrationInfo TobiiConnection::getCalibrationInfo() {
    std::vector<char> data;
    tobii_error_t error = tobii_calibration_retrieve(this->device, &TobiiConnection::receiveCalibrationInfo, &data);
    if ( error != TOBII_ERROR_NO_ERROR) {
        ROS_ERROR_STREAM("failed to get calibration data time: " << tobii_error_message(error));
        throw TobiiException(error);
    }

    tobii_bar_node::CalibrationInfo calibration_info;
    error = tobii_get_state_uint32(this->device, TOBII_STATE_CALIBRATION_ID, &calibration_info.calibration_id);
    if ( error != TOBII_ERROR_NO_ERROR) {
        ROS_ERROR_STREAM("failed to retrieve calibration id time: " << tobii_error_message(error));
        throw TobiiException(error);
    }

    error = tobii_calibration_parse(this->api, data.data(), data.size(), &TobiiConnection::parseCalibrationInfo, &calibration_info);
    if ( error != TOBII_ERROR_NO_ERROR) {
        ROS_ERROR_STREAM("failed to parse calibration data time: " << tobii_error_message(error));
        throw TobiiException(error);
    }

    return calibration_info;
}

uint8_t convert_calibration_point_status(tobii_calibration_point_status_t status) {
    switch (status) {
        case TOBII_CALIBRATION_POINT_STATUS_FAILED_OR_INVALID:
            return tobii_bar_node::CalibrationInfo::TOBII_CALIBRATION_POINT_STATUS_FAILED_OR_INVALID;
        case TOBII_CALIBRATION_POINT_STATUS_VALID_BUT_NOT_USED_IN_CALIBRATION:
            return tobii_bar_node::CalibrationInfo::TOBII_CALIBRATION_POINT_STATUS_VALID_BUT_NOT_USED_IN_CALIBRATION;
        case TOBII_CALIBRATION_POINT_STATUS_VALID_AND_USED_IN_CALIBRATION:
            return tobii_bar_node::CalibrationInfo::TOBII_CALIBRATION_POINT_STATUS_VALID_AND_USED_IN_CALIBRATION;
        default:
            return tobii_bar_node::CalibrationInfo::TOBII_CALIBRATION_POINT_STATUS_UNKNOWN;
    }
}

void TobiiConnection::parseCalibrationInfo(tobii_calibration_point_data_t const* point_data, void* user_data) {
    tobii_bar_node::CalibrationInfo * const calibration_info_ptr = static_cast<tobii_bar_node::CalibrationInfo *>(user_data);
    geometry_msgs::Point screen_point; screen_point.x = point_data->point_xy[0]; screen_point.y = point_data->point_xy[1];
    calibration_info_ptr->screen_point.push_back(screen_point);

    calibration_info_ptr->left_status.push_back(convert_calibration_point_status(point_data->left_status));
    geometry_msgs::Point left_point; left_point.x = point_data->point_xy[0]; left_point.y = point_data->point_xy[1];
    calibration_info_ptr->left_points.push_back(left_point);

    calibration_info_ptr->right_status.push_back(convert_calibration_point_status(point_data->right_status));
    geometry_msgs::Point right_point; right_point.x = point_data->point_xy[0]; right_point.y = point_data->point_xy[1];
    calibration_info_ptr->right_points.emplace_back(right_point);
}

int main(int argc, char* argv[]) {
    // init ROS node
    ros::init(argc, argv, "tobii_bar_node");
    ros::NodeHandle nh("~");

    TobiiConnection connection;
    OffsetManager offset_manager("set_offset", nh);

    // apparently i forgot how to scope c++ in a way that doesn't suck
    boost::scoped_ptr<BasicPublisher> basic_ptr;
    boost::scoped_ptr<BatchingPublisher> batching_ptr;

    double pub_rate;
    if (nh.getParam("batch_period", pub_rate) && pub_rate > 0.) {
        batching_ptr.reset(new BatchingPublisher("gaze_data", connection, offset_manager, ros::Duration(pub_rate)));
        batching_ptr->doSetup();
    } else {
        basic_ptr.reset(new BasicPublisher("gaze_data", connection, offset_manager));
        basic_ptr->doSetup();
    }
    
    ros::ServiceServer calibration_server = nh.advertiseService<tobii_bar_node::CalibrationInfoSrv::Request, tobii_bar_node::CalibrationInfoSrv::Response>("get_calibration_info",
        [&connection] (tobii_bar_node::CalibrationInfoSrv::Request const & req, tobii_bar_node::CalibrationInfoSrv::Response & resp) {
            try {
                resp.calibration_info = connection.getCalibrationInfo();
                resp.ok = true;
            } catch (TobiiException & ex) {
                resp.ok = false;
                resp.msg = ex.what();
            }
            return true;
        });

    // process ros stuff in a separate thread as tobii stuff, just in case
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok()) {
        connection.runOnce();
    }
}
