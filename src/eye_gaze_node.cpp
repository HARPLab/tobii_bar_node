#include "EyeGazeNode.hpp"


const ros::Duration BasicPublisher::SYNC_PERIOD(30);

BasicPublisher::BasicPublisher(std::string const & topic_name, TobiiConnection & connection) :
            nh(),
            pub(nh.advertise<ibmmpy::GazeData>(topic_name, 1)),
            time_offset(0), last_sync_time(0), connection(connection) {}

void BasicPublisher::processData(tobii_gaze_point_t const & gaze_point) {
    ros::Time cur_ros_time = ros::Time::now();
    if (this->last_sync_time == ros::Time(0) || cur_ros_time - this->last_sync_time > BasicPublisher::SYNC_PERIOD ) {
        this->updateOffset();
    }

    ros::Time recv_time = tobiiTimeToRos(gaze_point.timestamp_us);
    ibmmpy::GazeData msg;
    msg.header.stamp = recv_time;
    msg.world_data.push_back(ibmmpy::GazeDataPoint());
    msg.world_data[0].header.stamp = recv_time;
    msg.world_data[0].position.x = gaze_point.position_xy[0];
    msg.world_data[0].position.y = gaze_point.position_xy[1];
    msg.world_data[0].confidence = 1.0; // doesn't seem to be a way to get a confidence flag, and we're already filtering validity
    this->pub.publish(msg);
}

void BasicPublisher::updateOffset() {
    ros::Time ros1 = ros::Time::now();
    ros::Time dev = this->connection.getSystemTime();
    ros::Time ros2 = ros::Time::now();

    this->time_offset = ( ros1 - dev ) * 0.5 + (ros2 - dev) * 0.5;
    this->last_sync_time = ros1;
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


int main(int argc, char* argv[]) {
    // init ROS node
    ros::init(argc, argv, "tobii_bar_node");
    ros::NodeHandle n;

    TobiiConnection connection;
    BasicPublisher basic_pub("gaze_data", connection);
    basic_pub.doSetup();
    
    // process ros stuff in a separate thread as tobii stuff, just in case
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok()) {
        connection.runOnce();
    }
}
