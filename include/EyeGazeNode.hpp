#include <tobii/tobii.h>
#include <tobii/tobii_config.h>
#include <tobii/tobii_streams.h>

#include <string>
#include <vector>
#include <stdexcept>

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>

#include "ros/ros.h"
#include "ros/console.h"
#include "ibmmpy/GazeData.h"
#include "tobii_bar_node/CalibrationInfo.h"

#ifndef __EYEGAZENODE_HPP__
#define __EYEGAZENODE_HPP__

struct TobiiException : public std::runtime_error {
    public:
        TobiiException(tobii_error_t error) :
            std::runtime_error(tobii_error_message(error)),
            error(error) {}
        tobii_error_t const error;
};

inline void receiveUrl( char const* url, void* user_data ) {
    static_cast<std::vector<std::string> * >(user_data)->push_back(url);
};

inline ros::Time tobiiTimeToRos(int64_t const & tm) {
    return ros::Time(tm / 1000000, static_cast<int32_t>(tm % 1000000) * 1000);
};



struct TobiiConnection {
    public:
        typedef boost::function<void(ros::Time const &, tobii_gaze_point_t const &)> CallbackType;

        static const ros::Duration SYNC_PERIOD;

        TobiiConnection();

        virtual ~TobiiConnection();

        void setCallback(CallbackType cb);

        void runOnce();
        ros::Time getSystemTime();

        tobii_bar_node::CalibrationInfo getCalibrationInfo();

    private:
        inline static void dataCallbackCaller(tobii_gaze_point_t const * data, void * user_data) {
            static_cast<TobiiConnection *>(user_data)->dataCallback(*data);
        }
        inline void dataCallback(tobii_gaze_point_t const & data) {
            if (data.validity == TOBII_VALIDITY_VALID && static_cast<bool>(this->callback)) {
                ROS_DEBUG_THROTTLE(15, "received valid data point");
                ros::Time recv_time = tobiiTimeToRos(data.timestamp_us) + this->time_offset;
                this->callback(recv_time, data);
            } 
        }
        inline static void receiveCalibrationInfo(void const* data, size_t size, void* user_data) {
            static_cast<std::vector<char> *>(user_data)->assign(static_cast<char const *>(data), static_cast<char const *>(data)+size);
        }
        static void parseCalibrationInfo(tobii_calibration_point_data_t const* point_data, void* user_data);


        std::string url;
        tobii_api_t * api;
        tobii_device_t * device;
        CallbackType callback;
        ros::Time last_sync_time;
        ros::Duration time_offset;
        bool is_registered;
};

struct BasicPublisher {
    public:

        BasicPublisher(std::string const & topic_name, TobiiConnection & connection);

        // we don't have access to this in the constructor
        // so make this a separate function :(
        inline void doSetup() {
            this->connection.setCallback(boost::bind(&BasicPublisher::processData, this, _1, _2));
        }

        void processData(ros::Time const & recv_time, tobii_gaze_point_t const & gaze_point);

    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        TobiiConnection & connection;
};

struct BatchingPublisher {
    public:
        typedef std::pair<ros::Time, tobii_gaze_point_t>  MessageType;
        typedef std::deque<MessageType> QueueType;

        BatchingPublisher(std::string const &topic_name, TobiiConnection &connection, ros::Duration const & batch_rate);

        // we don't have access to this in the constructor
        // so make this a separate function :(
        inline void doSetup()
        {
            this->connection.setCallback(boost::bind(&BatchingPublisher::processData, this, _1, _2));
            this->timer = this->nh.createTimer(this->rate, boost::bind(&BatchingPublisher::sendMessage, this, _1));
        }
        void processData(ros::Time const &recv_time, tobii_gaze_point_t const &gaze_point);
        void sendMessage(ros::TimerEvent const & e);

    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        TobiiConnection &connection;

        boost::mutex cache_mutex;
        boost::scoped_ptr<QueueType> cache_ptr;
        ros::Duration rate;
        ros::Timer timer;
};

#endif // __EYEGAZENODE_HPP__
