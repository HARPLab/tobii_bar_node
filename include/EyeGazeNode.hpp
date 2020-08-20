#include <tobii/tobii.h>
#include <tobii/tobii_streams.h>

#include <string>
#include <vector>
#include <stdexcept>

#include <boost/function.hpp>
#include <boost/bind.hpp>

#include "ros/ros.h"
#include "ros/console.h"
#include "ibmmpy/GazeData.h"

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
        typedef boost::function<void(tobii_gaze_point_t const &)> CallbackType;

        static const ros::Duration SYNC_PERIOD;

        TobiiConnection();

        virtual ~TobiiConnection();

        void setCallback(CallbackType cb);

        void runOnce();
        ros::Time getSystemTime();

    private:
        inline static void dataCallbackCaller(tobii_gaze_point_t const * data, void * user_data) {
            static_cast<TobiiConnection *>(user_data)->dataCallback(*data);
        }
        inline void dataCallback(tobii_gaze_point_t const & data) {
            if (data.validity == TOBII_VALIDITY_VALID && static_cast<bool>(this->callback)) {
                ROS_DEBUG_THROTTLE(15, "received valid data point");
                this->callback(data);
            } 
        }


        std::string url;
        tobii_api_t * api;
        tobii_device_t * device;
        CallbackType callback;
        ros::Time last_sync_time;
        bool is_registered;
};

struct BasicPublisher {
    public:
        static const ros::Duration SYNC_PERIOD;

        BasicPublisher(std::string const & topic_name, TobiiConnection & connection);

        // we don't have access to this in the constructor
        // so make this a separate function :(
        inline void doSetup() {
            this->connection.setCallback(boost::bind(&BasicPublisher::processData, this, _1));
        }

        void processData(tobii_gaze_point_t const & gaze_point);

    private:
        void updateOffset();

        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Duration time_offset;
        ros::Time last_sync_time;
        TobiiConnection & connection;
};

#endif // __EYEGAZENODE_HPP__
