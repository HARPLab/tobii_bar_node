#include <tobii/tobii.h>
#include <tobii/tobii_streams.h>
#include <cstdio>
#include <cassert>
#include <cstring>

#include "ros/ros.h"
#include "gaze_chatter/Gaze.h"

#define BoundY 0.4
#define BoundX12  0.2
#define BoundX23 0.8

int get_bin_fromxy(double x, double y)
{
    // Bot bin
    if (y < BoundY){
        return 0;
    }
    else{
        if (x < BoundX12)
            return 1;
        else if (x >= BoundX12 && x < BoundX23)
            return 2;
        else // (x >= BoundX23)
            return 3;
    }
}


void gaze_point_callback( tobii_gaze_point_t const* gaze_point, void* user_data )
{
    if( gaze_point->validity == TOBII_VALIDITY_VALID )
//        printf( "Gaze point: %f, %f\n",
//                gaze_point->position_xy[ 0 ],
//                gaze_point->position_xy[ 1 ] );
        ;
    else
        printf("Gaze point not valid!");

    tobii_gaze_point_t* gaze_point_storage = (tobii_gaze_point_t*) user_data;
    *gaze_point_storage = *gaze_point;

}

static void url_receiver( char const* url, void* user_data )
{
    char* buffer = (char*)user_data;
    if( *buffer != '\0' ) return; // only keep first value

    if( strlen( url ) < 256 )
        strcpy( buffer, url );
}

int main(int argc, char* argv[])
{
    tobii_api_t* api;
    tobii_error_t error = tobii_api_create( &api, NULL, NULL );
    assert( error == TOBII_ERROR_NO_ERROR );

    char url[ 256 ] = { 0 };
    error = tobii_enumerate_local_device_urls( api, url_receiver, url );
    assert( error == TOBII_ERROR_NO_ERROR && *url != '\0' );

    tobii_device_t* device;
    error = tobii_device_create( api, url, &device );
    assert( error == TOBII_ERROR_NO_ERROR );

    printf("Connected to Tobii succesfully");

    tobii_gaze_point_t latest_gaze_point;
    latest_gaze_point.timestamp_us = 0LL;
    latest_gaze_point.validity = TOBII_VALIDITY_INVALID;

    error = tobii_gaze_point_subscribe( device, gaze_point_callback, &latest_gaze_point );
    assert( error == TOBII_ERROR_NO_ERROR );

    ros::init(argc, argv, "gaze_chatter_tobii");
    ros::NodeHandle n;

    ros::Publisher gaze_chatter_pub = n.advertise<gaze_chatter::Gaze>("gaze_chatter", 1);
//    ros::Rate loop_rate(10);

    int count = 0;
    gaze_chatter::Gaze gaze_msg;
    int is_running = 1000; // in this sample, exit after some iterations


    while(ros::ok())
    {
        error = tobii_wait_for_callbacks( 1, &device );
        assert( error == TOBII_ERROR_NO_ERROR || error == TOBII_ERROR_TIMED_OUT );

        error = tobii_device_process_callbacks( device );
        assert( error == TOBII_ERROR_NO_ERROR );

//        if( latest_gaze_point.validity == TOBII_VALIDITY_VALID )
//        {
        printf("Gaze point: %f, %f\n",
               latest_gaze_point.position_xy[0],
               latest_gaze_point.position_xy[1]);

        // package gaze msg for ros nodes
        gaze_msg.x = latest_gaze_point.position_xy[0];
        gaze_msg.y = latest_gaze_point.position_xy[1];
        //gaze_msg.label = get_bin_fromxy(gaze_msg.x, gaze_msg.y);
        gaze_msg.header.stamp = ros::Time::now();

//        ROS_INFO("%s", msg.data.c_str());
        gaze_chatter_pub.publish(gaze_msg);

//        ros::spinOnce();
//        loop_rate.sleep();
        ++count;

    }

    error = tobii_gaze_point_unsubscribe( device );
    assert( error == TOBII_ERROR_NO_ERROR );

    error = tobii_device_destroy( device );
    assert( error == TOBII_ERROR_NO_ERROR );

    error = tobii_api_destroy( api );
    assert( error == TOBII_ERROR_NO_ERROR );
    return 0;
}
