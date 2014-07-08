//Common
#include <stdio.h>

//Ros
#include <ros/ros.h>

//Transport of images
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

//Nodelet
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

//OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//Zbar library
#include <zbar.h>

//Msgs
#include "std_msgs/String.h"

//Threads
#include <thread>

//Service states
#include <hyve_msg/SetState.h>
#include <hyve_msg/GetState.h>



namespace bar_code_nodelet
{

class Bar_code_node : public nodelet::Nodelet{
private:
    //Debug variables & methods
    std::string debug_name;
    void print_info(const char *value);
    void print_error(char * value);

    //Subscriber to UvcCamNodelet
    image_transport::Subscriber image_sub_;
    image_transport::ImageTransport * it_;
    std::string video_stream_name;

    //Main loop thread
    std::thread loop_thread;

    //Pointer to last image from video stream
    cv_bridge::CvImagePtr cv_ptr;

    //Spinning state
    bool spinningState;

    bool new_data_available;
    bool isSubscribed;

    //Symbol type
    zbar::zbar_symbol_type_t symbol_type;

    //Services
    ros::ServiceServer serviceSpinningStateSetter;
    ros::ServiceServer serviceSpinningStateGetter;

    //Publisher
    ros::Publisher pub;
    std_msgs::StringPtr  out_msg;

    //Main loop
    void loop();

    void callback_img(const sensor_msgs::ImageConstPtr& msg);
public:
    virtual void onInit();
    Bar_code_node();
    bool service_SetSpinningState( hyve_msg::SetState::Request&, hyve_msg::SetState::Response&);
    bool service_GetSpinningState( hyve_msg::GetState::Request&, hyve_msg::GetState::Response&);
    void configureSpinning(ros::NodeHandle&);

};


PLUGINLIB_DECLARE_CLASS(bar_code_nodelet,Bar_code_node, bar_code_nodelet::Bar_code_node, nodelet::Nodelet);
}
