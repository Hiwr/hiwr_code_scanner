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



namespace hiwr_code_scanner
{

class HiwrCodeScannerNodelet : public nodelet::Nodelet{
private:
    //Debug variables & methods
    std::string debug_name_;
    void print_info(const char *value);
    void print_error(char * value);

    //Subscriber to UvcCamNodelet
    image_transport::Subscriber image_sub_;
    image_transport::ImageTransport * it_;
    std::string video_stream_name_;

    //Main loop thread
    std::thread loop_thread_;

    //Pointer to last image from video stream
    cv_bridge::CvImagePtr cv_ptr_;

    //Spinning state
    bool spinning_state_;

    bool new_data_available_;
    bool is_subscribed_;

    //Symbol type
    zbar::zbar_symbol_type_t symbol_type_;

    //Services
    ros::ServiceServer service_spinning_state_setter_;
    ros::ServiceServer service_spinning_state_getter_;

    //Publisher
    ros::Publisher pub_;
    std_msgs::StringPtr  out_msg_;

    //Main loop
    void loop();

    void callback_img(const sensor_msgs::ImageConstPtr& msg);
public:
    virtual void onInit();
    HiwrCodeScannerNodelet();
    bool serviceSetSpinningState( hyve_msg::SetState::Request&, hyve_msg::SetState::Response&);
    bool serviceGetSpinningState( hyve_msg::GetState::Request&, hyve_msg::GetState::Response&);
    void configureSpinning(ros::NodeHandle&);
    void printInfo(const char * value);
    void printError(char * value);

};


PLUGINLIB_DECLARE_CLASS(hiwr_code_scanner,HiwrCodeScannerNodelet, hiwr_code_scanner::HiwrCodeScannerNodelet, nodelet::Nodelet);
}
