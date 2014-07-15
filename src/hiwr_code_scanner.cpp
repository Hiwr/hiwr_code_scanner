/*********************************************************************
*
*
* Copyright 2014 Worldline
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
* 
*     http://www.apache.org/licenses/LICENSE-2.0
* 
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
* 
***********************************************************************/

#include "hiwr_code_scanner.h"

using namespace zbar;

namespace hiwr_code_scanner
{

HiwrCodeScannerNodelet::HiwrCodeScannerNodelet():
    spinning_state_(true), //Wait state
    debug_name_("[Nodelet hiwr_code_scanner]"),
    video_stream_name_("output_video"),
    symbol_type_(ZBAR_NONE),
    new_data_available_(false),
    is_subscribed_(false),
    out_msg_(new std_msgs::String)
{}

void HiwrCodeScannerNodelet::onInit(){
    ros::NodeHandle& public_nh = getNodeHandle();
    ros::NodeHandle& private_nh = getMTPrivateNodeHandle();
    it_ = new image_transport::ImageTransport(public_nh);

    if(!private_nh.getParam("video_stream", video_stream_name_)){
        print_error("unable to retrieve [video_stream] param, please check corresponding launch file");
        return;
    }

    int symbol_value = 0;
    if(!private_nh.getParam("symbol_type", symbol_value)){
        print_info("unable to retrieve [symbol_type] param, please check corresponding launch file");
    }else{
        symbol_type_ = static_cast<zbar_symbol_type_t>(symbol_value);
        print_info("setting symbol_type");
    }
    pub_ = private_nh.advertise<std_msgs::String>("code", 1);
    image_sub_ = it_->subscribe(video_stream_name_.c_str(), 1,&HiwrCodeScannerNodelet::callback_img, this);
    is_subscribed_ = true;

    configureSpinning(private_nh);

    //Main loop thread
    loop_thread_ = std::thread(&HiwrCodeScannerNodelet::loop , this);

    print_info("Initialization OK");
}

void HiwrCodeScannerNodelet::callback_img(const sensor_msgs::ImageConstPtr& msg){
    try
    {
        cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        ImageScanner scanner;

        //configure the reader
        scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 0); //Set flags off
        scanner.set_config(symbol_type_, ZBAR_CFG_ENABLE, 1); //Set flags we need on

        //obtain image data
        int width =cv_ptr_->image.cols;
        int height =cv_ptr_->image.rows;
        uchar *raw = (uchar *)cv_ptr_->image.data;

        // wrap image data
        Image image(width, height, "Y800", raw, width * height);

        // scan the image for barcodes
        int n = scanner.scan(image);

        //if we got something
        if(n > 0){ //FIXME if n==1 for unit reading
            // extract results
            for(Image::SymbolIterator symbol = image.symbol_begin();
                symbol != image.symbol_end();
                ++symbol) {
                // do something useful with results
                //print_info(symbol->get_data().c_str());
                out_msg_->data = symbol->get_data();
                new_data_available_ = true;
            }
        }

        //clean up
        image.set_data(NULL, 0);
    }
    catch (cv_bridge::Exception& e)
    {
        NODELET_INFO("cv_bridge exception: %s", e.what());
        return;
    }

}

//Main loop
void HiwrCodeScannerNodelet::loop(){
    print_info("I'm gonna loop...");
    while(ros::ok()){
        if(spinning_state_){

            //do we need to subscribe again?
            if(!is_subscribed_){
                image_sub_ = it_->subscribe(video_stream_name_.c_str(), 1,&HiwrCodeScannerNodelet::callback_img, this);
                is_subscribed_ = true;
            }

            if(new_data_available_){
                //publish bar code value if a new one is available
                pub_.publish(out_msg_);
                new_data_available_ = false;
            }
            else{
                usleep(100000);
            }
        }
        else{
            //unsubscribe to video_stream
            if(is_subscribed_){
                image_sub_.shutdown();
                is_subscribed_ = false;
            }
            usleep(100000);
        }
    }
    print_info("I'm done loopping");
}

void HiwrCodeScannerNodelet::configureSpinning(ros::NodeHandle& nh){
    print_info("START configuring spinning state");
    service_spinning_state_setter_ = nh.advertiseService("setSpinningState", &HiwrCodeScannerNodelet::serviceSetSpinningState, this);
    service_spinning_state_getter_ = nh.advertiseService("getSpinningState", &HiwrCodeScannerNodelet::serviceGetSpinningState, this);
    print_info("configuring DONE");
}


bool HiwrCodeScannerNodelet::serviceSetSpinningState( hyve_msg::SetState::Request &req ,hyve_msg::SetState::Response  &res  ){
    res.state = req.state;
    spinning_state_ = req.state;
    return true;
}

bool HiwrCodeScannerNodelet::serviceGetSpinningState( hyve_msg::GetState::Request &req ,hyve_msg::GetState::Response  &res  ){
    res.state =spinning_state_;
    return true;
}

void HiwrCodeScannerNodelet::printInfo(const char * value){
    NODELET_INFO("%s %s", debug_name_.c_str(), value);
}

void HiwrCodeScannerNodelet::printError(char * value){
    NODELET_ERROR("%s %s", debug_name_.c_str(), value);
}

PLUGINLIB_DECLARE_CLASS(hiwr_code_scanner,HiwrCodeScannerNodelet, hiwr_code_scanner::HiwrCodeScannerNodelet, nodelet::Nodelet);
}
