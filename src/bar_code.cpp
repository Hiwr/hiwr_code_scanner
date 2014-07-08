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

#include "bar_code.h"

using namespace zbar;

namespace bar_code_nodelet
{

Bar_code_node::Bar_code_node():
    spinningState(true), //Wait state
    debug_name("[Nodelet Bar_Code]"),
    video_stream_name("output_video"),
    symbol_type(ZBAR_NONE),
    new_data_available(false),
    isSubscribed(false),
    out_msg(new std_msgs::String)
{}

void Bar_code_node::onInit(){
    ros::NodeHandle& public_nh = getNodeHandle();
    ros::NodeHandle& private_nh = getMTPrivateNodeHandle();
    it_ = new image_transport::ImageTransport(public_nh);

    if(!private_nh.getParam("video_stream", video_stream_name)){
        print_error("unable to retrieve [video_stream] param, please check corresponding launch file");
        return;
    }

    int symbol_value = 0;
    if(!private_nh.getParam("symbol_type", symbol_value)){
        print_info("unable to retrieve [symbol_type] param, please check corresponding launch file");
    }else{
        symbol_type = static_cast<zbar_symbol_type_t>(symbol_value);
        print_info("setting symbol_type");
    }
    pub = private_nh.advertise<std_msgs::String>("output_bar_code", 1);
    image_sub_ = it_->subscribe(video_stream_name.c_str(), 1,&Bar_code_node::callback_img, this);
    isSubscribed = true;

    configureSpinning(private_nh);

    //Main loop thread
    loop_thread = std::thread(&Bar_code_node::loop , this);

    print_info("Initialization OK");
}

void Bar_code_node::callback_img(const sensor_msgs::ImageConstPtr& msg){
    try
    {
        //FIXME We need a MONO input image, if not we need to convert it to Grayscale
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        ImageScanner scanner;

        //configure the reader
        scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 0); //Set flags off
        scanner.set_config(symbol_type, ZBAR_CFG_ENABLE, 1); //Set flags we need on

        //obtain image data
        int width =cv_ptr->image.cols;
        int height =cv_ptr->image.rows;
        uchar *raw = (uchar *)cv_ptr->image.data;

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
                out_msg->data = symbol->get_data();
                new_data_available = true;
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
void Bar_code_node::loop(){
    print_info("I'm gonna loop...");
    while(ros::ok()){
        if(spinningState){

            //do we need to subscribe again?
            if(!isSubscribed){
                image_sub_ = it_->subscribe(video_stream_name.c_str(), 1,&Bar_code_node::callback_img, this);
                isSubscribed = true;
            }

            if(new_data_available){
                //publish bar code value if a new one is available
                //NODELET_INFO("PUBLISHING NEW DATA");
                pub.publish(out_msg);
                new_data_available = false;
            }
            else{
                usleep(100000);
            }
        }
        else{
            //unsubscribe to video_stream
            if(isSubscribed){
                image_sub_.shutdown();
                isSubscribed = false;
            }
            usleep(100000);
        }
    }
    print_info("I'm done loopping");
}

void Bar_code_node::configureSpinning(ros::NodeHandle& nh){
    print_info("START configuring spinning state");
    serviceSpinningStateSetter = nh.advertiseService("setSpinningState", &Bar_code_node::service_SetSpinningState, this);
    serviceSpinningStateGetter = nh.advertiseService("getSpinningState", &Bar_code_node::service_GetSpinningState, this);
    print_info("configuring DONE");
}


bool Bar_code_node::service_SetSpinningState( hyve_msg::SetState::Request &req ,hyve_msg::SetState::Response  &res  ){
    res.state = req.state;
    spinningState = req.state;
    return true;
}

bool Bar_code_node::service_GetSpinningState( hyve_msg::GetState::Request &req ,hyve_msg::GetState::Response  &res  ){
    res.state =spinningState;
    return true;
}

void Bar_code_node::print_info(const char * value){
    NODELET_INFO("%s %s", debug_name.c_str(), value);
}

void Bar_code_node::print_error(char * value){
    NODELET_ERROR("%s %s", debug_name.c_str(), value);
}

PLUGINLIB_DECLARE_CLASS(bar_code_nodelet,Bar_code_node, bar_code_nodelet::Bar_code_node, nodelet::Nodelet);
}
