hiwr\_code\_scanner
===============================================

The hiwr\_code\_scanner aims to read a video stream and retrieve symbols, like qr codes or common bar codes. It can retrieve the following formats, thanks to [Zbar library](http://zbar.sourceforge.net/):

* EAN-8             
* UPC-E               
* ISBN-10 (from EAN-13)
* UPC-A               
* EAN-13              
* ISBN-13 (from EAN-13)
* Interleaved 2 of 5
* Code 39          
* PDF417               
* QR Code             
* Code 128

Contributing
----------------------

Contributions via pull request are welcome and may be included under the
same license as below.

Copyright
----------------------

hiwr\_code\_scanner, except where otherwise noted, is released under the
[Apache License 2.0](http://www.apache.org/licenses/LICENSE-2.0.html).
See the LICENSE file located in the root directory.

How it works
----------------------

The node take in input a video stream, provided for instance by [hiwr_camera_controller](http://hiwr_camera_controller_page.com). Each image is analyzed and the first recognized symbol is published to a specific topic.

In order to improve general performances, the node is defined as a [nodelet](http://wiki.ros.org/nodelet). It shares memory with the camera node, also defined as a nodelet. The main result is that there is no recopy of images between nodes.

Build
----------------------
build instructions

Execution
----------------------

To start hiwr_code_scanner, do the following (assuming, you
have a working ROS core running, a touchscreen using ts library):

Make sure  your project is compiled and sourced.

Launch with an existing video stream topic:

    <node pkg="nodelet" type="nodelet" name="bar\_code" args="load bar_code_nodelet/Bar\_code\_node nodelet\_manager">
        <param name="video\_stream" type="str" value="/camStream/output\_video" />
        <rosparam file="$(find hyve\_bar\_code\_nodelet)/param/config\_Bar\_Code.yaml"/>
    </node>

Node
----------------------

### Subscribed Topics

- `/myWebcam/output_video`
 	  * The video stream of the camera


### Published Topics

- `output_bar_code`
	  * Send every detected bar code


### Services

- `/bar_code/setSpinningState` (Bool)
	  * Enable or disable detection

- `/bar_code/getSpinningState` (Bool)
	  * Get detection state

### Parameters

- `symbol_type` (int)
	  define the symbol_type to retrieve, 0 means all