# Jessicar I 

## RCCar with ROS Molodic + OpenCV + Yolo4-tiny

Base code: https://github.com/Road-Balance/donkey_ros      

There's Notion Lecture Notes but, It's written in Korean. 
Anyway, Here's the link

* [Notion Lecture Notes] https://www.notion.so/zeta7/JessiCar-1449b3fd5c984bab816920cb2b92002d

## Tested System information

**Jetson Nano 4GB or 2GB + IMX219 160 CSI camera**
* Jetpack 4.5.1 or 4.6.1
* Ubuntu 18.04
* ROS Melodic
* Opencv3.4.6 downgrade for darknet_ros

## Packages with Brief Explanation

```
├── jessicar_camera => Handling Image data For IMX219 Camera  
├── jessicar_control => Control RC Car with Adafruit PCA9685
├── jessicar_cv => Computer Vision Package with Opencv4 
├── jessicar_joy => Control RC Car with Logitech F710 Game Controller 
│
(...)
├── Images
├── LICENSE
├── README.md
```

## Prerequisite
Clone these Repo

```bash
$ cd ~/catkin_ws/src

Jessicar project code   
$ git clone https://github.com/zeta0707/jessicar.git   

darknet_ros
git clone --recursive https://github.com/Tossy0423/yolov4-for-darknet_ros.git

Custom Yolo4 train result       
$ git clone https://github.com/zeta0707/darknet_ros_custom.git

```

## Run script for selecting RCcar type
```bash
Usage: src/jessicar/script/jetRccarParam.sh target   
target: select one among these   
jetracer, jetbot, motorhat2wheel, motorhatSteer, nuriBldc, 298n2Wheel   
```

if you select waveshare jetracer
```bash
$ cd ~/catkin_ws/src/jessicar/script
$ ./jetRccarParam.sh jetracer
```

## Run script for selecting camera type
```bash
Usage: ./camSelect.sh target
target: select one between these
csicam, usbcam
```

if you select usbcam
```bash
$ cd ~/catkin_ws/src/jessicar/script
$ ./camSelect.sh usbcam
```

## Usage

### 1. **jessicar_camera package**

Packages for Image Streaming

> Check Camera Connection First!!!

**Using Gstreamer**
```bash
gst-launch-1.0 nvarguscamerasrc sensor_id=0 ! \
   'video/x-raw(memory:NVMM),width=3280, height=2464, framerate=21/1, format=NV12' ! \
   nvvidconv flip-method=0 ! 'video/x-raw,width=960, height=720' ! \
   nvvidconv ! nvegltransform ! nveglglessink -e
```

* `sensor_id` : this value depends on Camera Slot in Jetson Nano.

**Using ROS python**

```bash
# terminal #1
$ roslaunch jessicar_camera csicam.launch
# or
$ roslaunch jessicar_camera usbcam.launch

# terminal #2, PC or Jetson
$ rqt_image_view
```

### 2. **jessicar_control package**

Packages for controlling `RC Car` with `PCA9685` PWM driver.
You need to install `Adafruit_PCA9685` python package first 

There's four modes for controlling RC Car

* JoyStick Control
* Keyboard Control
* Blob Tracking
* Yolo4 object tracking
* Yolov4 traffic sign react

### 3. **jessicar_joy package**

There's two modes for using joystick -- delete Button mode

* Axes mode


### 4. **jessicar_cv package**

Packages for OpenCV applications

* Find Blob with Certain color
* Publish Image location as a `geometry_msgs/Point`



## Application

### **1. joy_control**

Control RC Car with game controller

<p align="center">
    <img src="./Images/joy_control.gif" width="500" />
</p>

```bash
#  terminal #1
$ roslaunch jessicar_control joy_control.launch

# terminal #2
$ roslaunch jessicar_joy jessicar_teleop_joy.launch

```

### **2. keyboard_control**

Control RC Car with keyboard

<p align="center">
    <img src="./Images/keyboard_control.gif" width="500" />
</p>

```bash
# terminal #1
$ roslaunch jessicar_control keyboard_control.launch

# terminal #2
$ roslaunch jessicar_teleop jessicar_teleop_key.launch

```

### **3. blob_tracking**

Find the any color box of the Jetson Nano on the screen and change the direction of the wheel accordingly.


<p align="center">
    <img src="./Images/blob_tracking.gif" width="500" />
</p>


```bash
# Jetson
$ roslaunch jessicar_control blob_all.launch
```

Debugging with `image_view`

```bash
# Jetson, but PC is better
rosrun image_view image_view image:=/image_raw
rosrun image_view image_view image:=/blob/image_mask
rosrun image_view image_view image:=/blob/image_blob
```

### **4. Yolo4_tracking**

Find the object of the Jetson Nano on the screen and change the direction of the wheel accordingly.


<p align="center">
    <img src="./Images/yolo_control.gif" width="500" />
</p>


```bash
#terminal #1
#object detect using Yolo_v4
zeta@zeta-nano:~/catkin_ws$ roslaunch darknet_ros yolo_v4.launch

#terminal #2, camera publish, object x/y -> car move
zeta@zeta-nano:~/catkin_ws$ roslaunch jessicar_control yolo_chase.launch 

```

### **5. Yolo4 traffic signal**

Train traffic signal, then Jetson nano will react to the traffic signal

<p align="center">
    <img src="./Images/Yolo_traffic.gif" width="500" />
</p>

```bash
#terminal #1, #object detect using Yolo_v4
zeta@zeta-nano:~/catkin_ws$ roslaunch darknet_ros yolov4_jessicar.launch

#terminal #2,camera publish, object -> start or stop
zeta@zeta-nano:~/catkin_ws$ roslaunch jessicar_control traffic_all.launch
```
