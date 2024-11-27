## yarp-devices-ros

> [!WARNING]  
> The functionality that provide compatibility between YARP and ROS 1 are deprecated and will be removed in YARP 3.11 . Please migrate to use https://github.com/robotology/yarp-devices-ros2 instead.
> This repository permit to continue to use ROS 1 devices with YARP 3.10, but will not work with YARP 3.11 .

This repository contains the following devices that used to be part of the main YARP repository, and were moved in this repo in YARP 3.9 :
* `controlBoard_nws_ros`
* `frameGrabber_nws_ros`
* `RGBDSensorFromRosTopic`
* `rgbdSensor_nws_ros`
* `map2D_nws_ros`
* `IMURosPublisher`
* `WrenchStampedRosPublisher`
* `TemperatureRosPublisher`
* `PoseStampedRosPublisher`
* `MagneticFieldRosPublisher`
* `laserFromRosTopic`
* `rgbdToPointCloudSensor_nws_ros`
* `frameTransformGet_nwc_ros`
* `frameTransformSet_nwc_ros`
* `localization2D_nws_ros`
* `odometry2D_nws_ros`
* `rangefinder2D_nws_ros`

This is quite a standard CMake repository that can be installed using the usual CMake process for installing packages.
