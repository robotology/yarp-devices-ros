/*
 * Copyright (C) 2006-2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */


#ifndef YARP_DEV_TEMPERATUREROSPUBLISHER_H
#define YARP_DEV_TEMPERATUREROSPUBLISHER_H

#include "GenericSensorRosPublisher.h"
#include <yarp/rosmsg/sensor_msgs/Temperature.h>

    /**
 * @ingroup dev_impl_wrapper
 *
 * \brief This wrapper connects to a device and publishes a ROS topic of type sensor_msgs::Temperature.
 *
 * | YARP device name |
 * |:-----------------:|
 * | `TemperatureRosPublisher` |
 *
 * The parameters accepted by this device are:
 * | Parameter name | SubParameter   | Type    | Units          | Default Value | Required                    | Description                                                       | Notes |
 * |:--------------:|:--------------:|:-------:|:--------------:|:-------------:|:--------------------------: |:-----------------------------------------------------------------:|:-----:|
 * | name           |      -         | string  | -              |   -           | Yes                         | Prefix of the port opened by this device                          | MUST start with a '/' character |
 * | period         |      -         | int     | ms             |   -           | Yes                          | Refresh period of the broadcasted values in ms                    |  |
 */
class TemperatureRosPublisher : public GenericSensorRosPublisher<yarp::rosmsg::sensor_msgs::Temperature>
{
    // Interface of the wrapped device
    yarp::dev::ITemperatureSensors* m_ITemperature{ nullptr };

public:
    using GenericSensorRosPublisher<yarp::rosmsg::sensor_msgs::Temperature>::GenericSensorRosPublisher;

    using GenericSensorRosPublisher<yarp::rosmsg::sensor_msgs::Temperature>::open;
    using GenericSensorRosPublisher<yarp::rosmsg::sensor_msgs::Temperature>::close;

    using GenericSensorRosPublisher<yarp::rosmsg::sensor_msgs::Temperature>::attachAll;
    using GenericSensorRosPublisher<yarp::rosmsg::sensor_msgs::Temperature>::detachAll;

    /* RateThread methods */
    void run() override;

    bool viewInterfaces() override;
};

#endif

