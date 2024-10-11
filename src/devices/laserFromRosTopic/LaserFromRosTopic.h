/*
 * SPDX-FileCopyrightText: 2006-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef LASER_FROM_ROS_TOPIC_H
#define LASER_FROM_ROS_TOPIC_H

#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Port.h>
#include <yarp/os/Subscriber.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IRangefinder2D.h>
#include <yarp/dev/Lidar2DDeviceBase.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/PolyDriver.h>

 // ROS state publisher
#include <yarp/os/Node.h>
#include <yarp/os/Subscriber.h>
#include <yarp/rosmsg/sensor_msgs/LaserScan.h>
#include <yarp/rosmsg/impl/yarpRosHelper.h>

#include <mutex>
#include <string>
#include <vector>

// Compatibility with both YARP 3.9 and 3.10
// See https://github.com/robotology/yarp/pull/3130
#if __has_include (<yarp/dev/LaserScan2D.h>)

// YARP 3.9 case
#include <yarp/dev/LaserScan2D.h>

namespace yarp {
namespace sig {
typedef yarp::dev::LaserScan2D LaserScan2D;
}
}

#else

// YARP 3.10 case
#include <yarp/sig/LaserScan2D.h>

#endif


typedef unsigned char byte;

//---------------------------------------------------------------------------------------------------------------
enum base_enum
{
    BASE_IS_NAN = 0,
    BASE_IS_INF = 1,
    BASE_IS_ZERO = 2
};

class InputPortProcessor :
    public yarp::os::Subscriber<yarp::rosmsg::sensor_msgs::LaserScan>
{
    std::mutex             m_port_mutex;
    yarp::sig::LaserScan2D m_lastScan;
    yarp::os::Stamp        m_lastStamp;
    bool                   m_contains_data;

public:
    InputPortProcessor(const InputPortProcessor& alt) :
            yarp::os::Subscriber<yarp::rosmsg::sensor_msgs::LaserScan>(),
            m_lastScan(alt.m_lastScan),
            m_lastStamp(alt.m_lastStamp),
            m_contains_data(alt.m_contains_data)
    {
    }

    InputPortProcessor();
    using yarp::os::Subscriber<yarp::rosmsg::sensor_msgs::LaserScan>::onRead;
    virtual void onRead(yarp::rosmsg::sensor_msgs::LaserScan& v) override;
    void getLast(yarp::sig::LaserScan2D& data, yarp::os::Stamp& stmp);
};

/**
 * @ingroup dev_impl_lidar
 *
 * \brief `laserFromRosTopic`: Documentation to be added
 */
class LaserFromRosTopic : public yarp::dev::Lidar2DDeviceBase,
                              public yarp::os::PeriodicThread,
                              public yarp::dev::DeviceDriver
{
protected:
    bool                            m_option_override_limits;
    std::vector <std::string>       m_port_names;
    yarp::os::Node*                 m_ros_node = nullptr;
    std::vector<InputPortProcessor> m_input_ports;
    std::vector <yarp::os::Stamp>        m_last_stamp;
    std::vector <yarp::sig::LaserScan2D> m_last_scan_data;
    yarp::dev::PolyDriver                m_tc_driver;
    yarp::dev::IFrameTransform*          m_iTc = nullptr;

    std::vector <std::string>            m_src_frame_id;
    std::string                          m_dst_frame_id;
    yarp::sig::Vector                    m_empty_laser_data;
    base_enum                            m_base_type;

    void calculate(yarp::sig::LaserScan2D scan, yarp::sig::Matrix m);

public:
    LaserFromRosTopic(double period = 0.01) : Lidar2DDeviceBase(), PeriodicThread(period)
    {
        m_option_override_limits=false;
        m_base_type = base_enum::BASE_IS_NAN;
    }

    ~LaserFromRosTopic()
    {
    }

    bool open(yarp::os::Searchable& config) override;
    bool close() override;
    bool threadInit() override;
    void threadRelease() override;
    void run() override;

public:
    //IRangefinder2D interface
    bool setDistanceRange        (double min, double max) override;
    bool setScanLimits           (double min, double max) override;
    bool setHorizontalResolution (double step) override;
    bool setScanRate             (double rate) override;

public:
    //Lidar2DDeviceBase
    bool acquireDataFromHW() override final;
};

#endif
