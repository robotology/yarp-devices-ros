# SPDX-FileCopyrightText: 2006-2023 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

add_library(RGBDRosConversionUtils OBJECT)

target_sources(RGBDRosConversionUtils
  PRIVATE
    RGBDRosConversionUtils.cpp
    RGBDRosConversionUtils.h
    rosPixelCode.h
    rosPixelCode.cpp
)

target_include_directories(RGBDRosConversionUtils PUBLIC ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(RGBDRosConversionUtils
  PRIVATE
    YARP::YARP_os
    YARP::YARP_sig
    YARP::YARP_rosmsg
    YARP::YARP_dev
)

set_property(TARGET RGBDRosConversionUtils PROPERTY FOLDER "Devices/Shared")
