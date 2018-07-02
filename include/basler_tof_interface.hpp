///////////////////////////////////////////////////////////////////////////\file
///
///   Copyright 2018 SINTEF AS
///
///   This Source Code Form is subject to the terms of the Mozilla
///   Public License, v. 2.0. If a copy of the MPL was not distributed
///   with this file, You can obtain one at https://mozilla.org/MPL/2.0/
///
////////////////////////////////////////////////////////////////////////////////

#ifndef __I3DS_BASLER_TOF_INTERFACE_HPP
#define __I3DS_BASLER_TOF_INTERFACE_HPP


#include <ConsumerImplHelper/ToFCamera.h>

#include <i3ds/topic.hpp>
#include <i3ds/tof_camera_sensor.hpp>

// #include "i3ds/camera_sensor.hpp"


// Does sampling operation, returns true if more samples are requested.
typedef std::function<bool(unsigned char *image, unsigned long timestamp_us)> Operation;

using namespace GenTLConsumerImplHelper;

class Basler_ToF_Interface
{
public:

  //typedef Topic<128, ToFMeasurement500KCodec> ToFMeasurement;

  void Basler_ToF_Interface2();
  Basler_ToF_Interface(const std::string & connectionString, std::string const &camera_name, bool free_running, Operation operation);
  ~Basler_ToF_Interface();
  void do_activate();
  void do_start();
  void do_stop();
  void StartSamplingLoop();
  void do_deactivate();

  int64_t getShutterTime();
  void setShutterTime(int64_t value);

  void setRegion(PlanarRegion region);
  PlanarRegion getRegion();


  bool getAutoExposureEnabled();
  void setAutoExposureEnabled(bool value);

  int64_t getGain();
  void setGain(int64_t value);


  void setRegionEnabled(bool regionEnabled);
  bool getRegionEnabled();

  int64_t getMaxShutterTime();
  void setMaxShutterTime(int64_t);

  void setTriggerInterval_in_Hz(float rate_in_Hz);
  bool checkTriggerInterval(int64_t);
  void setTriggerInterval();
  float getSamplingsRate();
  void setTriggerSourceToLine1();
  void setTriggerModeOn(bool value);

  int64_t getMaxDepth();
  void setMaxDepth(int64_t depth);

  int64_t getMinDepthLocalInMM();
  int64_t getMaxDepthLocalInMM();



  int64_t getMinDepth();
  void setMinDepth(int64_t depth);

  bool connect();



  bool onImageGrabbed(GrabResult grabResult, BufferParts);

private:
  CToFCamera m_Camera;
  int m_nBuffersGrabbed;

  std::thread threadSamplingLoop;
  bool stopSamplingLoop;
  const std::string mConnectionID;
  const std::string camera_name;

  bool free_running_;
  Operation operation_;
  float samplingsRate_in_Hz_;
  int64_t minDepth_;
  int64_t maxDepth_;


  typedef std::chrono::high_resolution_clock clock;
};

#endif
