///////////////////////////////////////////////////////////////////////////\file
///
///   Copyright 2018 SINTEF AS
///
///   This Source Code Form is subject to the terms of the Mozilla
///   Public License, v. 2.0. If a copy of the MPL was not distributed
///   with this file, You can obtain one at https://mozilla.org/MPL/2.0/
///
////////////////////////////////////////////////////////////////////////////////

#ifndef __BASLER_TOF_WRAPPER_HPP
#define __BASLER_TOF_WRAPPER_HPP

#include <thread>

#include <ConsumerImplHelper/ToFCamera.h>

// Does sampling operation, returns true if more samples are requested.
typedef std::function<bool(const uint16_t* depth,
                           const uint16_t* confidence,
                           int width,
                           int height)> Operation;

using namespace GenTLConsumerImplHelper;

class BaslerToFWrapper
{
public:

  BaslerToFWrapper(std::string camera_name, Operation operation);
  ~BaslerToFWrapper();

  void Start();
  void Stop();

  int64_t getWidth();
  int64_t getHeight();
  int64_t getOffsetX();
  int64_t getOffsetY();

  int64_t maxWidth();
  int64_t maxHeight();

  void setWidth(int64_t value);
  void setHeight(int64_t value);
  void setOffsetX(int64_t value);
  void setOffsetY(int64_t value);

  float getTriggerRate();
  float minTriggerRate();
  float maxTriggerRate();
  void setTriggerRate(float rate);

  void setTriggerMode(bool enable);
  void setTriggerSource(std::string line);

  int64_t getMaxDepth();
  void setMaxDepth(int64_t depth);
  int64_t getMaxDepth_upper_limit();

  int64_t getMinDepth();
  void setMinDepth(int64_t depth);
  int64_t getMinDepth_lower_limit();


private:

  const Operation operation_;

  std::string getEnum(const char* name);
  void setEnum(const char* name, std::string value);

  int64_t getInt(const char* name);
  int64_t maxInt(const char* name);
  int64_t minInt(const char* name);
  void setInt(const char* name, int64_t value);

  float getFloat(const char* name);
  float maxFloat(const char* name);
  float minFloat(const char* name);
  void setFloat(const char* name, float value);

  void setSelector(std::string component, bool value);

  bool HandleResult(GrabResult result, BufferParts parts);
  void SampleLoop();

  CToFCamera camera_;

  std::thread sampler_;
  bool running_;
};

#endif
