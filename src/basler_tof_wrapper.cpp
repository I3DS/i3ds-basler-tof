///////////////////////////////////////////////////////////////////////////\file
///
///   Copyright 2018 SINTEF AS
///
///   This Source Code Form is subject to the terms of the Mozilla
///   Public License, v. 2.0. If a copy of the MPL was not distributed
///   with this file, You can obtain one at https://mozilla.org/MPL/2.0/
///
////////////////////////////////////////////////////////////////////////////////

#include "basler_tof_wrapper.hpp"

// TODO: Should be configured in CMake
#define GENICAM_GENTL64_PATH "/opt/BaslerToF/lib64/gentlproducer/gtl"

using namespace GenTLConsumerImplHelper;

#define BOOST_LOG_DYN_LINK

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

BaslerToFWrapper::BaslerToFWrapper(std::string camera_name, Operation operation)
  : operation_(operation)
{
  setenv("GENICAM_GENTL64_PATH", GENICAM_GENTL64_PATH, 1);

  CToFCamera::InitProducer();

  camera_.Open(UserDefinedName, camera_name);

  // These are fixed settings for I3DS.
  setSelector("Range", true);
  setEnum("PixelFormat", "Coord3D_C16");
  setSelector("Intensity", false);
  setSelector("Confidence", true);

  // TODO: Add option for HDR?
  setEnum("ExposureAuto", "Continuous");
}

BaslerToFWrapper::~BaslerToFWrapper()
{
  if (camera_.IsOpen())
    {
      camera_.Close();
    }

  if (CToFCamera::IsProducerInitialized())
    {
      CToFCamera::TerminateProducer();   // Won't throw any exceptions
    }
}

std::string
BaslerToFWrapper::getEnum(const char* name)
{
  GenApi::CEnumerationPtr ptr(camera_.GetParameter(name));
  return std::string(ptr->ToString());
}

void
BaslerToFWrapper::setEnum(const char* name, std::string value)
{
  GenApi::CEnumerationPtr ptr(camera_.GetParameter(name));
  ptr->FromString(value.c_str());
}

int64_t
BaslerToFWrapper::getInt(const char* name)
{
  GenApi::CIntegerPtr ptr(camera_.GetParameter(name));
  return ptr->GetValue();
}

int64_t
BaslerToFWrapper::minInt(const char* name)
{
  GenApi::CIntegerPtr ptr(camera_.GetParameter(name));
  return ptr->GetMin();
}

int64_t
BaslerToFWrapper::maxInt(const char* name)
{
  GenApi::CIntegerPtr ptr(camera_.GetParameter(name));
  return ptr->GetMax();
}

void
BaslerToFWrapper::setInt(const char* name, int64_t value)
{
  GenApi::CIntegerPtr ptr(camera_.GetParameter(name));
  ptr->SetValue(value);
}

float
BaslerToFWrapper::getFloat(const char* name)
{
  GenApi::CFloatPtr ptr(camera_.GetParameter(name));
  return ptr->GetValue();
}

float
BaslerToFWrapper::minFloat(const char* name)
{
  GenApi::CFloatPtr ptr(camera_.GetParameter(name));
  return ptr->GetMin();
}

float
BaslerToFWrapper::maxFloat(const char* name)
{
  GenApi::CFloatPtr ptr(camera_.GetParameter(name));
  return ptr->GetMax();
}

void
BaslerToFWrapper::setFloat(const char* name, float value)
{
  GenApi::CFloatPtr ptr(camera_.GetParameter(name));
  ptr->SetValue(value);
}

void
BaslerToFWrapper::setSelector(std::string component, bool enable)
{
  GenApi::CEnumerationPtr ptrSelector = camera_.GetParameter("ComponentSelector");
  GenApi::CBooleanPtr ptrEnable = camera_.GetParameter("ComponentEnable");

  ptrSelector->FromString(component.c_str());
  ptrEnable->SetValue(enable);
}

int64_t
BaslerToFWrapper::getWidth()
{
  return getInt("Width");
}

int64_t
BaslerToFWrapper::getHeight()
{
  return getInt("Height");
}

int64_t
BaslerToFWrapper::getOffsetX()
{
  return getInt("OffsetX");
}

int64_t
BaslerToFWrapper::getOffsetY()
{
  return getInt("OffsetY");
}

void
BaslerToFWrapper::setWidth(int64_t value)
{
  setInt("Width", value);
}

void
BaslerToFWrapper::setHeight(int64_t value)
{
  setInt("Height", value);
}

void
BaslerToFWrapper::setOffsetX(int64_t value)
{
  setInt("OffsetX", value);
}

void
BaslerToFWrapper::setOffsetY(int64_t value)
{
  setInt("OffsetY", value);
}

int64_t
BaslerToFWrapper::maxWidth()
{
  return maxInt("Width");
}

int64_t
BaslerToFWrapper::maxHeight()
{
  return maxInt("Height");
}

float
BaslerToFWrapper::getTriggerRate()
{
  return getFloat("AcquisitionFrameRate");
}

float
BaslerToFWrapper::minTriggerRate()
{
  return minFloat("AcquisitionFrameRate");
}

float
BaslerToFWrapper::maxTriggerRate()
{
  return maxFloat("AcquisitionFrameRate");
}

void
BaslerToFWrapper::setTriggerRate(float rate)
{
  setFloat("AcquisitionFrameRate", rate);
}

void
BaslerToFWrapper::setTriggerMode(bool enable)
{
  setEnum("TriggerMode", enable ? "On" : "Off");
}

void
BaslerToFWrapper::setTriggerSource(std::string line)
{
  setEnum("TriggerSource", line);
}

int64_t
BaslerToFWrapper::getMaxDepth()
{
  return getInt("DepthMax");
}

void
BaslerToFWrapper::setMaxDepth(int64_t depth)
{
  setInt("DepthMax", depth);
}

int64_t
BaslerToFWrapper::getMinDepth()
{
  return getInt("DepthMin");
}

void
BaslerToFWrapper::setMinDepth(int64_t depth)
{
  setInt("DepthMin", depth);
}

void
BaslerToFWrapper::Start()
{
  running_ = true;
  sampler_ = std::thread(&BaslerToFWrapper::SampleLoop, this);
}

void
BaslerToFWrapper::Stop()
{
  running_ = false;

  if (sampler_.joinable())
    {
      sampler_.join();
    }
}

void
BaslerToFWrapper::SampleLoop()
{
  // Start grabbing with buffer size 15 and 500 ms timeout.
  camera_.GrabContinuous(15, 500, this, &BaslerToFWrapper::HandleResult);
}

bool
BaslerToFWrapper::HandleResult(GrabResult result, BufferParts parts)
{
  if (result.status == GrabResult::Timeout)
    {
      return false; // Indicate to stop acquisition
    }


  if (result.status == GrabResult::Ok)
    {
      const int width = (int) parts[0].width;
      const int height = (int) parts[0].height;
      const uint16_t* depth = (uint16_t*) parts[0].pData;
      const uint16_t* confidence = (uint16_t*) parts[1].pData;

      operation_(depth, confidence, width, height);
    }

  return running_;
}
