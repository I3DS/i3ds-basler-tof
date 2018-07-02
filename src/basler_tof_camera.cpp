///////////////////////////////////////////////////////////////////////////\file
///
///   Copyright 2018 SINTEF AS
///
///   This Source Code Form is subject to the terms of the Mozilla
///   Public License, v. 2.0. If a copy of the MPL was not distributed
///   with this file, You can obtain one at https://mozilla.org/MPL/2.0/
///
////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <sstream>
#include <iomanip>
#include <memory>

#include <ConsumerImplHelper/ToFCamera.h>

#include "basler_tof_camera.hpp"
#include "basler_tof_interface.hpp"

#define BOOST_LOG_DYN_LINK

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

namespace logging = boost::log;

i3ds::BaslerToFCamera::BaslerToFCamera(Context::Ptr context, NodeID node, Parameters param)
  : ToFCamera(node),
    param_(param),
    publisher_(context, node)
{
  using namespace std::placeholders;

  BOOST_LOG_TRIVIAL(info) << "BaslerToFCamera::BaslerToFCamera()";

  cameraInterface = std::unique_ptr<Basler_ToF_Interface>(new Basler_ToF_Interface("", param.camera_name, param.free_running,
                    std::bind(&i3ds::BaslerToFCamera::send_sample, this, _1, _2)));

}

i3ds::BaslerToFCamera::~BaslerToFCamera()
{
}

bool
i3ds::BaslerToFCamera::region_enabled() const
{
  return false;
}

PlanarRegion
i3ds::BaslerToFCamera::region() const
{
  return {0, 0, 0, 0};
}

double
i3ds::BaslerToFCamera::range_min_depth() const
{
  return 0.0;
}

double
i3ds::BaslerToFCamera::range_max_depth() const
{
  return 1.0e3;
}

void
i3ds::BaslerToFCamera::do_activate()
{
  BOOST_LOG_TRIVIAL(info) << "do_activate()";

  cameraInterface->connect();
}

void
i3ds::BaslerToFCamera::do_start()
{
  BOOST_LOG_TRIVIAL(info) << "do_start()";

  cameraInterface->do_start();
}

void
i3ds::BaslerToFCamera::do_stop()
{
  BOOST_LOG_TRIVIAL(info) << "do_stop()";

  cameraInterface->do_stop();
}

void
i3ds::BaslerToFCamera::do_deactivate()
{
  BOOST_LOG_TRIVIAL(info) << "do_deactivate()";

  cameraInterface->do_deactivate();
}

bool
i3ds::BaslerToFCamera::is_sampling_supported(SampleCommand sample)
{
  BOOST_LOG_TRIVIAL(info) << "is_rate_supported() " << sample.period;
  return cameraInterface->checkTriggerInterval(sample.period);
}

void
i3ds::BaslerToFCamera::handle_region(RegionService::Data& command)
{
  BOOST_LOG_TRIVIAL(info) << "handle_region()";
  if (!(is_active()))
    {

      BOOST_LOG_TRIVIAL(info) << "handle_region()-->Not in active state";

      std::ostringstream errorDescription;
      errorDescription << "handle_region: Not in active state";
      throw i3ds::CommandError(error_value, errorDescription.str());
    }

  cameraInterface->setRegionEnabled(command.request.enable);

  if (command.request.enable)
    {
      cameraInterface->setRegion(command.request.region);
    }
}

void
i3ds::BaslerToFCamera::handle_range(RangeService::Data& command)
{

}

bool
i3ds::BaslerToFCamera::send_sample(unsigned char *image, unsigned long timestamp_us)
{
  BOOST_LOG_TRIVIAL(info) << "BaslerToFCamera::send_sample()x";

  BufferParts *parts = reinterpret_cast<BufferParts *>(image);
  PartInfo partInfo0 = (*parts)[0];

  // TODO Is Copy or assignment
  BufferParts p = *parts;

// BOOST_LOG_TRIVIAL (info) << "Test Copy or assignment pointers: parts, p: " << parts << ":" << ((void *)p);

  int x = partInfo0.width;




  uint16_t *depth = (uint16_t *)p[0].pData;


  int width = (*parts)[0].width;
  int height = (*parts)[0].height;

  int ht =  p[0].height;
  const int numberOfPixels = width * height;
  BOOST_LOG_TRIVIAL(info) << "numberOfPixels: " << numberOfPixels << " width:  "<< width << " height: " << height << " xx: " << x <<
                          " ht"<< ht <<" p[0].width: " << p[0].width;


  // just checks for  configuration of camera.
  /*std::ostringstream errorDescription;
        errorDescription << "Send data:Error!! Wrong  initialization of ToF Camera data type";
        throw i3ds::CommandError(error_value, errorDescription.str());
  */
  if (p[0].partType != Range)
    {
      BOOST_LOG_TRIVIAL(info) << "Error!! Wrong  initialization of ToF Camera data type";
    }
  if (p[1].partType != Confidence)
    {
      BOOST_LOG_TRIVIAL(info) << "Error!! Wrong  initialization of ToF Camera data type";
    }

  uint16_t *pConfidenceArr = (uint16_t *)p[1].pData;
  int64_t minDepth = cameraInterface->getMinDepthLocalInMM();
  int64_t maxDepth = cameraInterface->getMaxDepthLocalInMM();

  BOOST_LOG_TRIVIAL(info) << "minDepth: " << minDepth << " maxDepth: " << maxDepth;

  ToFCamera::MeasurementTopic::Data frame;

  for (int i= 0; i < numberOfPixels; i++)
    {
      //Calculate distance
      float f = (minDepth+(depth[i]*maxDepth)* (1./std::numeric_limits<uint16_t>::max()))*0.001;
      frame.distances.arr[i] = f;

      // Check confidence
      if ((depth[i] == 0) || (pConfidenceArr[i] == 0))
        {
          frame.validity.arr[i] = depth_range_error ; //TODO Correct status?
        }

      else
        {
          frame.validity.arr[i] = depth_valid;
        }


      if (i==(numberOfPixels/2- x/2))
        {
          BOOST_LOG_TRIVIAL(info) << "Mid-pixel  frame_.distances.arr[i][" <<i << "]:" << std::setprecision(5) <<
                                  frame.distances.arr[i] <<
                                  " => " <<
                                  std::setprecision(5) << f << " [meter]" <<
                                  " Confidence: " << ((frame.validity.arr[i]== depth_valid) ? "Ok":"Error");
        }
    }

  frame.attributes.timestamp = timestamp_us;
  frame.attributes.validity = sample_valid;

  publisher_.Send<ToFCamera::MeasurementTopic>(frame);

  return true;
}
