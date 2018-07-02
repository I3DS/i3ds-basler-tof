///////////////////////////////////////////////////////////////////////////\file
///
///   Copyright 2018 SINTEF AS
///
///   This Source Code Form is subject to the terms of the Mozilla
///   Public License, v. 2.0. If a copy of the MPL was not distributed
///   with this file, You can obtain one at https://mozilla.org/MPL/2.0/
///
////////////////////////////////////////////////////////////////////////////////


#ifndef __BASLER_TOF_CAMERA_HPP
#define __BASLER_TOF_CAMERA_HPP

#include "i3ds/topic.hpp"
#include "i3ds/publisher.hpp"
#include <i3ds/tof_camera_sensor.hpp>

#include <memory>

#include "basler_tof_interface.hpp"

namespace i3ds
{

class BaslerToFCamera : public ToFCamera
{
public:

  struct Parameters
  {
    std::string camera_name;
    bool free_running;
  };

  BaslerToFCamera(Context::Ptr context, NodeID id, Parameters param);
  virtual ~BaslerToFCamera();

  // Getters.
  virtual bool region_enabled() const;
  virtual PlanarRegion region() const;

  virtual double range_min_depth() const;
  virtual double range_max_depth() const;

  virtual bool is_sampling_supported(SampleCommand sample);

protected:

  // Actions.
  virtual void do_activate();
  virtual void do_start();
  virtual void do_stop();
  virtual void do_deactivate();

  // Handlers.
  virtual void handle_region(RegionService::Data& command);
  virtual void handle_range(RangeService::Data& command);

private:

  const Parameters param_;

  bool send_sample(unsigned char* image, unsigned long timestamp_us);

  Publisher publisher_;

  std::unique_ptr<Basler_ToF_Interface> cameraInterface;
};

} // namespace i3ds

#endif