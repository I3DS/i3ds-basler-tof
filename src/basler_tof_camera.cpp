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

#include "basler_tof_camera.hpp"

#define BOOST_LOG_DYN_LINK

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

namespace logging = boost::log;

i3ds::BaslerToFCamera::BaslerToFCamera ( Context::Ptr context, NodeID node, Parameters param )
    : ToFCamera ( node ),
      param_ ( param ),
      publisher_ ( context, node )
{
    using namespace std::placeholders;

    BOOST_LOG_TRIVIAL ( info ) << "BaslerToFCamera::BaslerToFCamera()";

    camera_ = nullptr;
}

i3ds::BaslerToFCamera::~BaslerToFCamera()
{
    if ( camera_ )
    {
        delete camera_;
    }
}


void
i3ds::BaslerToFCamera::set_error_state(const std::string &error_message, const bool dont_throw = false ) const
{
  BOOST_LOG_TRIVIAL ( error ) << "Error message: " << error_message;

  set_failure();
  if ( !dont_throw )
    {
      throw i3ds::CommandError ( error_other, error_message );
    }
}


/// Realtime check of region_enabled. Avoiding updating an variable all the time. Safer for consistencity
bool
i3ds::BaslerToFCamera::region_enabled() const
{
  bool retval = false;
  try {
    if ( ( camera_->Width() == camera_->SensorWidth() ) &&
	 ( camera_->Height() == camera_->SensorHeight() ) &&
	 ( camera_->OffsetX() == 0 ) &&
	 ( camera_->OffsetY() == 0 )
       )
	{
	  retval = false;
	}
    else
	 {
	  retval = true;
	 }
    } catch ( const GenICam::GenericException &e )
       {
	    BOOST_LOG_TRIVIAL ( error ) << "region() problem communicating with hw";
	    set_error_state("Error communicating with ToF in region_enabled(): " + std::string ( e.what() ) );
	}
    return retval;
}

PlanarRegion
i3ds::BaslerToFCamera::region() const
{
  PlanarRegion region;
  BOOST_LOG_TRIVIAL ( info ) << "Check region";
  try {
	region.offset_x = ( T_UInt16 ) camera_->OffsetX();
	region.offset_y = ( T_UInt16 ) camera_->OffsetY();
	region.size_x = ( T_UInt16 ) camera_->Width();
	region.size_y = ( T_UInt16 ) camera_->Height();
    } catch ( const GenICam::GenericException &e )
	 {
	      BOOST_LOG_TRIVIAL ( error ) << "region() problem communicating with hw";
	      set_error_state( "Error communicating with ToF in region(): " + std::string ( e.what() ));
	  }
    return region;
}


double
i3ds::BaslerToFCamera::range_min_depth() const
{
    return 1.0e-3 * ( double ) camera_->getMinDepth();
}

double
i3ds::BaslerToFCamera::range_max_depth() const
{
    return 1.0e-3 * ( double ) camera_->getMaxDepth();
}



double
i3ds::BaslerToFCamera::range_min_depth_lower_limit() const
{
    return 1.0e-3 * ( double ) camera_->getMinDepth_lower_limit();
}

double
i3ds::BaslerToFCamera::range_max_depth_upper_limit() const
{
    return 1.0e-3 * ( double ) camera_->getMaxDepth_upper_limit();
}


/// Sensorboard Temperature converted to Kelvin
double
i3ds::BaslerToFCamera::temperature () const
{
  double retval = 0.0;
  try
    {
      retval = camera_->getTemperature () + 273.15;
    } catch ( const GenICam::GenericException &e )
      {
	  BOOST_LOG_TRIVIAL ( error ) << "temperature() problem communicating with hw";
	  set_error_state("Error communicating with ToF in temperature(): " + std::string ( e.what() ));
       }
    return retval;
}


void
i3ds::BaslerToFCamera::do_activate()
{
    using namespace std::placeholders;

    BOOST_LOG_TRIVIAL ( info ) << "do_activate()";

    try
    {
        auto operation = std::bind ( &i3ds::BaslerToFCamera::send_sample, this, _1, _2, _3, _4 );

        camera_ = new BaslerToFWrapper ( param_.camera_name, operation );
        BOOST_LOG_TRIVIAL ( info ) << "region_enabled() " << region_enabled();
        set_device_name ( camera_->GetDeviceModelName() );

    }
    catch ( const GenICam::GenericException &e )
    {
        if ( camera_ )
        {
            BOOST_LOG_TRIVIAL ( info ) << "Camera deinit";
            delete camera_;
            camera_ = nullptr;
        }

        throw i3ds::CommandError ( error_other, "Error activating ToF: " + std::string ( e.what() ) );
    }
}

void
i3ds::BaslerToFCamera::do_start()
{
    BOOST_LOG_TRIVIAL ( info ) << "do_start()";
    try {

      min_depth_ = range_min_depth();
      max_depth_ = range_max_depth();

      if ( param_.free_running )
      {
	  camera_->setTriggerMode ( false );
	  camera_->setTriggerRate ( 1.0e6 / period() );
      }
      else
      {
	  camera_->setTriggerMode ( true );
	  camera_->setTriggerSource ( "Line1" );
      }

      camera_->Start();
    } catch ( const GenICam::GenericException &e )
     {
	BOOST_LOG_TRIVIAL ( error ) << "do_start() problem communicating with hw";
	set_error_state("Error communicating with ToF in do_start(): " + std::string ( e.what() ), false);
      }
}

void
i3ds::BaslerToFCamera::do_stop()
{
    BOOST_LOG_TRIVIAL ( info ) << "do_stop()";

    camera_->Stop();
}

void
i3ds::BaslerToFCamera::do_deactivate()
{
    BOOST_LOG_TRIVIAL ( info ) << "do_deactivate()";

    delete camera_;
    camera_ = nullptr;
}

bool
i3ds::BaslerToFCamera::is_sampling_supported ( SampleCommand sample )
{
  bool retval = false;
  try {
    if ( !param_.free_running )
    {
        throw i3ds::CommandError ( error_other, "Period is not relevant in free-running mode" );
    }

    const float rate = 1.0e6 / sample.period;
    const float max_rate = camera_->maxTriggerRate();
    const float min_rate = camera_->minTriggerRate();
    BOOST_LOG_TRIVIAL ( trace ) << "rate() " << rate;
    BOOST_LOG_TRIVIAL ( trace ) << "max_rate " << max_rate;
    BOOST_LOG_TRIVIAL ( trace ) << "min_rate " << min_rate;
    BOOST_LOG_TRIVIAL ( trace ) << "(min_rate <= rate && rate <= max_rate) " << ((min_rate <= rate) && (rate <= max_rate));


    retval = min_rate <= rate && rate <= max_rate;
  } catch ( const GenICam::GenericException &e )
      {
	BOOST_LOG_TRIVIAL ( error ) << "is_sampling_supported() problem communicating with hw";
	set_error_state("Error communicating with ToF in is_sampling_supported(): " + std::string ( e.what() ));
       }
    return retval;
}

void
i3ds::BaslerToFCamera::handle_region ( RegionService::Data &command )
{
    BOOST_LOG_TRIVIAL ( info ) << "handle_region()";
    try
      {
	check_standby();

	if ( command.request.enable )
	  {
	      PlanarRegion region = command.request.region;

	      // Parameter checks
	      if ( ( region.size_x % 2 ) || ( region.size_y % 2 ) ||
		      ( region.offset_x % 2 ) || ( region.offset_y % 2 )
		 )
	      {
		  throw i3ds::CommandError ( error_value, "Region sizes and offsets have to be a even number." );
	      }



	      if ( ( region.size_x == 0 ) || ( region.size_y == 0 ) )
	      {
		  throw i3ds::CommandError ( error_value, "Region size (width or height) can not be zero" );
	      }

	      if ( ( region.size_x + region.offset_x ) > ( ( unsigned ) camera_->SensorWidth() ) )
	      {
		  throw i3ds::CommandError ( error_value, "Impossible condition: (Region width + offset_x) > (Maximum width of sensor) => " +
					     std::to_string ( ( region.size_x + region.offset_x ) ) +
					     " > " +
					     std::to_string ( camera_->SensorWidth() )
					   );
	      }

	      if ( ( region.size_y + region.offset_y ) > ( ( unsigned ) camera_->SensorHeight() ) )
	      {
		  throw i3ds::CommandError ( error_value, "Imposible condition (Region height + offset_y) > (Maximum height of sensor) => " +
					     std::to_string ( ( region.size_y + region.offset_y ) ) +
					     " > " +
					     std::to_string ( camera_->SensorHeight() )
					   );
	      }

	      // Decrease first, increase afterwards
	      if ( region.size_x > ( ( unsigned ) camera_->Width() ) )
	      {
		  camera_->setOffsetX ( region.offset_x );
		  camera_->setWidth ( region.size_x );
	      }
	      else
	      {
		  camera_->setWidth ( region.size_x );
		  camera_->setOffsetX ( region.offset_x );
	      }

	      if ( region.size_y > ( ( unsigned ) camera_->Height() ) )
	      {
		  camera_->setOffsetY ( region.offset_y );
		  camera_->setHeight ( region.size_y );
	      }
	      else
	      {
		  camera_->setHeight ( region.size_y );
		  camera_->setOffsetX ( region.offset_y );
	      }
	  }
	  else
	  {
	      camera_->setOffsetX ( 0 );
	      camera_->setOffsetY ( 0 );
	      camera_->setWidth ( camera_->SensorWidth() );
	      camera_->setHeight ( camera_->SensorHeight() );
	  }
      }
    catch ( const GenICam::GenericException &e )
      {
	BOOST_LOG_TRIVIAL ( error ) << "handle_region() problem communicating with hw";

	set_error_state("Error communicating with ToF in handle_region(): " + std::string ( e.what() ) );
       }

}

void
i3ds::BaslerToFCamera::handle_range ( RangeService::Data &command )
{
    BOOST_LOG_TRIVIAL ( info ) << "handle_range()";
    try {
      check_standby();

      // Check input parameters
      if ( command.request.min_depth < range_min_depth_lower_limit() )
	{
	    throw i3ds::CommandError ( error_value, "Minimum depth must be greater than " + std::to_string ( range_min_depth_lower_limit() ) + "[m]" );
	}

      if ( command.request.max_depth > range_max_depth_upper_limit() )
	{
	    throw i3ds::CommandError ( error_value, "Maximum depth must be less than " + std::to_string ( range_max_depth_upper_limit() ) + "[m]" );
	}

      if ( command.request.min_depth > command.request.max_depth )
	{
	    throw i3ds::CommandError ( error_value, "Maximum depth must be larger than minimum depth" );
	}

      camera_->setMinDepth ( ( int64_t ) ( command.request.min_depth * 1000 ) );
      camera_->setMaxDepth ( ( int64_t ) ( command.request.max_depth * 1000 ) );
    } catch ( const GenICam::GenericException &e )
      {
	BOOST_LOG_TRIVIAL ( error ) << "handle_range() problem communicating with hw";
	set_error_state("Error communicating with ToF in handle_range(): " + std::string ( e.what() ) );
      }
}

bool
i3ds::BaslerToFCamera::send_sample ( const uint16_t *depth, const uint16_t *confidence, int width, int height )
{
    BOOST_LOG_TRIVIAL ( info ) << "BaslerToFCamera::send_sample()";
    BOOST_LOG_TRIVIAL ( info ) << "ProcessingMode " << camera_->getEnum ( "ProcessingMode");

    const int size = width * height;

    ToFCamera::MeasurementTopic::Data frame;
    ToFCamera::MeasurementTopic::Codec::Initialize ( frame );

    // TODO: Also need offset
    frame.region.size_x = ( T_UInt16 ) width;
    frame.region.size_y = ( T_UInt16 ) height;

    frame.distances.nCount = size;
    frame.validity.nCount = size;

    // Depth of 2**16 - 1 is max_depth, 0 is min_depth
    const double KA = ( max_depth_ - min_depth_ ) / 65535.0;
    const double KB = min_depth_;

    for ( int i = 0; i < size; i++ )
    {
        frame.distances.arr[i] = KA * depth[i] + KB;

        // TODO: Check if we can add threshold here?
        if ( depth[i] == 0 || confidence[i] == 0 )
        {
            frame.validity.arr[i] = depth_range_error;
        }
    }

    // TODO: Set timestamp
    frame.attributes.timestamp = 0;
    frame.attributes.validity = sample_valid;

    publisher_.Send<ToFCamera::MeasurementTopic> ( frame );
    return true;
}
