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

    region_enabled_ = false;
    camera_ = nullptr;
}

i3ds::BaslerToFCamera::~BaslerToFCamera()
{
    if ( camera_ )
    {
        delete camera_;
    }
}

bool
i3ds::BaslerToFCamera::region_enabled() const
{
    return region_enabled_;
}

PlanarRegion
i3ds::BaslerToFCamera::region() const
{
    PlanarRegion region;
    BOOST_LOG_TRIVIAL ( info ) << "Check region";
    region.offset_x = ( T_UInt16 ) camera_->OffsetX();
    region.offset_y = ( T_UInt16 ) camera_->OffsetY();
    region.size_x = ( T_UInt16 ) camera_->Width();
    region.size_y = ( T_UInt16 ) camera_->Height();

    BOOST_LOG_TRIVIAL ( info ) << "Check region"  <<  region.offset_x ;

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


void
i3ds::BaslerToFCamera::do_activate()
{
    using namespace std::placeholders;

    BOOST_LOG_TRIVIAL ( info ) << "do_activate()";

    try
    {
        auto operation = std::bind ( &i3ds::BaslerToFCamera::send_sample, this, _1, _2, _3, _4 );

        camera_ = new BaslerToFWrapper ( param_.camera_name, operation );
        if (
            ( camera_->Width() == camera_->SensorWidth() ) &&
            ( camera_->Height() == camera_->SensorHeight() ) &&
            ( camera_->OffsetX() == 0 ) &&
            ( camera_->OffsetY() == 0 )
            /*
            (camera_->Width.GetValue() == camera_->SensorWidth.GetValue()) &&
            (camera_->Height.GetValue() == camera_->SensorHeight.GetValue()) &&
            (camera_->OffsetX.GetValue() == 0 ) &&
            (camera_->OffsetY.GetValue() == 0 )
            */

        )
        {
            region_enabled_ = false;
        }
        else
        {
            region_enabled_ = true;
        }
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

    if ( !param_.free_running )
    {
        throw i3ds::CommandError ( error_other, "Period is not relevant in free-running mode" );
    }

    const float rate = 1.0e6 / sample.period;
    const float max_rate = camera_->maxTriggerRate();
    const float min_rate = camera_->minTriggerRate();

    return min_rate <= rate && rate <= max_rate;
}

void
i3ds::BaslerToFCamera::handle_region ( RegionService::Data &command )
{
    BOOST_LOG_TRIVIAL ( info ) << "handle_region()";

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
            throw i3ds::CommandError ( error_value, "Set_region width + offset_x > maximal width : " +
                                       std::to_string ( ( region.size_x + region.offset_x ) ) +
                                       ">" +
                                       std::to_string ( camera_->SensorWidth() )
                                     );
        }

        if ( ( region.size_y + region.offset_y ) > ( ( unsigned ) camera_->SensorHeight() ) )
        {
            throw i3ds::CommandError ( error_value, "Set_region height + offset_y > maximal height : " +
                                       std::to_string ( ( region.size_y + region.offset_y ) ) +
                                       ">" +
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

    region_enabled_ = command.request.enable;
}

void
i3ds::BaslerToFCamera::handle_range ( RangeService::Data &command )
{
    BOOST_LOG_TRIVIAL ( info ) << "handle_range()";

    check_standby();

    // Check input parameters
    if ( command.request.min_depth < range_min_depth_lower_limit() )
    {
        throw i3ds::CommandError ( error_other, "Minimum depth must be greater than " + std::to_string ( range_min_depth_lower_limit() ) + "[m]" );
    }

    if ( command.request.max_depth > range_max_depth_upper_limit() )
    {
        throw i3ds::CommandError ( error_other, "Maximum depth must be less than " + std::to_string ( range_max_depth_upper_limit() ) + "[m]" );
    }
    if ( command.request.min_depth > command.request.max_depth )
    {
        throw i3ds::CommandError ( error_other, "Maximum depth must be larger than minimum depth" );
    }

    camera_->setMinDepth ( ( int64_t ) ( command.request.min_depth * 1000 ) );
    camera_->setMaxDepth ( ( int64_t ) ( command.request.max_depth * 1000 ) );
}

bool
i3ds::BaslerToFCamera::send_sample ( const uint16_t *depth, const uint16_t *confidence, int width, int height )
{
    BOOST_LOG_TRIVIAL ( info ) << "BaslerToFCamera::send_sample()";

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
