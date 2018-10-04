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
#include <exception>


// TODO: Should be configured in CMake
#define GENICAM_GENTL64_PATH "/opt/BaslerToF/lib64/gentlproducer/gtl"

using namespace GenTLConsumerImplHelper;

#define BOOST_LOG_DYN_LINK

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

BaslerToFWrapper::BaslerToFWrapper ( std::string camera_name, Operation operation, Error_signaler error_signaler )
    : operation_ ( operation ), error_signaler_( error_signaler )
{
    setenv ( "GENICAM_GENTL64_PATH", GENICAM_GENTL64_PATH, 1 );

    CToFCamera::InitProducer();
    try
    {
        camera_.Open ( UserDefinedName, camera_name );

        // These are fixed settings for I3DS.
        setSelector ( "Range", true );
        setEnum ( "PixelFormat", "Coord3D_C16" );
        setSelector ( "Intensity", false );
        setSelector ( "Confidence", true );

        //#define HDR // vs Standard
//#define HDR // vs Standard
#ifndef HDR
        setEnum ( "ProcessingMode", "Standard" );
        BOOST_LOG_TRIVIAL ( info ) << "ProcessingMode: Standard";
#else // single
        // TODO: Add option for HDR?
        setEnum ( "ProcessingMode", "Hdr" )
        BOOST_LOG_TRIVIAL ( info ) << "ProcessingMode: Hdr";
#endif

        setEnum ( "ExposureAuto", "Continuous" );
        setFloat ( "Agility", 0.1 );
        setInt ( "Delay", 1 );

    }
    catch ( const GenICam::GenericException &e )
    {
        if ( camera_.IsOpen() )
        {
            camera_.Close();
        }

        if ( CToFCamera::IsProducerInitialized() )
        {
            CToFCamera::TerminateProducer();   // Won't throw any exceptions
        }
        throw e;
    }

}

BaslerToFWrapper::~BaslerToFWrapper()
{
    if ( camera_.IsOpen() )
    {
        camera_.Close();
    }

    if ( CToFCamera::IsProducerInitialized() )
    {
        CToFCamera::TerminateProducer();   // Won't throw any exceptions
    }
}

std::string
BaslerToFWrapper::getEnum ( const char *name )
{
    GenApi::CEnumerationPtr ptr ( camera_.GetParameter ( name ) );
    return std::string ( ptr->ToString() );
}

void
BaslerToFWrapper::setEnum ( const char *name, std::string value )
{
    GenApi::CEnumerationPtr ptr ( camera_.GetParameter ( name ) );
    ptr->FromString ( value.c_str() );
}

int64_t
BaslerToFWrapper::getInt ( const char *name )
{
    GenApi::CIntegerPtr ptr ( camera_.GetParameter ( name ) );
    return ptr->GetValue();
}

int64_t
BaslerToFWrapper::minInt ( const char *name )
{
    GenApi::CIntegerPtr ptr ( camera_.GetParameter ( name ) );
    return ptr->GetMin();
}

int64_t
BaslerToFWrapper::maxInt ( const char *name )
{
    GenApi::CIntegerPtr ptr ( camera_.GetParameter ( name ) );
    return ptr->GetMax();
}

void
BaslerToFWrapper::setInt ( const char *name, int64_t value )
{
    GenApi::CIntegerPtr ptr ( camera_.GetParameter ( name ) );
    ptr->SetValue ( value );
}

float
BaslerToFWrapper::getFloat ( const char *name )
{
    GenApi::CFloatPtr ptr ( camera_.GetParameter ( name ) );
    return ptr->GetValue();
}

float
BaslerToFWrapper::minFloat ( const char *name )
{
    GenApi::CFloatPtr ptr ( camera_.GetParameter ( name ) );
    return ptr->GetMin();
}

float
BaslerToFWrapper::maxFloat ( const char *name )
{
    GenApi::CFloatPtr ptr ( camera_.GetParameter ( name ) );
    return ptr->GetMax();
}

void
BaslerToFWrapper::setFloat ( const char *name, float value )
{
    GenApi::CFloatPtr ptr ( camera_.GetParameter ( name ) );
    ptr->SetValue ( value );
}

void
BaslerToFWrapper::setSelector ( std::string component, bool enable )
{
    GenApi::CEnumerationPtr ptrSelector = camera_.GetParameter ( "ComponentSelector" );
    GenApi::CBooleanPtr ptrEnable = camera_.GetParameter ( "ComponentEnable" );

    ptrSelector->FromString ( component.c_str() );
    ptrEnable->SetValue ( enable );
}

int64_t
BaslerToFWrapper::Width()
{
    return getInt ( "Width" );
}

int64_t
BaslerToFWrapper::Height()
{
    return getInt ( "Height" );
}

int64_t
BaslerToFWrapper::OffsetX()
{
    return getInt ( "OffsetX" );
}

int64_t
BaslerToFWrapper::OffsetY()
{
    return getInt ( "OffsetY" );
}

void
BaslerToFWrapper::setWidth ( int64_t value )
{
    setInt ( "Width", value );
}

void
BaslerToFWrapper::setHeight ( int64_t value )
{
    setInt ( "Height", value );
}

void
BaslerToFWrapper::setOffsetX ( int64_t value )
{
    setInt ( "OffsetX", value );
}

void
BaslerToFWrapper::setOffsetY ( int64_t value )
{
    setInt ( "OffsetY", value );
}

int64_t
BaslerToFWrapper::SensorWidth()
{
    return getInt ( "WidthMax" );
}

int64_t
BaslerToFWrapper::SensorHeight()
{
    return getInt ( "HeightMax" );
}

float
BaslerToFWrapper::getTriggerRate()
{
    return getFloat ( "AcquisitionFrameRate" );
}

float
BaslerToFWrapper::minTriggerRate()
{
    return minFloat ( "AcquisitionFrameRate" );
}

float
BaslerToFWrapper::maxTriggerRate()
{
    return maxFloat ( "AcquisitionFrameRate" );
}

void
BaslerToFWrapper::setTriggerRate ( float rate )
{
    setFloat ( "AcquisitionFrameRate", rate );
}

void
BaslerToFWrapper::setTriggerMode ( bool enable )
{
    setEnum ( "TriggerMode", enable ? "On" : "Off" );
}

void
BaslerToFWrapper::setTriggerSource ( std::string line )
{
    setEnum ( "TriggerSource", line );
}

int64_t
BaslerToFWrapper::getMaxDepth()
{
    return getInt ( "DepthMax" );
}

void
BaslerToFWrapper::setMaxDepth ( int64_t depth )
{
    setInt ( "DepthMax", depth );
}

int64_t
BaslerToFWrapper::getMinDepth()
{
    return getInt ( "DepthMin" );
}


int64_t
BaslerToFWrapper::getMinDepth_lower_limit()
{
    return  minInt ( "DepthMin" );
}

int64_t
BaslerToFWrapper::getMaxDepth_upper_limit()
{
    return  maxInt ( "DepthMax" );
}




void
BaslerToFWrapper::setMinDepth ( int64_t depth )
{
    setInt ( "DepthMin", depth );
}

/// Temperature in celcius
float
BaslerToFWrapper::getTemperature ()
{
    return getFloat ( "DeviceTemperature" );
}


std::string
BaslerToFWrapper::GetDeviceModelName()
{
    GenApi::CStringPtr ptr1 ( camera_.GetParameter ( "DeviceModelName" ) );
    gcstring interfaceDisplayName = ptr1->GetValue();
    BOOST_LOG_TRIVIAL ( info ) << interfaceDisplayName;
    return interfaceDisplayName.c_str();
}


void
BaslerToFWrapper::Start()
{
    running_ = true;
    timeout_counter_ = 0;
    error_counter_ = 0;
    error_flagged_ = false;

    sampler_ = std::thread ( &BaslerToFWrapper::SampleLoop, this );
}

void
BaslerToFWrapper::Stop()
{
  running_ = false;

  if ( sampler_.joinable() )
    {
      sampler_.join();
    }
}


void
BaslerToFWrapper::SampleLoop()
{
  try {
    // Start grabbing with buffer size 15 and 500 ms timeout.
    error_flagged_ = false;
    camera_.GrabContinuous ( 15, 500, this, &BaslerToFWrapper::HandleResult );
    if( error_flagged_ )
      {
	error_signaler_ (flagged_error_message_, true);
	error_flagged_ = false;
      }
  }catch ( const GenICam::GenericException &e )
  {
      BOOST_LOG_TRIVIAL ( error ) <<  "Exception error message: " << e.what();
  }


}

void
BaslerToFWrapper::set_error_status(const std::string error_message)
{
  BOOST_LOG_TRIVIAL ( error ) << "set_error_status() in wrapper";
  BOOST_LOG_TRIVIAL ( error ) <<  "Error message: " << error_message;
  error_flagged_ = true;
  flagged_error_message_ = error_message;
}

bool
BaslerToFWrapper::HandleResult ( GrabResult result, BufferParts parts )
{
    BOOST_LOG_TRIVIAL ( info ) << "HandleResult()";

    if ( !camera_.IsConnected() )
      {
	BOOST_LOG_TRIVIAL ( error ) << "Camera reporting: Not connected. Going to error state.";
	set_error_status("Camera reporting: Not connected. Going to error state.");
	running_ = false;
	return running_;
      }

    if ( result.status == GrabResult::Timeout )
      {
	BOOST_LOG_TRIVIAL ( info ) << "Timeout waiting for image";
	timeout_counter_ ++;
	if (timeout_counter_ > 10)
	  {
	    BOOST_LOG_TRIVIAL ( error ) << "More than 10 error in sampling loop. Going to error state";
	    set_error_status("Camera reporting: Not connected. Going to error state.");
	    running_ = false;
	  }
	return running_; // Just continue and wait for another image.
      }

    if ( result.status == GrabResult::Ok )
    {
        const int width = ( int ) parts[0].width;
        const int height = ( int ) parts[0].height;
        const uint16_t *depth = ( uint16_t * ) parts[0].pData;
        const uint16_t *confidence = ( uint16_t * ) parts[1].pData;

        if ( parts[0].partType != Range || parts[1].partType != Confidence )
        {
            BOOST_LOG_TRIVIAL ( info ) << "Invalid configuration of measurement";
            throw std::logic_error ( "Invalid configuration of measurement" );
        }


        operation_ ( depth, confidence, width, height );
    }
    return running_;
}
