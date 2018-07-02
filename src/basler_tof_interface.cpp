#include <ConsumerImplHelper/ToFCamera.h>
#include <iostream>
#include <iomanip>
#include <exception>
#include <thread>
#include <stdlib.h>

#include "basler_tof_interface.hpp"

// REMARK: Must be set different to point to different library than the other Basler cameras
// or it will not run.
// Should it be configured in cmake?
#define GENICAM_GENTL64_PATH "/opt/BaslerToF/lib64/gentlproducer/gtl"

using namespace GenTLConsumerImplHelper;
using namespace std;

#define BOOST_LOG_DYN_LINK

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

// TODO Move identifying parameter and which to use?

// TODO Check Throwing in sampling loop. May be the error handling should return and set a global flag as in example.?

Basler_ToF_Interface::Basler_ToF_Interface(std::string const & connectionString,
    std::string const & camera_name,
    bool free_running,
    Operation operation) :
  mConnectionID(connectionString), camera_name(camera_name), free_running_(
    free_running), operation_(operation)
{
  BOOST_LOG_TRIVIAL(info) << "Basler_ToF_Interface constructor";
  BOOST_LOG_TRIVIAL(info) << "Connection String: " << mConnectionID;
  Basler_ToF_Interface2();

}

void
Basler_ToF_Interface::Basler_ToF_Interface2()
{
  // Must be set different than the other basler cameras
  // GENICAM_GENTL64_PATH=/opt/BaslerToF/lib64/gentlproducer/gtl
  BOOST_LOG_TRIVIAL(info) << "Setting environment variable: GENICAM_GENTL64_PATH to :"
                          << GENICAM_GENTL64_PATH << std::endl;
  setenv("GENICAM_GENTL64_PATH", GENICAM_GENTL64_PATH, 1);
  BOOST_LOG_TRIVIAL(info)  << "Initializing camera environment for ToF camera"
                           << GENICAM_GENTL64_PATH;
  CToFCamera::InitProducer();
}

Basler_ToF_Interface::~Basler_ToF_Interface()
{
  // Release the GenTL producer and all of its resources.
  // Note: Don't call TerminateProducer() until the destructor of the CToFCamera
  // class has been called. The destructor may require resources which may not
  // be available anymore after TerminateProducer() has been called.
  BOOST_LOG_TRIVIAL(info) << "Terminating camera environment for ToF camera" << endl;
  if (CToFCamera::IsProducerInitialized())
    {
      CToFCamera::TerminateProducer();   // Won't throw any exceptions
    }
}

void
Basler_ToF_Interface::setRegion(const PlanarRegion region)
{
  BOOST_LOG_TRIVIAL(info) << "setRegion";

  GenApi::CIntegerPtr ptrWidth(m_Camera.GetParameter("Width"));
  GenApi::CIntegerPtr ptrHeight(m_Camera.GetParameter("Height"));
  GenApi::CIntegerPtr ptrOffsetX(m_Camera.GetParameter("OffsetX"));
  GenApi::CIntegerPtr ptrOffsetY(m_Camera.GetParameter("OffsetY"));

  ptrWidth->SetValue(region.size_x);
  ptrHeight->SetValue(region.size_y);
  ptrOffsetX->SetValue(region.offset_x);
  ptrOffsetY->SetValue(region.offset_y);
}

PlanarRegion
Basler_ToF_Interface::getRegion()
{
  BOOST_LOG_TRIVIAL(info) << "getRegion";

  GenApi::CIntegerPtr ptrWidth(m_Camera.GetParameter("Width"));
  GenApi::CIntegerPtr ptrHeight(m_Camera.GetParameter("Height"));
  GenApi::CIntegerPtr ptrOffsetX(m_Camera.GetParameter("OffsetX"));
  GenApi::CIntegerPtr ptrOffsetY(m_Camera.GetParameter("OffsetY"));
  int64_t size_x = ptrWidth->GetValue();
  int64_t size_y = ptrHeight->GetValue();
  int64_t offset_x = ptrOffsetX->GetValue();
  int64_t offset_y = ptrOffsetY->GetValue();

  PlanarRegion region;

  region.offset_x = offset_x;
  region.offset_y = offset_y;
  region.size_x = size_x;
  region.size_y = size_y;

  return region;
}

void
Basler_ToF_Interface::setTriggerInterval()
{
  BOOST_LOG_TRIVIAL(info) << "Setting sampling rate " << samplingsRate_in_Hz_;

  GenApi::CFloatPtr ptrTriggerInterval(
    m_Camera.GetParameter("AcquisitionFrameRate"));
  ptrTriggerInterval->SetValue(samplingsRate_in_Hz_);
}

void
Basler_ToF_Interface::setTriggerInterval_in_Hz(float rate_in_Hz)
{
  BOOST_LOG_TRIVIAL(info) << "setting sampling rate " << rate_in_Hz;

  GenApi::CFloatPtr ptrTriggerInterval(
    m_Camera.GetParameter("AcquisitionFrameRate"));
  ptrTriggerInterval->SetValue(rate_in_Hz);
}


// TODO where is the Conversion float uS?

float
Basler_ToF_Interface::getSamplingsRate()
{
  GenApi::CFloatPtr ptrTriggerInterval(
    m_Camera.GetParameter("AcquisitionFrameRate"));
  return ptrTriggerInterval->GetValue();
}

bool
Basler_ToF_Interface::checkTriggerInterval(int64_t period)  // period is in us, camera operates in Hz
{

  float wished_rate_in_Hz = 1e6 / period;
  BOOST_LOG_TRIVIAL(info) << "Tof CheckTriggerInterval: " << wished_rate_in_Hz
                          << " Hz = " << period << " uS";
  GenApi::CFloatPtr ptrTriggerInterval(
    m_Camera.GetParameter("AcquisitionFrameRate"));
  float max_rate_Hz = ptrTriggerInterval->GetMax();
  float min_rate_Hz = ptrTriggerInterval->GetMin();
  BOOST_LOG_TRIVIAL(info) << "Allowed rate in Hz: Min: " << min_rate_Hz
                          << "Hz, Max: " << max_rate_Hz << "Hz";
  if ((wished_rate_in_Hz >= min_rate_Hz) && wished_rate_in_Hz <= max_rate_Hz)
    {
      BOOST_LOG_TRIVIAL(info)
          << "Tof CheckTriggerInterval. Frequency ok. Storing it.";
      samplingsRate_in_Hz_ = wished_rate_in_Hz;
      return true;
    }
  else
    {
      BOOST_LOG_TRIVIAL(info)
          << "Tof CheckTriggerInterval. Frequency NOT ok. Using the old one.";
      return false;
    }
}

/// REMARK HDR mode
bool
Basler_ToF_Interface::getAutoExposureEnabled()
{
  GenApi::CEnumerationPtr ptrAutoExposureEnabled(
    m_Camera.GetParameter("ExposureAuto"));
  const char *returnedString = ptrAutoExposureEnabled->ToString();
  if (strncmp(returnedString, "Continuous", 4) == 0)
    {
      return true;
    }
  else
    {
      return false;
    }
}

void
Basler_ToF_Interface::setAutoExposureEnabled(bool value)
{
  GenApi::CEnumerationPtr ptrAutoExposureEnabled(
    m_Camera.GetParameter("ExposureAuto"));
  if (value == true)
    {
      ptrAutoExposureEnabled->FromString("Continuous");
    }
  else
    {
      ptrAutoExposureEnabled->FromString("Off");
    }
}

void
Basler_ToF_Interface::setTriggerModeOn(bool value)
{
  GenApi::CEnumerationPtr ptrAutoExposureEnabled(
    m_Camera.GetParameter("TriggerMode"));
  if (value == true)
    {
      ptrAutoExposureEnabled->FromString("On");
    }
  else
    {
      ptrAutoExposureEnabled->FromString("Off");
    }
}

/// \TODO Not found?
int64_t
Basler_ToF_Interface::getGain()
{
  return -1;
}

/// \TODO Not found?
void
Basler_ToF_Interface::setGain(int64_t value)
{

}

// Not used in ToF camera
void
Basler_ToF_Interface::setRegionEnabled(bool regionEnabled)
{
  return;
}

// Not Used in tof Camera
bool
Basler_ToF_Interface::getRegionEnabled()
{
  return true;
}

int64_t
Basler_ToF_Interface::getMaxDepth()
{
  GenApi::CIntegerPtr ptrMaxDepth(m_Camera.GetParameter("DepthMax"));
  return ptrMaxDepth->GetValue();

}


void
Basler_ToF_Interface::setMaxDepth(int64_t depth)
{
  GenApi::CIntegerPtr ptrMaxDepth(m_Camera.GetParameter("DepthMax"));
  return ptrMaxDepth->SetValue(depth);

}

int64_t
Basler_ToF_Interface::getMinDepth()
{
  GenApi::CIntegerPtr ptrMinDepth(m_Camera.GetParameter("DepthMin"));
  return ptrMinDepth->GetValue();

}

void
Basler_ToF_Interface::setMinDepth(int64_t depth)
{
  GenApi::CIntegerPtr ptrMinDepth(m_Camera.GetParameter("DepthMin"));
  return ptrMinDepth->SetValue(depth);

}


/// \TODO Is this exposure time here?
// Actually a float in us
int64_t
Basler_ToF_Interface::getMaxShutterTime()
{
  GenApi::CFloatPtr ptrMaxShutterTime(m_Camera.GetParameter("ExposureTime"));
  return ptrMaxShutterTime->GetValue();

}

/// TODO Is this exposure time here?
// Actually a float in us
void
Basler_ToF_Interface::setMaxShutterTime(int64_t value)
{
  GenApi::CFloatPtr ptrMaxShutterTime(m_Camera.GetParameter("ExposureTime"));
  ptrMaxShutterTime->SetValue(value);
}

bool
Basler_ToF_Interface::connect()
{
  do_activate();
  return true;
}

// TODO Which parameter to use to find?
void
Basler_ToF_Interface::do_activate()
{
  BOOST_LOG_TRIVIAL(info) << "do_activate ()";
  try
    {
      // Open the first camera found, i.e., establish a connection to the camera device.
      // m_Camera.OpenFirstCamera ();

      //  CToFCamera::OpenFirstCamera() is a shortcut for the following sequence:
      //CameraList cameras = CToFCamera::EnumerateCameras ();
      //CameraInfo camInfo = *cameras.begin ();
      // m_Camera.Open (camInfo);

      //  If there are multiple cameras connected and you want to open a specific one, use
      //  the CToFCamera::Open( CameraInfoKey, string ) method.

      // Example: Open a camera using its IP address
      // CToFCamera::Open( IpAddress, "10.0.1.110" );
      //m_Camera.Open (IpAddress, "10.0.1.110");
      try
        {
          m_Camera.Open(UserDefinedName, camera_name);
        }
      catch (const GenICam::GenericException& e)
        {
          BOOST_LOG_TRIVIAL(info) << "TOF::do_activate";
          std::ostringstream errorDescription;
          errorDescription << "TOF::do_activate: " << e.GetDescription();
          BOOST_LOG_TRIVIAL(info) << "TOF::do_activate:Error:"<< e.GetDescription();
          throw i3ds::CommandError(error_value, errorDescription.str());
        };
      /*
       Instead of the IP address, any other property of the CameraInfo struct can be used,
       e.g., the serial number or the user-defined name:

       CToFCamera::O  return -1pen( SerialNumber, "23167572" );
       CToFCamera::Open( UserDefinedName, "Left" );
       */

      // BOOST_LOG_TRIVIAL (info) <<  << "Connected to camera " << m_Camera.GetCameraInfo ().strDisplayName;

      // Enable 3D (point cloud) data, intensity data, and confidence data.
      GenApi::CEnumerationPtr ptrComponentSelector = m_Camera.GetParameter(
            "ComponentSelector");
      GenApi::CBooleanPtr ptrComponentEnable = m_Camera.GetParameter(
            "ComponentEnable");
      GenApi::CEnumerationPtr ptrPixelFormat = m_Camera.GetParameter(
            "PixelFormat");

      // Enable range data.
      ptrComponentSelector->FromString("Range");
      ptrComponentEnable->SetValue(true);
      // Range information can be sent either as a 16-bit grey value image or as 3D coordinates (point cloud). For this sample, we want to acquire 3D coordinates.
      // Note: To change the format of an image component, the Component Selector must first be set to the component
      // you want to configure (see above).
      // To use 16-bit integer depth information, choose "Mono16" instead of "Coord3D_ABC32f".
      //ptrPixelFormat->FromString ("Coord3D_ABC32f");
      ptrPixelFormat->FromString("Coord3D_C16");

      ptrComponentSelector->FromString("Intensity");
      //ptrComponentEnable->SetValue (false);
      ptrComponentEnable->SetValue(false);

      ptrComponentSelector->FromString("Confidence");
      ptrComponentEnable->SetValue(true);

      samplingsRate_in_Hz_ = getSamplingsRate();
      minDepth_ = getMinDepth();
      maxDepth_ = getMaxDepth();

    }
  catch (const GenICam::GenericException& e)
    {
      cerr << "Exception occurred: " << e.GetDescription() << endl;
      // After successfully opening the camera, the IsConnected method can be used
      // to check if the device is still connected.
      if (m_Camera.IsOpen() && !m_Camera.IsConnected())
        {
          cerr << "Camera has been removed." << endl;
          throw std::runtime_error("Camera has been removed.");
        }
      else
        {
          throw std::runtime_error(e.GetDescription());
        }
    }

}

void
Basler_ToF_Interface::do_start()
{
  BOOST_LOG_TRIVIAL(info) << "do_start()";
  if (free_running_)
    {
      setTriggerModeOn(false);
      setTriggerInterval_in_Hz(samplingsRate_in_Hz_);

    }
  else
    {
      setTriggerModeOn(true);
      setTriggerSourceToLine1();
    }
  minDepth_ = getMinDepth();
  maxDepth_ = getMaxDepth();

  threadSamplingLoop = std::thread(&Basler_ToF_Interface::StartSamplingLoop,
                                   this);
}

void
Basler_ToF_Interface::setTriggerSourceToLine1()
{
  GenApi::CEnumerationPtr ptrAutoExposureEnabled(
    m_Camera.GetParameter("TriggerSource"));

  ptrAutoExposureEnabled->FromString("Line1");
}


int64_t
Basler_ToF_Interface::getMinDepthLocalInMM()
{
  return minDepth_;
}

int64_t
Basler_ToF_Interface::getMaxDepthLocalInMM()
{
  return maxDepth_;
}



void
Basler_ToF_Interface::do_stop()
{
  BOOST_LOG_TRIVIAL(info) << "do_stop()";
  stopSamplingLoop = true;
  if (threadSamplingLoop.joinable())
    {
      BOOST_LOG_TRIVIAL(info) << "waiting for join()";
      threadSamplingLoop.join();
      BOOST_LOG_TRIVIAL(info)  << "did join()";
    }
  // m_Camera.Close();
}

void
Basler_ToF_Interface::do_deactivate()
{
  m_Camera.Close();
}

void
Basler_ToF_Interface::StartSamplingLoop()
{
  BOOST_LOG_TRIVIAL(info)  << "startSamplingLoopx()";
  try
    {
      m_nBuffersGrabbed = 0;
      // Acquire images until the call-back onImageGrabbed indicates to stop acquisition.
      // 5 buffers are used (round-robin).
      m_Camera.GrabContinuous(15, (samplingsRate_in_Hz_ * 1000 * 1.2), this,
                              &Basler_ToF_Interface::onImageGrabbed);

      // Clean-up
      //  m_Camera.Close ();
    }
  catch (const GenICam::GenericException& e)
    {
      cerr << "Exception occurred: " << e.GetDescription() << endl;
      // After successfully opening the camera, the IsConnected method can be used
      // to check if the device is still connected.
      if (m_Camera.IsOpen() && !m_Camera.IsConnected())
        {
          cerr << "Camera has been removed." << endl;
          throw std::runtime_error("Camera has been removed.");
        }
      else
        {
          throw std::runtime_error(e.GetDescription());
        }
    }
}

/// TODO Show we avoid throwing exception from inside here? and just signal via variable as in example?
bool
Basler_ToF_Interface::onImageGrabbed(GrabResult grabResult, BufferParts parts)
{
  if (grabResult.status == GrabResult::Timeout)
    {
      cerr << "Timeout occurred. Acquisition stopped." << endl;
      // The timeout might be caused by a removal of the camera. Check if the camera
      // is still connected.m_Camera
      if (!m_Camera.IsConnected())
        {
          cerr << "Camera has been removed." << endl;
          throw std::runtime_error("Camera has been removed.");
        }

      return false; // Indicate to stop acquisition
    }
  m_nBuffersGrabbed++;
  if (grabResult.status != GrabResult::Ok)
    {
      cerr << "Image " << m_nBuffersGrabbed << "was not grabbed." << endl;
    }
  else
    {
      // Retrieve the values for the center pixel
      const int width = (int) parts[0].width;
      const int height = (int) parts[0].height;
      const int x = (int)(0.5 * width);
      const int y = (int)(0.5 * height);
      /* CToFCamera::Coord3D *p3DCoordinate = (CToFCamera::Coord3D*) parts[0].pData
      + y * width + x;
       uint16_t *pIntensity = (uint16_t*) parts[1].pData + y * width + x;
       uint16_t *pConfidence = (uint16_t*) parts[2].pData + y * width + x;

       BOOST_LOG_TRIVIAL (info)  << "BASLERTOF::before operationXXXYYYYYY";
       BOOST_LOG_TRIVIAL (info)  << "Center pixelc of image " << setw (2) << m_nBuffersGrabbed
      << ": ";
       cout.setf (ios_base::fixed);
       cout.precision (1);
       if (p3DCoordinate->IsValid ())
      cout << "x=" << setw (6) << p3DCoordinate->x << " y=" << setw (6)
       << p3DCoordinate->y << " z=" << setw (6) << p3DCoordinate->z;
       else
      cout << "x=   n/a y=   n/a z=   n/a";

      */
      /*   cout << " intensity=" << setw (5) << *pIntensity << " confidence="
       << setw (5) << *pConfidence << endl;
      */

      clock::time_point next = clock::now();
      BOOST_LOG_TRIVIAL(info)  << "BASLERTOF::before operation";
      operation_((unsigned char *)&parts, std::chrono::duration_cast<std::chrono::microseconds>(next.time_since_epoch()).count());


      uint16_t *rangePix = (uint16_t *)parts[0].pData+ y * width + x;
      int rangeSize = (int)parts[0].size;
      BOOST_LOG_TRIVIAL(info) << "Test range parts[0]: " << setw(5)  << *rangePix;
      BOOST_LOG_TRIVIAL(info) << "Test rangeSize parts[0]: " << setw(5)  << rangeSize << " Calculation(640*480*2)="<< (640*480*2);



      uint16_t *conPix = (uint16_t *)parts[1].pData+ y * width + x;
      BOOST_LOG_TRIVIAL(info)  << "Test parts[1]: " << setw(5)  << *conPix;
      int rangeSize1 = (int)parts[1].size;
      BOOST_LOG_TRIVIAL(info) <<  "Test rangeSize parts[1]: " << setw(5)  << rangeSize
                              << " Calculation(640*480*2)="<< (640*480*2);
      if (parts[1].partType == Intensity)
        BOOST_LOG_TRIVIAL(info) << "parts[1] == Intensity";
      if (parts[1].partType == Confidence)
        BOOST_LOG_TRIVIAL(info) << "parts[1] == Confidence";

      uint16_t *p = (uint16_t *)parts[0].pData;
      BOOST_LOG_TRIVIAL(info) << "minDepth_:" << setprecision(5) << minDepth_;
      BOOST_LOG_TRIVIAL(info) << "maxDepth_:" << setprecision(5) << maxDepth_;

      //float minDept_meter =  minDept_/100.
      //float maxDepth_meter maxDepth_)/ std::numeric_limits<uint16_t>::max();
      int numberOfPixels = width*height;
      float arr[1000000];
      bool confidence[1000000];
      uint16_t *pConfidenceArr = (uint16_t *)parts[1].pData;

      for (int i= 0; i < numberOfPixels; i++)
        {
          float f = (minDepth_+(p[i]*maxDepth_)* (1./std::numeric_limits<uint16_t>::max()))*0.001;
          arr[i] = f;

          //confidence
          if ((p[i]==0) || (pConfidenceArr[i] == 0))
            {
              confidence[i] = false;
            }
          else
            {
              confidence[i] = true;
            }


          if (i==(numberOfPixels/2- x))
            {
              BOOST_LOG_TRIVIAL(info) << "Mid-pixel p["<<i<<"]:" << setprecision(5) << p[i] <<" => "
                                      << setprecision(5) << f << " [meter]"
                                      << " Confidence: " <<std::boolalpha << confidence[i];
            }
        }

    }
  return !stopSamplingLoop;
}
