//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2018
// European Synchrotron Radiation Facility
// BP 220, Grenoble 38043
// FRANCE
//
// This is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//###########################################################################

/*************************************************************************/
/*! 
 *  \file   SlsJungfrauCamera.cpp
 *  \brief  SlsJungfrau detector hardware class implementation
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************/

#include <cstdlib>

#include "lima/Exceptions.h"
#include "SlsJungfrauCamera.h"

#include <slsDetectorUsers.h>
#include <sls_detector_defs.h>

using namespace lima;
using namespace lima::SlsJungfrau;

#include <cmath>

/************************************************************************
 * \brief constructor
 * \param in_config_file_name complete path to the configuration file
 * \param in_readout_time_sec readout time in seconds
 * \param in_receiver_fifo_depth Number of frames in the receiver memory
 * \param in_frame_packet_number Number of packets we should get in each receiver frame
 ************************************************************************/
Camera::Camera(const std::string & in_config_file_name   ,
               const double        in_readout_time_sec   ,
               const long          in_receiver_fifo_depth,
               const long          in_frame_packet_number) : m_thread(*this), m_frames_manager(*this)
{
    DEB_CONSTRUCTOR();

    m_detector_control    = NULL;
    m_detector_receivers  = NULL;
    m_bit_depth           = 0;
    m_max_width           = 0;
    m_max_height          = 0;
    m_width               = 0; 
    m_height              = 0;
    m_exposure_time       = 0.0;
    m_latency_time        = 0.0;
    m_delay_after_trigger = 0.0;
    m_threshold_energy_eV = 0.0;
    m_status              = Camera::Idle; // important for the calls of updateTimes & updateTriggerData in the init method.
    m_clock_divider       = Camera::FullSpeed;

    m_readout_time_sec    = in_readout_time_sec   ;
    m_receiver_fifo_depth = in_receiver_fifo_depth;
    m_frame_packet_number = static_cast<uint32_t>(in_frame_packet_number);

    m_detector_type             = "undefined";
    m_detector_model            = "undefined";
    m_detector_firmware_version = "undefined";
    m_detector_software_version = "undefined";
    m_module_firmware_version   = "undefined";

    init(in_config_file_name);
}

/************************************************************************
 * \brief destructor
 ************************************************************************/
Camera::~Camera()
{
    DEB_DESTRUCTOR();

    // stopping the acquisition
    stopAcq();

    // releasing the detector control instance
    DEB_TRACE() << "Camera::Camera - releasing the detector control instance";

    if(m_detector_control != NULL)
    {
        m_detector_control->setReceiverOnline(slsDetectorDefs::OFFLINE_FLAG);
        m_detector_control->setOnline(slsDetectorDefs::OFFLINE_FLAG);

        delete m_detector_control;
        m_detector_control = NULL;
    }

    // releasing the controller class for detector receivers functionalities
    DEB_TRACE() << "Camera::Camera - releasing the controller class for detector receivers functionalities";
    
    if(m_detector_receivers != NULL)
    {
        delete m_detector_receivers;
        m_detector_receivers = NULL;
    }
}

/************************************************************************
 * \brief inits the camera while setting the configuration file name
 * \param in_config_file_name complete path to the configuration file
 ************************************************************************/
void Camera::init(const std::string & in_config_file_name)
{
    DEB_MEMBER_FUNCT();

    DEB_TRACE() << "config file name :" << in_config_file_name;

    int result;

    // before, cleaning the shared memory
    // cleanSharedMemory();

    // initing the class attributes
    m_config_file_name = in_config_file_name;

    // creating the controller class for detector receivers functionalities
    // CameraReceivers needs direct access to camera (*this)
    m_detector_receivers = new CameraReceivers(*this);

    // creating the receivers (just one for Jungfrau)
    m_detector_receivers->init(m_config_file_name);

    // creating the detector control instance
    int id = 0;

    m_detector_control = new slsDetectorUsers(id);

    // configuration file is used to properly configure advanced settings in the shared memory
    result = m_detector_control->readConfigurationFile(m_config_file_name);

    if(result == slsDetectorDefs::FAIL)
    {
        THROW_HW_FATAL(ErrorType::Error) << "readConfigurationFile failed! Could not initialize the camera!";
    }

    // Setting the detector online
    m_detector_control->setOnline(slsDetectorDefs::ONLINE_FLAG);

    // Connecting to the receiver
    m_detector_control->setReceiverOnline(slsDetectorDefs::ONLINE_FLAG);

    // getting the bit depth of the camera
    m_bit_depth = m_detector_control->setBitDepth(SLS_GET_VALUE);

    // getting the maximum detector size
    result = m_detector_control->getMaximumDetectorSize(m_max_width, m_max_height);

    if(result == slsDetectorDefs::FAIL)
    {
        THROW_HW_FATAL(ErrorType::Error) << "getMaximumDetectorSize failed! Could not initialize the camera!";
    }

    // getting the detector size to be sure
    int x0;
    int y0;

    result = m_detector_control->getDetectorSize(x0, y0, m_width, m_height);

    if(result == slsDetectorDefs::FAIL)
    {
        THROW_HW_FATAL(ErrorType::Error) << "getDetectorSize failed! Could not initialize the camera!";
    }

    // disabling the file write by the camera
    m_detector_control->enableWriteToFile(slsDetectorDefs::DISABLED);

    // setting the receiver fifo depth (number of frames in the receiver memory)
    // in a next sdk release, we could have a method to call.
    // At the moment, we should use the put command.
    {
        std::stringstream tempStream;
        tempStream << "rx_fifodepth " << m_receiver_fifo_depth;
        setCmd(tempStream.str());
    }

    // initing the internal copies of exposure & latency times
    updateTimes();

    // initing the internal copies of trigger mode label, number of cyles, number of frames per cycle, number of frames
    updateTriggerData();

    // initing some const data
    // Module firmware version does not exist for Jungfrau
    m_detector_type             = m_detector_control->getDetectorDeveloper();
    m_detector_model            = m_detector_control->getDetectorType();
    m_detector_firmware_version = convertVersionToString(m_detector_control->getDetectorFirmwareVersion());
    m_detector_software_version = convertVersionToString(m_detector_control->getDetectorSoftwareVersion());

    // starting the acquisition thread
    m_thread.start();

    // logging some versions informations
    DEB_TRACE() << "Module   Firmware Version : " << getModuleFirmwareVersion  ();
    DEB_TRACE() << "Detector Firmware Version : " << getDetectorFirmwareVersion();
    DEB_TRACE() << "Detector Software Version : " << getDetectorSoftwareVersion();
}

/************************************************************************
 * \brief cleans the shared memory used by the camera
 ************************************************************************/
void Camera::cleanSharedMemory()
{
    DEB_MEMBER_FUNCT();

    std::string cmd = "ipcs -m | grep -E '^0x000016[0-9a-z]{2}' | "
                      "awk '{print $2}' | while read m; do ipcrm -m $m; done";

    std::system(cmd.c_str());
}

/************************************************************************
 * \brief creates an autolock mutex for sdk methods access
 ************************************************************************/
lima::AutoMutex Camera::sdkLock() const
{
    DEB_MEMBER_FUNCT();
    return lima::AutoMutex(m_sdk_cond.mutex());
}

//==================================================================
// Related to HwInterface
//==================================================================
//------------------------------------------------------------------
// acquisition management
//------------------------------------------------------------------
/*******************************************************************
 * \brief prepares the acquisition
 *******************************************************************/
void Camera::prepareAcq()
{
    DEB_MEMBER_FUNCT();

    // Only snap is allowed
    {
        int64_t nb_frames = getNbFrames();

        if(nb_frames == 0LL)
            THROW_HW_ERROR(ErrorType::Error) << "Start mode is not allowed for this device! Please use Snap mode.";
    }


    // reset the number of caught frames in the sdk
    // in the next sdk release, we will use the method int resetFramesCaughtInReceiver().
    // At the moment, we should use the put command.
    // the given value 0 is just because put requires an argument, so it can be anything.
    setCmd("resetframescaught 0");

    // clear the frames containers
    m_frames_manager.clear();
}

/*******************************************************************
 * \brief starts the acquisition (start/snap)
 *******************************************************************/
void Camera::startAcq()
{
    DEB_MEMBER_FUNCT();

    m_thread.sendCmd(CameraThread::StartAcq);
    m_thread.waitNotStatus(CameraThread::Idle);
}

/*******************************************************************
 * \brief stops the acquisition
 *******************************************************************/
void Camera::stopAcq()
{
    DEB_MEMBER_FUNCT();

	DEB_TRACE() << "executing StopAcq command...";

    if(m_thread.getStatus() != CameraThread::Error)
    {
        m_thread.execStopAcq();

        // Waiting for thread to finish or to be in error
        m_thread.waitNotStatus(CameraThread::Running);
    }

    // thread in error
    if(m_thread.getStatus() == CameraThread::Error)
    {
        // aborting & restart the thread
        m_thread.abort();
    }
}

/************************************************************************
 * \brief Acquisition data management
 * \param m_receiver_index receiver index
 * \param in_frame_index frame index (starts at 0)
 * \param in_packet_number number of packets caught for this frame
 * \param in_timestamp time stamp in 10MHz clock
 * \param in_data_pointer frame image pointer
 * \param in_data_size frame image size 
 ************************************************************************/
void Camera::acquisitionDataReady(const int      in_receiver_index,
                                  const uint64_t in_frame_index   ,
                                  const uint32_t in_packet_number ,
                                  const uint64_t in_timestamp     ,
                                  const char *   in_data_pointer  ,
                                  const uint32_t in_data_size     )
{
    DEB_MEMBER_FUNCT();

    // the start of this call is in a sls callback, so we should be as fast as possible
    // to avoid frames lost.
    StdBufferCbMgr & buffer_mgr  = m_buffer_ctrl_obj.getBuffer();
    lima::FrameDim   frame_dim   = buffer_mgr.getFrameDim();
    int              mem_size    = frame_dim.getMemSize();

    // checking the frame size
    if(mem_size != in_data_size)
    {
        DEB_TRACE() << "Incoherent sizes during frame copy process : " << 
                       "sls size (" << in_data_size << ")" <<
                       "lima size (" << mem_size << ")";
    }
    else
    {
        m_frames_manager.manageFirstFrameTreatment(in_frame_index, in_timestamp);

        uint64_t relative_frame_index = m_frames_manager.computeRelativeFrameIndex(in_frame_index);

        // ckecking if there is no packet lost.
        if(in_packet_number == m_frame_packet_number)
        {
            uint64_t relative_timestamp   = m_frames_manager.computeRelativeTimestamp (in_timestamp  );

            // copying the frame
            char * dest_buffer = static_cast<char *>(buffer_mgr.getFrameBufferPtr(relative_frame_index));
            memcpy(dest_buffer, in_data_pointer, in_data_size);

            // giving the frame to the frames manager
            CameraFrame frame(relative_frame_index, in_packet_number, relative_timestamp);
            m_frames_manager.addReceived(in_receiver_index, frame);
        }
        else
        // rejected frame
        {
            DEB_TRACE() << "Rejected Frame [ " << relative_frame_index << ", " << in_packet_number << " ]";
        }
    }
}

/************************************************************************
 * \brief start receiver listening mode
 * \return OK or FAIL
 ************************************************************************/
int Camera::startReceiver()
{
    // protecting the sdk concurrent access
    lima::AutoMutex sdk_mutex = sdkLock(); 

    return m_detector_control->startReceiver();
}

/************************************************************************
 * \brief stop receiver listening mode
 * \return OK or FAIL
 ************************************************************************/
int Camera::stopReceiver()
{
    // protecting the sdk concurrent access
    lima::AutoMutex sdk_mutex = sdkLock(); 

    return m_detector_control->stopReceiver();
}

/************************************************************************
 * \brief start detector real time acquisition in non blocking mode
 * \return OK if all detectors are properly started, FAIL otherwise
 ************************************************************************/
int Camera::startAcquisition()
{
    // protecting the sdk concurrent access
    lima::AutoMutex sdk_mutex = sdkLock(); 

    return m_detector_control->startAcquisition();
}

/************************************************************************
 * \brief stop detector real time acquisition
 * \return OK if all detectors are properly stopped, FAIL otherwise
 ************************************************************************/
int Camera::stopAcquisition()
{
    // protecting the sdk concurrent access
    lima::AutoMutex sdk_mutex = sdkLock(); 

    return m_detector_control->stopAcquisition();
}

//------------------------------------------------------------------
// status management
//------------------------------------------------------------------
/************************************************************************
 * \brief returns the current detector status
 * \return current hardware status
 ************************************************************************/
Camera::Status Camera::getDetectorStatus()
{
    DEB_MEMBER_FUNCT();

    // protecting the sdk concurrent access
    lima::AutoMutex sdk_mutex = sdkLock(); 

    Camera::Status result;

    // In auto mode:
    // Detector starts with idle to running and at end of acquisition, returns to idle.
    //
    // In trigger mode:
    // Detector starts with idle to waiting and stays in waiting until it gets a trigger.
    // Upon trigger, it switches to running and goes back to waiting again until it gets a trigger.
    // Upon end of acquisition (after it get n triggers and m frames as configured), it will go to idle.
    //
    // If one explicitly calls stop acquisition:
    // Detectors goes from running to idle.
    //
    // If there is fifo overflow:
    // the detector will go to stopped state and stay there until the fifos are cleared.
    // It should be concidered as an error.    

    // getting the detector status
    int status = m_detector_control->getDetectorStatus();

    // ready to start acquisition or acquisition stopped externally
    if((status == slsDetectorDefs::runStatus::IDLE   ) ||
       (status == slsDetectorDefs::runStatus::STOPPED))
    {
        result = Camera::Idle;
    }
    else
    // waiting for trigger or gate signal 
    if(status == slsDetectorDefs::runStatus::WAITING)
    {
        result = Camera::Waiting;
    }
    else
    // acquisition is running 
    if(status == slsDetectorDefs::runStatus::RUNNING)
    {
        result = Camera::Running;
    }
    else
    // fifo full or unexpected error 
    if(status == slsDetectorDefs::runStatus::ERROR)
    {
        result = Camera::Error;
    }
    else
    // these states do not apply to Jungfrau
    // RUN_FINISHED : acquisition not running but data in memory 
    // TRANSMITTING : acquisition running and data in memory 
    if((status == slsDetectorDefs::runStatus::RUN_FINISHED) ||
       (status == slsDetectorDefs::runStatus::TRANSMITTING))
    {
        THROW_HW_ERROR(ErrorType::Error) << "Camera::getStatus failed! An unexpected state was returned!";
    }
    else
    // impossible state 
    {
        THROW_HW_ERROR(ErrorType::Error) << "Camera::getStatus failed! An unknown state was returned!";
    }

    // making a copy of the latest read status
    m_status = result;

    return result;
}

/************************************************************************
 * \brief returns the current camera status
 * \return current hardware status
 ************************************************************************/
Camera::Status Camera::getStatus()
{
    DEB_MEMBER_FUNCT();

    Camera::Status result;

    int thread_status = m_thread.getStatus();

    // error during the acquisition management ?
    // the device becomes in error state.
    if(thread_status == CameraThread::Error)
    {
        result = Camera::Error;
    }
    else
    // the device is in acquisition.
    // During an aquisition, this is the acquisition thread which read the hardware camera status.
    // So we use the latest read camera status.
    if(thread_status == CameraThread::Running)
    {
        result = Camera::Running;
    }
    else
    // the device is not in acquisition or in error, so we can read the hardware camera status
    {
        result = getDetectorStatus();
    }

    return result;
}

//------------------------------------------------------------------
// frames management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets the number of acquired frames
 * \return current acquired frames number
 *******************************************************************/
uint64_t Camera::getNbAcquiredFrames() const
{
    DEB_MEMBER_FUNCT();

    // reading in the number of treated frames in the frame manager
    return m_frames_manager.getNbTreatedFrames();
}

/************************************************************************
 * \brief get the number of frames in the containers
 * \return the number of frames by type
 ************************************************************************/
void Camera::getNbFrames(size_t & out_received, 
                         size_t & out_complete,
                         size_t & out_treated ) const
{
    DEB_MEMBER_FUNCT();

    // reading in the number of frames in the frame manager
    m_frames_manager.getNbFrames(out_received, out_complete, out_treated);
}

/*******************************************************************
 * \brief Gets the frame manager const access
 * \return frame manager const reference
 *******************************************************************/
const CameraFrames & Camera::getFrameManager() const
{
    DEB_MEMBER_FUNCT();
    return m_frames_manager;
}

/*******************************************************************
 * \brief Gets the frame manager access
 * \return frame manager reference
 *******************************************************************/
CameraFrames & Camera::getFrameManager()
{
    DEB_MEMBER_FUNCT();
    return m_frames_manager;
}

//==================================================================
// Related to HwDetInfoCtrlObj
//==================================================================
//------------------------------------------------------------------
// image size management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets the maximum image width
 * \return maximum image width
 *******************************************************************/
unsigned short Camera::getMaxWidth() const
{
    DEB_MEMBER_FUNCT();
    return m_max_width;
}

/*******************************************************************
 * \brief Gets the maximum image height
 * \return maximum image height
 *******************************************************************/
unsigned short Camera::getMaxHeight() const
{
    DEB_MEMBER_FUNCT();
    return m_max_height;
}

/*******************************************************************
 * \brief Gets the image width
 * \return image width
 *******************************************************************/
unsigned short Camera::getWidth() const
{
    DEB_MEMBER_FUNCT();
    return m_width;
}

/*******************************************************************
 * \brief Gets the image height
 * \return image height
 *******************************************************************/
unsigned short Camera::getHeight() const
{
    DEB_MEMBER_FUNCT();
    return m_height;
}

//------------------------------------------------------------------
// current image type management
//------------------------------------------------------------------
/*******************************************************************
 * \brief gets the default image type
 *******************************************************************/
lima::ImageType Camera::getDefImageType() const
{
    DEB_MEMBER_FUNCT();
    return lima::ImageType::Bpp16;
}

/*******************************************************************
 * \brief gets the current image type
 * \return current image type
*******************************************************************/
lima::ImageType Camera::getImageType() const
{
    DEB_MEMBER_FUNCT();

    lima::ImageType type;

    // getting the bit depth of the camera
    int bit_depth = m_bit_depth;

    switch(bit_depth)
    {
        case 16: 
            type = lima::ImageType::Bpp16;
            break;
        
        default:
            THROW_HW_ERROR(ErrorType::Error) << "This pixel format of the camera is not managed, only 16 bits cameras are managed!";
            break;
    }

    return type;
}

/*******************************************************************
 * \brief sets the current image type
 * \param in_type new image type
 *******************************************************************/
void Camera::setImageType(lima::ImageType in_type)
{
    DEB_MEMBER_FUNCT();

    int bit_depth;

    switch(in_type)
    {
        case lima::ImageType::Bpp16:
            bit_depth = 16;
            break;
        
        default:
            THROW_HW_ERROR(ErrorType::Error) << "This pixel format of the camera is not managed, only 16 bits cameras are managed!";
            break;
    }

    // protecting the sdk concurrent access
    {
        lima::AutoMutex sdk_mutex = sdkLock(); 

        // setting the bit depth of the camera
        m_detector_control->setBitDepth(bit_depth);
    }

    // store the value in a class variable
    m_bit_depth = bit_depth;
}

//------------------------------------------------------------------
// pixel size management
//------------------------------------------------------------------
/*******************************************************************
 * \brief gets the pixel size
 * \param out_x_size width pixel size
 * \param out_y_size height pixel size
 *******************************************************************/
void Camera::getPixelSize(double & out_x_size, double & out_y_size) const
{
    DEB_MEMBER_FUNCT();
    out_x_size = out_y_size = 75e-6;
}

//------------------------------------------------------------------
// detector info management
//------------------------------------------------------------------
/*******************************************************************
 * \brief gets the detector type
 * \return detector type
 *******************************************************************/
std::string Camera::getDetectorType() const
{
    DEB_MEMBER_FUNCT();
    return m_detector_type;
}

/*******************************************************************
 * \brief gets the detector model
 * \return detector model
 *******************************************************************/
std::string Camera::getDetectorModel() const
{
    DEB_MEMBER_FUNCT();
    return m_detector_model;
}

/*******************************************************************
 * \brief converts a version id to a string
 * \return version in string format (uppercase & hexa)
 *******************************************************************/
std::string Camera::convertVersionToString(int64_t in_version)
{
    std::stringstream tempStream;
    tempStream << "0x" << std::uppercase << std::hex << in_version;
    return tempStream.str();
}

/*******************************************************************
 * \brief gets Module Firmware Version
 * \return Module Firmware Version
 *******************************************************************/
std::string Camera::getModuleFirmwareVersion() const
{
    DEB_MEMBER_FUNCT();
    return m_module_firmware_version;
}

/*******************************************************************
 * \brief gets Detector Firmware Version
 * \return Detector Firmware Version
 *******************************************************************/
std::string Camera::getDetectorFirmwareVersion() const
{
    DEB_MEMBER_FUNCT();
    return m_detector_firmware_version;
}

/*******************************************************************
 * \brief gets Detector Software Version
 * \return Detector Software Version
 *******************************************************************/
std::string Camera::getDetectorSoftwareVersion() const
{
    DEB_MEMBER_FUNCT();
    return m_detector_software_version;
}

//==================================================================
// Related to HwSyncCtrlObj
//==================================================================
//------------------------------------------------------------------
// trigger mode management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Updates trigger data (mode, frame, cycle) using camera data 
 *******************************************************************/
void Camera::updateTriggerData()
{
    DEB_MEMBER_FUNCT();

    // during acquisition, camera data access is not allowed because it
    // could put the camera into an error state. 
    // So we give the latest read value.
    if(m_status == Camera::Idle)
    {
        // protecting the sdk concurrent access
        {
            lima::AutoMutex sdk_mutex = sdkLock(); 

            // getting the current trigger mode index
            int trigger_mode_index = m_detector_control->setTimingMode(SLS_GET_VALUE);

            // converting trigger mode index to trigger mode label
            m_trig_mode_label = slsDetectorUsers::getTimingMode(trigger_mode_index);
            
            // reading the number of frames per cycle
            m_nb_frames_per_cycle = m_detector_control->setNumberOfFrames(SLS_GET_VALUE);

            // reading the number of cycles
            m_nb_cycles = m_detector_control->setNumberOfCycles(SLS_GET_VALUE);
        }

        // computing the number of frames
        m_nb_frames = m_nb_cycles * m_nb_frames_per_cycle;

        // computing the lima trigger mode
        if(m_trig_mode_label == SLS_TRIGGER_MODE_AUTO)
        {
            m_trig_mode = lima::TrigMode::IntTrig;
        }
        else
        if(m_trig_mode_label == SLS_TRIGGER_MODE_TRIGGER)
        {
            if(m_nb_cycles == 1LL)
            {
                m_trig_mode = lima::TrigMode::ExtTrigSingle;
            }
            else
            {
                m_trig_mode = lima::TrigMode::ExtTrigMult;
            }
        }
        else
        {
            THROW_HW_ERROR(ErrorType::Error) << "updateTriggerData : This camera trigger Mode is not managed! (" << m_trig_mode_label << ")";
        }
    }
}

/*******************************************************************
 * \brief Checks the trigger mode validity
 * \return true if passed trigger mode is supported
 *******************************************************************/
bool Camera::checkTrigMode(lima::TrigMode in_trig_mode) const
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(in_trig_mode);
    bool valid_mode;    

    switch (in_trig_mode)
    {       
        case lima::TrigMode::IntTrig      :
        case lima::TrigMode::ExtTrigSingle:
        case lima::TrigMode::ExtTrigMult  :
            valid_mode = true;
            break;

        default:
            valid_mode = false;
            break;
    }

    return valid_mode;
}

/*******************************************************************
 * \brief Sets the trigger mode
 * \param in_mode needed trigger mode 
 *                (IntTrig, ExtTrigSingle, ExtTrigMultiple)
 *******************************************************************/
void Camera::setTrigMode(lima::TrigMode in_mode)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setTrigMode - " << DEB_VAR1(in_mode);

    // updating the internal copies of trigger mode label, number of cyles, number of frames per cycle, number of frames
    updateTriggerData();

    // trigger mode was already checked in the interface part, no need to do it again.
    std::string trig_mode;

    switch (in_mode)
    {       
        case lima::TrigMode::IntTrig:
            trig_mode = SLS_TRIGGER_MODE_AUTO;
            break;

        case lima::TrigMode::ExtTrigSingle:
        case lima::TrigMode::ExtTrigMult  :
            trig_mode = SLS_TRIGGER_MODE_TRIGGER; // same trigger mode, the difference will be in frames and cycles numbers
            break;

        default:
            THROW_HW_ERROR(ErrorType::Error) << "Cannot change the Trigger Mode of the camera, this mode is not managed:" << in_mode;
            break;
    }

    // protecting the sdk concurrent access
    {
        lima::AutoMutex sdk_mutex = sdkLock(); 

        // initing the number of frames per cycle and  number of cycles 
        // to avoid problems during the trigger mode change.
        m_nb_frames_per_cycle = m_detector_control->setNumberOfFrames(1);
        m_nb_cycles           = m_detector_control->setNumberOfCycles(1);

        // conversion of trigger mode label to trigger mode index
        int trigger_mode_index = slsDetectorUsers::getTimingMode(trig_mode);

        // apply the trigger change
        m_detector_control->setTimingMode(trigger_mode_index);

        // change the frames per cycle and the cycles values
        int64_t nb_frames_per_cycle;
        int64_t nb_cycles          ;

        switch (in_mode)
        {       
            case lima::TrigMode::IntTrig:
                nb_frames_per_cycle = m_nb_frames;
                nb_cycles           = 1LL;
                break;

            case lima::TrigMode::ExtTrigSingle:
                nb_frames_per_cycle = m_nb_frames;
                nb_cycles           = 1LL;
                break;

            case lima::TrigMode::ExtTrigMult:
                nb_frames_per_cycle = 1LL;
                nb_cycles           = m_nb_frames;
                break;

            default:
                break;
        }

        // setting the number of cycles
        m_nb_cycles = m_detector_control->setNumberOfCycles(nb_cycles);

        // setting the number of frames per cycle
        m_nb_frames_per_cycle = m_detector_control->setNumberOfFrames(nb_frames_per_cycle);
    }

    // reseting the number of frames to its old value to 
    // correctly set the nb frames per cycle and the number of cycles
    setNbFrames(m_nb_frames);
}

/*******************************************************************
 * \brief Gets the trigger mode
 * \return trigger mode
 *******************************************************************/
lima::TrigMode Camera::getTrigMode()
{
    DEB_MEMBER_FUNCT();

    // updating the internal copies of trigger mode label, number of cyles, number of frames per cycle, number of frames
    updateTriggerData();

    return m_trig_mode;
}

//------------------------------------------------------------------
// times management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Updates exposure & latency times using camera data 
 *******************************************************************/
void Camera::updateTimes()
{
    DEB_MEMBER_FUNCT();
    
    // during acquisition, camera data access is not allowed because it
    // could put the camera into an error state. 
    // So we give the latest read value.
    if(m_status == Camera::Idle)
    {
        double exposure_period;

        // protecting the sdk concurrent access
        {
            lima::AutoMutex sdk_mutex = sdkLock(); 

            m_exposure_time = m_detector_control->setExposureTime  (SLS_GET_VALUE, true); // in seconds
            exposure_period = m_detector_control->setExposurePeriod(SLS_GET_VALUE, true); // in seconds
        }

        // compute latency time
        // exposure period = exposure time + latency time
        m_latency_time = exposure_period - m_exposure_time;
    }
}

//------------------------------------------------------------------
// exposure time management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets the exposure time
 * \return exposure time
 *******************************************************************/
double Camera::getExpTime()
{
    DEB_MEMBER_FUNCT();

    // updating the internal copies of exposure & latency times
    updateTimes();

    DEB_RETURN() << DEB_VAR1(m_exposure_time);
    return m_exposure_time;
}

/*******************************************************************
 * \brief Sets the exposure time
 * \param in_exp_time needed exposure time
*******************************************************************/
void Camera::setExpTime(double in_exp_time)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setExpTime - " << DEB_VAR1(in_exp_time) << " (sec)";

    double exposure_time  ;
    double exposure_period;

    // protecting the sdk concurrent access
    {
        lima::AutoMutex sdk_mutex = sdkLock(); 

        // if we change the exposure time, we need to update also the exposure period
        // exposure period = exposure time + latency time
        exposure_time   = m_detector_control->setExposureTime  (in_exp_time, true); // in seconds
        exposure_period = m_detector_control->setExposurePeriod(exposure_time + m_latency_time, true); // in seconds
    }

    // updating the internal copies of exposure & latency times
    updateTimes();
}

//------------------------------------------------------------------
// latency time management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets the latency time
 * \return latency time
 *******************************************************************/
double Camera::getLatencyTime()
{
    DEB_MEMBER_FUNCT();

    // updating the internal copies of exposure & latency times
    updateTimes();

    DEB_RETURN() << DEB_VAR1(m_latency_time);
    return m_latency_time;
}

/*******************************************************************
 * \brief Sets the latency time
 * \param in_latency_time needed latency time
*******************************************************************/
void Camera::setLatencyTime(double in_latency_time)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setLatencyTime - " << DEB_VAR1(in_latency_time) << " (sec)";

    // protecting the sdk concurrent access
    {
        lima::AutoMutex sdk_mutex = sdkLock(); 

        // exposure period = exposure time + latency time
        m_detector_control->setExposurePeriod(m_exposure_time + in_latency_time, true); // in seconds
    }

    // updating the internal copies of exposure & latency times
    updateTimes();
}

//------------------------------------------------------------------
// number of frames management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Sets the number of frames
 * \param in_nb_frames number of needed frames
*******************************************************************/
void Camera::setNbFrames(int64_t in_nb_frames)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setNbFrames - " << DEB_VAR1(in_nb_frames);
    if(in_nb_frames < 0LL)
        throw LIMA_HW_EXC(InvalidValue, "Invalid nb of frames");

    // updating the internal copies of trigger mode label, number of cyles, number of frames per cycle, number of frames
    updateTriggerData();

    int64_t nb_frames_per_cycle;
    int64_t nb_cycles          ;

    switch (m_trig_mode)
    {       
        case lima::TrigMode::IntTrig:
            nb_frames_per_cycle = in_nb_frames;
            nb_cycles           = 1LL;
            break;

        case lima::TrigMode::ExtTrigSingle:
            nb_frames_per_cycle = in_nb_frames;
            nb_cycles           = 1LL;
            break;

        case lima::TrigMode::ExtTrigMult:
            nb_frames_per_cycle = 1LL;
            nb_cycles           = in_nb_frames;
            break;

        default:
            break;
    }

    // protecting the sdk concurrent access
    {
        lima::AutoMutex sdk_mutex = sdkLock(); 

        // setting the number of cycles
        m_nb_cycles = m_detector_control->setNumberOfCycles(nb_cycles);

        // setting the number of frames per cycle
        m_nb_frames_per_cycle = m_detector_control->setNumberOfFrames(nb_frames_per_cycle);
    }

    // updating the internal copies of trigger mode label, number of cyles, number of frames per cycle, number of frames
    updateTriggerData();
}

/*******************************************************************
 * \brief Gets the number of frames
 * \return number of frames
 *******************************************************************/
int64_t Camera::getNbFrames()
{
    DEB_MEMBER_FUNCT();

    // updating the internal copies of trigger mode label, number of cyles, number of frames per cycle, number of frames
    updateTriggerData();

    return m_nb_frames;
}

/*******************************************************************
 * \brief Gets the internal number of frames (for thread access)
 * \return number of frames
 *******************************************************************/
uint64_t Camera::getInternalNbFrames()
{
    DEB_MEMBER_FUNCT();
    return static_cast<uint64_t>(m_nb_frames);
}

//------------------------------------------------------------------
// valid ranges management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets exposure and latency ranges
 * \return valid ranges structure
 *******************************************************************/
HwSyncCtrlObj::ValidRangesType Camera::getValidRanges() const
{
    DEB_MEMBER_FUNCT();

    double          min_time = 10e-9;
    double          max_time = 10e+9;

    HwSyncCtrlObj::ValidRangesType valid_ranges;

    valid_ranges.min_exp_time = min_time;
    valid_ranges.max_exp_time = max_time;
    valid_ranges.min_lat_time = m_readout_time_sec;
    valid_ranges.max_lat_time = max_time;

    return valid_ranges;
}

//==================================================================
// Related to BufferCtrl object
//==================================================================
/*******************************************************************
 * \brief Gets the internal buffer manager
 * \return the internal buffer manager
 *******************************************************************/
HwBufferCtrlObj * Camera::getBufferCtrlObj()
{
    DEB_MEMBER_FUNCT();
    return &m_buffer_ctrl_obj;
}

//==================================================================
// Related to commands (put & get)
//==================================================================
/*******************************************************************
 * \brief Converts a standard string to args arguments
 * \param in_command command in command line format
 * \param out_argv output c-strings c-array
 * \param out_argc output number of arguments of out_argv
 *******************************************************************/
void Camera::convertStringToArgs(const std::string & in_command,
                                 char  * *         & out_argv  ,
                                 int               & out_argc  )
{
	DEB_MEMBER_FUNCT();

    out_argv = NULL;
    out_argc = 0   ;

    // filling a string vector with the command line elements
    std::vector<std::string> elements;
    std::stringstream ss(in_command);

	while (ss) 
    {
        std::string element;
		ss >> element;

        if(element.size() > 0)
        {
            elements.push_back(element);
        }
	}

    // setting argc value
    out_argc = elements.size();

    // allocating argv array
	out_argv = new char * [out_argc];
    
    // filling argv array
	for (int element_index = 0; element_index < out_argc; element_index++)
    {
        out_argv[element_index] = new char[elements[element_index].size() + 1]; // adding the allocation of end of c-string 
        strcpy(out_argv[element_index], elements[element_index].c_str()); // copying the string including the eos
    }
}

/*******************************************************************
 * \brief Releases args arguments
 * \param in_out_argv output c-strings c-array
 * \param in_out_argc output number of arguments of out_argv
 *******************************************************************/
void Camera::releaseArgs(char * * & in_out_argv  ,
                         int      & in_out_argc  )
{
	DEB_MEMBER_FUNCT();

    if(in_out_argv != NULL)
    {
        // releasing the c_strings array content
        for (int element_index = 0; element_index < in_out_argc; element_index++)
        {
            delete [] in_out_argv[element_index];
        }

        // releasing the c_strings array
        delete [] in_out_argv;

        in_out_argv = NULL;
        in_out_argc = 0   ;
    }
}

/*******************************************************************
 * \brief Executes a set command
 * \param in_command command in command line format
 * \return the command result
 *******************************************************************/
std::string Camera::setCmd(const std::string & in_command)
{
	DEB_MEMBER_FUNCT();
    DEB_PARAM() << "Camera::setCmd - execute set command:\"" << in_command << "\"";

    char  * *   argv  ;
    int         argc  ;
    std::string result;

    convertStringToArgs(in_command, argv, argc);

    if(argc > 0)
    {
        // protecting the sdk concurrent access
        lima::AutoMutex sdk_mutex = sdkLock(); 

        result = m_detector_control->putCommand(argc, argv);
    }

    releaseArgs(argv, argc);

	DEB_RETURN() << "result=\"" << result << "\"";
    return result;
}

/*******************************************************************
 * \brief Executes a get command
 * \param in_command command in command line format
 * \return the command result
 *******************************************************************/
std::string Camera::getCmd(const std::string & in_command)
{
	DEB_MEMBER_FUNCT();
    DEB_PARAM() << "Camera::getCmd - execute get command:\"" << in_command << "\"";

    char  * *   argv  ;
    int         argc  ;
    std::string result;

    convertStringToArgs(in_command, argv, argc);

    if(argc > 0)
    {
        // protecting the sdk concurrent access
        lima::AutoMutex sdk_mutex = sdkLock(); 

        result = m_detector_control->getCommand(argc, argv);
    }

    releaseArgs(argv, argc);

	DEB_RETURN() << "result=\"" << result << "\"";
    return result;
}

//==================================================================
// Related to specifics attributes
//==================================================================
/*******************************************************************
 * \brief Gets the threshold energy in eV
 * \return threshold energy in eV
 *******************************************************************/
int Camera::getThresholdEnergy()
{
    DEB_MEMBER_FUNCT();

    // during acquisition, camera data access is not allowed because it
    // could put the camera into an error state. 
    // So we give the latest read value.
    if(m_status == Camera::Idle)
    {
        int threshold_energy_eV;

        // protecting the sdk concurrent access
        {
            lima::AutoMutex sdk_mutex = sdkLock(); 

            threshold_energy_eV = m_detector_control->getThresholdEnergy();
        }

        if(threshold_energy_eV == slsDetectorDefs::FAIL)
        {
            THROW_HW_ERROR(ErrorType::Error) << "getThresholdEnergy failed!";
        }

        m_threshold_energy_eV = threshold_energy_eV;
    }

    DEB_RETURN() << DEB_VAR1(m_threshold_energy_eV);
    return m_threshold_energy_eV;
}

/*******************************************************************
 * \brief Sets the threshold energy in eV
 * \param in_threshold_energy_eV needed threshold energy in eV
*******************************************************************/
void Camera::setThresholdEnergy(int in_threshold_energy_eV)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setThresholdEnergy - " << DEB_VAR1(in_threshold_energy_eV) << " (eV)";

    int result;

    // protecting the sdk concurrent access
    {
        lima::AutoMutex sdk_mutex = sdkLock(); 

        result = m_detector_control->setThresholdEnergy(in_threshold_energy_eV);
    }

    if(result == slsDetectorDefs::FAIL)
    {
        THROW_HW_ERROR(ErrorType::Error) << "setThresholdEnergy failed!";
    }
}

/*******************************************************************
 * \brief Gets the clock divider
 * \return clock divider
 *******************************************************************/
lima::SlsJungfrau::Camera::ClockDivider Camera::getClockDivider()
{
    DEB_MEMBER_FUNCT();

    // during acquisition, camera data access is not allowed because it
    // could put the camera into an error state. 
    // So we give the latest read value.
    if(m_status == Camera::Idle)
    {
        int clock_divider;

        // protecting the sdk concurrent access
        {
            lima::AutoMutex sdk_mutex = sdkLock(); 
            
            clock_divider = m_detector_control->setClockDivider(SLS_GET_VALUE);
        }

        m_clock_divider = static_cast<enum ClockDivider>(clock_divider);
    }

    DEB_RETURN() << DEB_VAR1(m_clock_divider);
    return m_clock_divider;
}

/*******************************************************************
 * \brief Sets the clock divider
 * \param in_clock_divider needed clock divider
*******************************************************************/
void Camera::setClockDivider(lima::SlsJungfrau::Camera::ClockDivider in_clock_divider)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setClockDivider - " << DEB_VAR1(in_clock_divider);

    // protecting the sdk concurrent access
    lima::AutoMutex sdk_mutex = sdkLock(); 

    m_detector_control->setClockDivider(static_cast<int>(in_clock_divider));
}

/*******************************************************************
 * \brief Gets the delay after trigger (in seconds)
 * \return delay after trigger (in seconds)
 *******************************************************************/
double Camera::getDelayAfterTrigger()
{
    DEB_MEMBER_FUNCT();

    // during acquisition, camera data access is not allowed because it
    // could put the camera into an error state. 
    // So we give the latest read value.
    if(m_status == Camera::Idle)
    {
        // protecting the sdk concurrent access
        lima::AutoMutex sdk_mutex = sdkLock(); 

        m_delay_after_trigger = m_detector_control->setDelayAfterTrigger(SLS_GET_VALUE, true); // in seconds
    }

    DEB_RETURN() << DEB_VAR1(m_delay_after_trigger);
    return m_delay_after_trigger;
}

/*******************************************************************
 * \brief Sets the delay after trigger (in seconds)
 * \param in_delay_after_trigger needed delay after trigger (in seconds)
*******************************************************************/
void Camera::setDelayAfterTrigger(double in_delay_after_trigger)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setDelayAfterTrigger - " << DEB_VAR1(in_delay_after_trigger) << " (sec)";

    // protecting the sdk concurrent access
    lima::AutoMutex sdk_mutex = sdkLock(); 

    m_detector_control->setDelayAfterTrigger(in_delay_after_trigger, true); // in seconds
}

//==================================================================
// Related to event control object
//==================================================================
/*******************************************************************
 * \brief Gets the Lima event control object
 * \return event control object pointer
*******************************************************************/
HwEventCtrlObj* Camera::getEventCtrlObj()
{
	return &m_event_ctrl_obj;
}

//========================================================================================
