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
 *  \author Cedric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
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
#include <iostream>
#include <fstream>

/************************************************************************
 * \brief constructor
 * \param in_config_file_name complete path to the configuration file
 * \param in_readout_time_sec readout time in seconds
 * \param in_receiver_fifo_depth Number of frames in the receiver memory
 * \param in_frame_packet_number Number of packets we should get in each receiver frame
 * \param in_gains_coeffs_file_name complete path of the gains coefficients file
 * \param in_pedestal_file_names complete path of the pedestal images
 * \param in_pedestal_nb_frames number of frames used to generate the pedestal images
 ************************************************************************/
Camera::Camera(const std::string &            in_config_file_name      ,
               const double                   in_readout_time_sec      ,
               const long                     in_receiver_fifo_depth   ,
               const long                     in_frame_packet_number   ,
               const std::string &            in_gains_coeffs_file_name,
               const std::vector<std::string> in_pedestal_file_names   ,
               const std::vector<long>        in_pedestal_nb_frames    ) : m_frames_manager(*this)
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
    m_gain_mode           = Camera::undefined;

    m_readout_time_sec    = in_readout_time_sec   ;
    m_receiver_fifo_depth = in_receiver_fifo_depth;
    m_frame_packet_number = static_cast<uint32_t>(in_frame_packet_number);
 
    m_gains_coeffs_file_name = in_gains_coeffs_file_name;
    m_pedestal_file_names    = in_pedestal_file_names;
    m_pedestal_nb_frames     = in_pedestal_nb_frames;  
    m_calibration_running    = false;
    m_dark_images_loaded     = false;

    m_detector_type             = "undefined";
    m_detector_model            = "undefined";
    m_detector_firmware_version = "undefined";
    m_detector_software_version = "undefined";
    m_module_firmware_version   = "undefined";

    init(in_config_file_name);

    // creating the camera thread
    m_thread = new CameraThread(*this);

    // starting the acquisition thread
    m_thread->start();

    // creating the camera calibration thread
    m_dark_thread = new CameraDarkThread(*this);

    // starting the acquisition thread
    m_dark_thread->start();
}

/************************************************************************
 * \brief destructor
 ************************************************************************/
Camera::~Camera()
{
    DEB_DESTRUCTOR();

    // stopping the acquisition and aborting the thread
    applyStopAcq(false, true);

    // releasing the camera thread
    delete m_thread;

    // stopping the calibration and aborting the thread
    applyStopCalibration(false, true);

    // releasing the camera thread
    delete m_dark_thread;

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

    m_detector_control = new slsDetectorUsers(result, id);

    if(result == slsDetectorDefs::FAIL)
    {
        THROW_HW_FATAL(ErrorType::Error) << "slsDetectorUsers constructor failed! Could not initialize the camera!";
    }

    // configuration file is used to properly configure advanced settings in the shared memory
    result = m_detector_control->readConfigurationFile(m_config_file_name);

    if(result == slsDetectorDefs::FAIL)
    {
        // cleaning the shared memory because sometimes it can be corrupted
        // It could help when the device will be restarted.
        cleanSharedMemory();

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
    m_detector_control->setReceiverFifoDepth(m_receiver_fifo_depth);

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

    // logging some versions informations
    DEB_TRACE() << "Module   Firmware Version : " << getModuleFirmwareVersion  ();
    DEB_TRACE() << "Detector Firmware Version : " << getDetectorFirmwareVersion();
    DEB_TRACE() << "Detector Software Version : " << getDetectorSoftwareVersion();

    // loading the gains'coefficients file
    loadGainsCoeffsFile(m_gains_coeffs_file_name, m_gains_coeffs, m_width, m_height);

    // loading the dark images files
    if(m_pedestal_file_names.size() == SLS_NUMBER_OF_DARK_IMAGES)
    {
        std::size_t nb_darks;

        for(nb_darks = 0 ; nb_darks < m_pedestal_file_names.size() ; nb_darks++)
        {
            if(!loadDarkImageFile(m_pedestal_file_names[nb_darks], m_pedestal_images, m_width, m_height))
            {
                break;
            }
        }

        // no problem occured
        if(nb_darks == m_pedestal_file_names.size())
        {
            m_dark_images_loaded = true;
        }
        else
        // a problem occured, we flush the dark images which could have been loaded
        {
            m_pedestal_images.clear();
        }
    }

    // build the intensity coeffs buffer
    updateIntensityCoeffs();
}

/************************************************************************
 * \brief cleans the shared memory used by the camera
 ************************************************************************/
void Camera::cleanSharedMemory()
{
    DEB_MEMBER_FUNCT();

    std::string cmd = "rm /dev/shm/slsDetectorPackage*;";
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

/************************************************************************
 * \brief creates an autolock mutex for calibration data access
 ************************************************************************/
lima::AutoMutex Camera::calibrationLock() const
{
    DEB_MEMBER_FUNCT();
    return lima::AutoMutex(m_calibration_cond.mutex());
}

//==================================================================
// Related to HwInterface
//==================================================================
//------------------------------------------------------------------
// calibration management
//------------------------------------------------------------------
/*******************************************************************
 * \brief starts the calibration
 *******************************************************************/
void Camera::startCalibration()
{
    DEB_MEMBER_FUNCT();

    stopCalibration();

    m_dark_thread->sendCmd(CameraDarkThread::StartCalibration);
    m_dark_thread->waitNotStatus(CameraDarkThread::Idle);
}

/*******************************************************************
 * \brief stops the calibration
 *******************************************************************/
void Camera::stopCalibration()
{
    DEB_MEMBER_FUNCT();

    // stopping the thread and restarting the thread in case of error
    applyStopCalibration(true, false);
}

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
    m_detector_control->resetFramesCaughtInReceiver();

    // clear the frames containers
    m_frames_manager.clear();
}

/*******************************************************************
 * \brief starts the acquisition (start/snap)
 *******************************************************************/
void Camera::startAcq()
{
    stopAcq();

    m_thread->sendCmd(CameraThread::StartAcq);
    m_thread->waitNotStatus(CameraThread::Idle);
}

/*******************************************************************
 * \brief stops the acquisition
 *******************************************************************/
void Camera::stopAcq()
{
    DEB_MEMBER_FUNCT();

    // stopping the thread and restarting the thread in case of error
    applyStopAcq(true, false);
}

/*******************************************************************
 * \brief stops the acquisition and abort or restart the acq thread 
 *        if it is in error. Can also abort the thread when we exit
 *        the program.
 *******************************************************************/
void Camera::applyStopAcq(bool in_restart, bool in_always_abort)
{
    DEB_MEMBER_FUNCT();

	DEB_TRACE() << "executing StopAcq command...";

    m_thread->execStopAcq();

    // thread in error
    if(m_thread->getStatus() == CameraThread::Error)
    {
        // aborting the thread
        m_thread->abort();

        if(in_restart)
        {
            // releasing the camera thread
            delete m_thread;

            // creating the camera thread
            m_thread = new CameraThread(*this);

            // starting the acquisition thread
            m_thread->start();
        }
    }
    else
    // we are going to exit the program, so we are forcing an abort
    if(in_always_abort)
    {
        // aborting the thread
        m_thread->abort();
    }
}

/*******************************************************************
 * \brief stops the calibration and abort or restart the thread 
 *        if it is in error. Can also abort the thread when we exit
 *        the program.
 *******************************************************************/
void Camera::applyStopCalibration(bool in_restart, bool in_always_abort)
{
    DEB_MEMBER_FUNCT();

	DEB_TRACE() << "executing StopCalibration command...";

    m_dark_thread->execStopCalibration();

    // thread in error
    if(m_dark_thread->getStatus() == CameraThread::Error)
    {
        // aborting the thread
        m_dark_thread->abort();

        if(in_restart)
        {
            // releasing the camera thread
            delete m_dark_thread;

            // creating the camera thread
            m_dark_thread = new CameraDarkThread(*this);

            // starting the thread
            m_dark_thread->start();
        }
    }
    else
    // we are going to exit the program, so we are forcing an abort
    if(in_always_abort)
    {
        // aborting the thread
        m_dark_thread->abort();
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
    if(!m_calibration_running)
    {
        // 24 bits intensity images
        if(areGainCoeffsLoaded())
        {
            m_frames_manager.manageFirstFrameTreatment(in_frame_index, in_timestamp);

            uint64_t relative_frame_index = m_frames_manager.computeRelativeFrameIndex(in_frame_index);

            // ckecking if there is no packet lost.
            if(in_packet_number == m_frame_packet_number)
            {
                uint64_t relative_timestamp = m_frames_manager.computeRelativeTimestamp(in_timestamp);

                // giving the frame to the frames manager
                CameraFrame frame(relative_frame_index, in_packet_number, relative_timestamp);

                // the image will be copied during the call of this method
                m_frames_manager.addReceived(in_receiver_index, frame, in_data_pointer, in_data_size);
            }
            else
            // rejected frame
            {
                DEB_TRACE() << "Rejected Frame [ " << relative_frame_index << ", " << in_packet_number << " ]";
            }
        }
        // 16 bits raw images
        else
        {
            StdBufferCbMgr & buffer_mgr  = m_buffer_ctrl_obj.getBuffer();
            lima::FrameDim   frame_dim   = buffer_mgr.getFrameDim();

            // checking the frame size 
            int mem_size = frame_dim.getMemSize();

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
                    uint64_t relative_timestamp = m_frames_manager.computeRelativeTimestamp (in_timestamp);

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
    }
    else
    // calibration is running)
    {
        m_frames_manager.manageFirstFrameTreatment(in_frame_index, in_timestamp);

        uint64_t relative_frame_index = m_frames_manager.computeRelativeFrameIndex(in_frame_index);

        // ckecking if there is no packet lost.
        if(in_packet_number == m_frame_packet_number)
        {
            // giving the frame to the frames manager
            CameraFrame frame(relative_frame_index, in_packet_number, 0); // does not need the timestamp

            // the image will be copied during the call of this method
            m_frames_manager.addReceived(in_receiver_index, frame, in_data_pointer, in_data_size);
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
    DEB_MEMBER_FUNCT();

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
    DEB_MEMBER_FUNCT();

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
    DEB_MEMBER_FUNCT();

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
    DEB_MEMBER_FUNCT();

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

    int thread_status      = m_thread->getStatus();
    int dark_thread_status = m_dark_thread->getStatus();

    // error during the acquisition management or calibration ?
    // the device becomes in error state.
    if((thread_status == CameraThread::Error)||(dark_thread_status == CameraDarkThread::Error))
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
    // the device is in calibration.
    if(dark_thread_status == CameraDarkThread::Running)
    {
        result = Camera::Calibration;
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

/*******************************************************************
 * \brief Gets the detector image size
 * \param size returned detector image size
 *******************************************************************/
void Camera::getDetectorImageSize(Size& size)
{
	DEB_MEMBER_FUNCT();
    size = Size(getWidth(), getHeight());
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

    // in the case of a gain coeffs file was loaded, the final image will be in 24 bits
    if(areGainCoeffsLoaded())
    {
        bit_depth = 24;
    }

    switch(bit_depth)
    {
        case 16: 
            type = lima::ImageType::Bpp16;
            break;

        case 24: 
            type = lima::ImageType::Bpp24;
            break;

        default:
            THROW_HW_ERROR(ErrorType::Error) << "This pixel format of the camera is not managed, only 16 bits cameras are managed!";
            break;
    }

    return type;
}

/*******************************************************************
 * \brief gets the current image type
 * \param type new image type
*******************************************************************/
void Camera::getImageType(ImageType& type) const
{
	DEB_MEMBER_FUNCT();

    type = getImageType();
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
        case lima::ImageType::Bpp24: 
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

        // converting trigger mode index to trigger mode label
        m_trig_mode_label = slsDetectorUsers::getTimingMode(trigger_mode_index);

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
 * \param in_module_index module index
 * \return the command result
 *******************************************************************/
std::string Camera::setCmd(const std::string & in_command, int in_module_index)
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

        result = m_detector_control->putCommand(argc, argv, in_module_index);
    }

    releaseArgs(argv, argc);

	DEB_RETURN() << "result=\"" << result << "\"";
    return result;
}

/*******************************************************************
 * \brief Executes a get command
 * \param in_command command in command line format
 * \param in_module_index module index
 * \return the command result
 *******************************************************************/
std::string Camera::getCmd(const std::string & in_command, int in_module_index)
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

        result = m_detector_control->getCommand(argc, argv, in_module_index);
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

//------------------------------------------------------------------
// gain mode management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets the gain mode
 * \return gain mode
 *******************************************************************/
lima::SlsJungfrau::Camera::GainMode Camera::getGainMode(void)
{
    DEB_MEMBER_FUNCT();

    // during acquisition, camera data access is not allowed because it
    // could put the camera into an error state. 
    // So we give the latest read value.
    if(m_status == Camera::Idle)
    {
        slsDetectorDefs::detectorSettings gain_mode;

        // protecting the sdk concurrent access
        {
            lima::AutoMutex sdk_mutex = sdkLock(); 

            // getting the current gain mode index
            gain_mode = static_cast<slsDetectorDefs::detectorSettings>(m_detector_control->setSettings(SLS_GET_VALUE));
        }

        // computing the gain mode
        if((gain_mode == slsDetectorDefs::detectorSettings::UNDEFINED    ) ||
           (gain_mode == slsDetectorDefs::detectorSettings::UNINITIALIZED))
        {
            m_gain_mode = Camera::undefined;
        }
        else
        if(gain_mode == slsDetectorDefs::detectorSettings::DYNAMICGAIN)
        {
            m_gain_mode = Camera::dynamic;
        }
        else
        if(gain_mode == slsDetectorDefs::detectorSettings::DYNAMICHG0)
        {
            m_gain_mode = Camera::dynamichg0;
        }
        else
        if(gain_mode == slsDetectorDefs::detectorSettings::FIXGAIN1)
        {
            m_gain_mode = Camera::fixgain1;
        }
        else
        if(gain_mode == slsDetectorDefs::detectorSettings::FIXGAIN2)
        {
            m_gain_mode = Camera::fixgain2;
        }
        else
        if(gain_mode == slsDetectorDefs::detectorSettings::FORCESWITCHG1)
        {
            m_gain_mode = Camera::forceswitchg1;
        }
        else
        if(gain_mode == slsDetectorDefs::detectorSettings::FORCESWITCHG2)
        {
            m_gain_mode = Camera::forceswitchg2;
        }
        else
        {
            THROW_HW_ERROR(ErrorType::Error) << "getGainMode : This camera gain mode is not managed: (" << gain_mode << ")";
        }
    }

    return m_gain_mode;
}

/*******************************************************************
 * \brief Sets the gain mode
 * \param in_gain_mode needed gain mode 
 *******************************************************************/
void Camera::setGainMode(lima::SlsJungfrau::Camera::GainMode in_gain_mode)
{
    DEB_MEMBER_FUNCT();

    // converting the detector gain mode to the sdk gain mode
    slsDetectorDefs::detectorSettings gain_mode;

    switch (in_gain_mode)
    {       
        case Camera::dynamic:
            gain_mode = slsDetectorDefs::detectorSettings::DYNAMICGAIN;
            break;

        case Camera::dynamichg0:
            gain_mode = slsDetectorDefs::detectorSettings::DYNAMICHG0;
            break;

        case Camera::fixgain1:
            gain_mode = slsDetectorDefs::detectorSettings::FIXGAIN1;
            break;

        case Camera::fixgain2:
            gain_mode = slsDetectorDefs::detectorSettings::FIXGAIN2;
            break;

        case Camera::forceswitchg1:
            gain_mode = slsDetectorDefs::detectorSettings::FORCESWITCHG1;
            break;

        case Camera::forceswitchg2:
            gain_mode = slsDetectorDefs::detectorSettings::FORCESWITCHG2;
            break;

        default:
            THROW_HW_ERROR(ErrorType::Error) << "setGainMode : Cannot change the Trigger mode of the camera, this mode is not managed: (" << in_gain_mode << ")";
            break;
    }

    // protecting the sdk concurrent access
    {
        lima::AutoMutex sdk_mutex = sdkLock(); 

        // setting the current gain mode index and loading the settings to the detector
        DEB_TRACE() << "setGainMode  " << gain_mode;

        int result = m_detector_control->setSettings(static_cast<int>(gain_mode));

        if(result == slsDetectorDefs::FAIL)
        {
            THROW_HW_ERROR(ErrorType::Error) << "setGainMode failed!";
        }
    }

    // updating the internal data
    getGainMode();
}

/*******************************************************************
 * \brief Loads the gains'coefficients file
 * \param in_file_name complete file name to load
 * \param out_gains_coeffs container filled with the data
 * \param in_width width of the detector image 
 * \param in_height height of the detector image
 * \return true if success, false if error
 *******************************************************************/
bool Camera::loadGainsCoeffsFile(const std::string & in_file_name,
                                 std::vector<std::vector<double>> & out_gains_coeffs,
                                 unsigned short in_width ,
                                 unsigned short in_height)
{
    DEB_MEMBER_FUNCT();

    double *    buffer = NULL;
    bool        result = true;
    std::size_t size   = 0LL ;

    DEB_TRACE() << "opening Gains coefficients file " << in_file_name << "...";

    std::ifstream file( in_file_name.c_str(), std::ios::in | std::ios::binary | std::ios::ate);

    //File does not exist
    if(file.fail())
    {
        result = false;
    }
    else
    {
        // copies all data into buffer
        try
        {
            size = file.tellg();
            file.seekg (0, std::ios::beg);
            
            buffer = new double [size / sizeof(double)];
            file.read (reinterpret_cast<char *>(buffer), size);
            file.close();

            // compute the number of gains
            // the file content is : G0, G1, G2,HG0
            // we do not need HG0
            int gain_size = in_width * in_height; 

            // check the file size
            if((size / sizeof(double)) == (gain_size * 4))
            {
                int nb_gains  = (size / (gain_size * sizeof(double))) - 1;

                out_gains_coeffs.resize(nb_gains);

                // allocate the vector
                for(int gains_index = 0 ; gains_index < nb_gains ; gains_index++)
                {
                    std::vector<double> & gains_coeffs = out_gains_coeffs[gains_index];
                    gains_coeffs.resize(gain_size);

                    memcpy(&gains_coeffs[0], buffer + (gain_size * gains_index), gain_size * sizeof(double));
                }
            }
            else
            {
                DEB_TRACE() << "Size is incoherent!";
                DEB_TRACE() << "Found " << (size / sizeof(double)) << " elements.";
                DEB_TRACE() << "Should be " << (gain_size * 4) << " elements.";
                result = false;
            }
        }
        catch(...)
        {
            result = false;
        }
    }

    if(buffer != NULL)
    {
        delete [] buffer;
    }

    if(result)
    {
        DEB_TRACE() << "File loaded : " << size << " bytes.";
    }
    else
    {
        DEB_TRACE() << "error occured during opening.";
    }

    return result;
}

/*******************************************************************
 * \brief tells if the gain coefficients were loaded
 * \return true if the gain coefficients were loaded, else false
 *******************************************************************/
bool Camera::areGainCoeffsLoaded(void) const
{
    // protecting the concurrent access
    lima::AutoMutex sdk_mutex = calibrationLock(); 

    return !m_gains_coeffs.empty();
}

/*******************************************************************
 * \brief Gets the gain coefficients state
 * \return the state (loaded or not loaded)
 *******************************************************************/
std::string Camera::getGainCoeffsState(void)
{
    return (areGainCoeffsLoaded() ? SLS_GAIN_COEFFS_STATE_LOADED : SLS_GAIN_COEFFS_STATE_NOT_LOADED);
}

/*******************************************************************
 * \brief Gets the coefficients for a gain type
 * \param in_gain_index index of the gain (starts at 0)
 * \param out_gain_coeffs buffer which will be filled with the values
 * \return none
 *******************************************************************/
void Camera::getGainCoeffsState(int in_gain_index, double * out_gain_coeffs)
{
    // protecting the concurrent access
    lima::AutoMutex sdk_mutex = calibrationLock(); 

    if(in_gain_index < m_gains_coeffs.size())
    {
        memcpy(out_gain_coeffs, m_gains_coeffs[in_gain_index].data(), m_gains_coeffs[in_gain_index].size() * sizeof(double));
    }
    else
    {
        memset(out_gain_coeffs, 0, m_width * m_height * sizeof(double));
    }
}

//==================================================================
// Related to the calibration process
//==================================================================
/*******************************************************************
 * \brief set the exposure time and period used for the calibration
 * \param[in] in_pedestal_exposures_sec pedestal exposure time in seconds
 * \param[in] in_pedestal_periods_sec pedestal period in seconds
 *******************************************************************/
void Camera::setCalibrationExposureTimeAndPeriod(double in_pedestal_exposures_sec, double in_pedestal_periods_sec)
{
    DEB_MEMBER_FUNCT();

    // protecting the concurrent access
    lima::AutoMutex sdk_mutex = calibrationLock(); 

    m_pedestal_exposures_sec.clear();
    m_pedestal_periods_sec.clear  ();

    for(std::size_t nb_darks = 0 ; nb_darks < m_pedestal_file_names.size() ; nb_darks++)
    {
        m_pedestal_exposures_sec.push_back(in_pedestal_exposures_sec);
        m_pedestal_periods_sec.push_back  (in_pedestal_periods_sec  );
    }
}

/*******************************************************************
 * \brief Gets the calibration state
 * \return the state
 *******************************************************************/
std::string Camera::getCalibrationState(void)
{
    std::string result = SLS_CALIBRATION_STATE_NONE;

    // protecting the concurrent access
    {
        lima::AutoMutex sdk_mutex = calibrationLock(); 

        if(m_dark_images_loaded)
        {
            result = SLS_CALIBRATION_STATE_LOADED;
        }
        else
        {
            Camera::Status status = getStatus();

            if(status == Camera::Calibration)
            {
                std::size_t nb_dark_images = m_pedestal_images.size();

                if(nb_dark_images == SLS_NUMBER_OF_DARK_IMAGES)
                    result = SLS_CALIBRATION_STATE_GENERATED;
                else
                if(nb_dark_images == 0)
                    result = SLS_CALIBRATION_STATE_RUNNING_0_3;
                else
                if(nb_dark_images == 1)
                    result = SLS_CALIBRATION_STATE_RUNNING_1_3;
                else
                if(nb_dark_images == 2)
                    result = SLS_CALIBRATION_STATE_RUNNING_2_3;
            }
        }
    }

    return result;
}

/*******************************************************************
 * \brief Gets a dark image
 * \param in_gain_index index of the gain (starts at 0)
 * \param out_dark_image dark image which will be filled
 * \return none
 *******************************************************************/
void Camera::getDarkImage(int in_gain_index, uint16_t * out_dark_image)
{
    // protecting the concurrent access
    lima::AutoMutex sdk_mutex = calibrationLock(); 

    if(in_gain_index < m_pedestal_images.size())
    {
        memcpy(out_dark_image, m_pedestal_images[in_gain_index].data(), m_pedestal_images[in_gain_index].size() * sizeof(uint16_t));
    }
    else
    {
        memset(out_dark_image, 0, m_width * m_height * sizeof(uint16_t));
    }
}

/*******************************************************************
 * \brief Loads a dark image from a file
 * \param in_file_name complete file name to load
 * \param out_pedestal_images container filled with the data
 * \param in_width width of the detector image 
 * \param in_height height of the detector image
 * \return true if success, false if error
 *******************************************************************/
bool Camera::loadDarkImageFile(const std::string & in_file_name,
                               std::vector<std::vector<uint16_t>> & out_pedestal_images,
                               unsigned short in_width ,
                               unsigned short in_height)
{
    DEB_MEMBER_FUNCT();

    uint16_t *  buffer = NULL;
    bool        result = true;
    std::size_t size   = 0LL ;

    DEB_TRACE() << "opening dark image file " << in_file_name << "...";

    std::ifstream file( in_file_name.c_str(), std::ios::in | std::ios::binary | std::ios::ate);

    //File does not exist
    if(file.fail())
    {
        result = false;
    }
    else
    {
        // copies all data into buffer
        try
        {
            size = file.tellg();
            file.seekg (0, std::ios::beg);
            
            buffer = new uint16_t [size / sizeof(uint16_t)];
            file.read (reinterpret_cast<char *>(buffer), size);
            file.close();

            int pixels_nb = in_width * in_height; 

            // check the file size
            if((size / sizeof(uint16_t)) == pixels_nb)
            {
                out_pedestal_images.resize(out_pedestal_images.size() + 1);
                std::vector<uint16_t> & pedestal_image = out_pedestal_images.back();
                pedestal_image.resize(pixels_nb);

                memcpy(pedestal_image.data(), buffer, pixels_nb * sizeof(uint16_t));
            }
            else
            {
                DEB_TRACE() << "Size is incoherent!";
                DEB_TRACE() << "Found " << (size / sizeof(uint16_t)) << " elements.";
                DEB_TRACE() << "Should be " << pixels_nb << " elements.";
                result = false;
            }
        }
        catch(...)
        {
            result = false;
        }
    }

    if(buffer != NULL)
    {
        delete [] buffer;
    }

    if(result)
    {
        DEB_TRACE() << "File loaded : " << size << " bytes.";
    }
    else
    {
        DEB_TRACE() << "error occured during opening.";
    }

    return result;
}

/*******************************************************************
 * \brief Erases a dark image file
 * \param in_file_name complete file name to erase
 * \return true if success, false if error
 *******************************************************************/
bool Camera::eraseDarkImageFile(const std::string & in_file_name) const
{
    DEB_MEMBER_FUNCT();

    // delete the dark image
    bool file_exist;

    {
        std::ifstream file(in_file_name);
        file_exist = file;
    }

    if(file_exist)
    {
        if (remove(in_file_name.c_str()) != 0)
        {
            DEB_TRACE() << "remove of the previous dark image file " << in_file_name << " failed!";
            return false;
        }
    }

    return true;
}

/*******************************************************************
 * \brief Saves a dark image from a file
 * \param in_file_name complete file name to save
 * \param in_pedestal_image container which contains the dark image
 * \return true if success, false if error
 *******************************************************************/
bool Camera::saveDarkImageFile(const std::string & in_file_name,
                               const std::vector<uint16_t> & in_pedestal_image) const
{
    DEB_MEMBER_FUNCT();

    // first, delete the previous dark image
    if(!eraseDarkImageFile(in_file_name))
    {
        return false;
    }

    bool result = true;

    DEB_TRACE() << "opening dark image file " << in_file_name << "...";

    std::ofstream file( in_file_name.c_str(), std::ios::out | std::ios::binary);

    //File has a problem
    if(!file.is_open())
    {
        result = false;
    }
    else
    {
        // copies all data into buffer
        try
        {
            file.write(reinterpret_cast<const char *>(in_pedestal_image.data()), in_pedestal_image.size() * sizeof(uint16_t));
            file.close();
        }
        catch(...)
        {
            result = false;
        }
    }

    if(result)
    {
        DEB_TRACE() << "File saved : " << in_pedestal_image.size() * sizeof(uint16_t) << " bytes.";
    }
    else
    {
        DEB_TRACE() << "error occured during saving.";
    }

    return result;
}

/*******************************************************************
 * \brief reset the previous dark images in memory and erase the files
 *******************************************************************/
void Camera::resetDarkImageFile()
{
    DEB_MEMBER_FUNCT();

    // protecting the concurrent access
    lima::AutoMutex sdk_mutex = calibrationLock(); 

    m_dark_images_loaded = false;
    m_pedestal_images.clear();

    for(std::size_t nb_darks = 0 ; nb_darks < m_pedestal_file_names.size() ; nb_darks++)
    {
        eraseDarkImageFile(m_pedestal_file_names[nb_darks]);
    }
}

/*******************************************************************
 * \brief update the dark images memory data
 *******************************************************************/
void Camera::updateDarkImagesData()
{
    DEB_MEMBER_FUNCT();
    
    // protecting the concurrent access
    lima::AutoMutex sdk_mutex = calibrationLock(); 

    std::size_t nb_dark_images = m_pedestal_images.size();

    // check if the number of images in memory is correct
    if(nb_dark_images == m_pedestal_file_names.size())
    {
        m_dark_images_loaded = true;
    }
    else
    // a problem occured, we flush the dark images which could have been loaded or created in memory
    {
        m_pedestal_images.clear();
    }
}

//==================================================================
// Related to the computing of the intensity coeffs buffer
//==================================================================
/*******************************************************************
 * \brief Updates the intensity coeffs buffer (init or dark images change)
 * \return true if success, false if error
 *******************************************************************/
bool Camera::updateIntensityCoeffs(void)
{
    DEB_MEMBER_FUNCT();

    // checking the gain coeffs buffer
    if(m_gains_coeffs.size() < SLS_NUMBER_OF_DARK_IMAGES)
    {
        DEB_TRACE() << "error occured during update of intensity coeffs : invalid gain coeffs!";
        return false;
    }

    // protecting the concurrent access
    lima::AutoMutex sdk_mutex = calibrationLock(); 

    // checking the dark images
    {
        if(m_pedestal_images.size() < SLS_NUMBER_OF_DARK_IMAGES)
        {
            DEB_TRACE() << "error occured during update of intensity coeffs : invalid dark images!";
            return false;
        }
    }
    
    // the intensity coeffs buffer will contain dark image and gain coeffs data :
    // [Pixel0][dark0|gain0|dark1|gain1|empty|empty|dark2|gain2]...[Pixeli][dark0|gain0|dark1|gain1|dark2|gain2]
    // in the pixel of the detector, the value for the gain 0 is 0 
    // in the pixel of the detector, the value for the gain 1 is 1 
    // in the pixel of the detector, the value for the gain 2 is 3!
    const int pixels_nb         = m_width * m_height;
    const int data_nb_per_pixel = (SLS_NUMBER_OF_DARK_IMAGES + 1) * 2; // +1 to add empty space between gain 1 and gain 2

    m_intensity_coeffs.clear();
    m_intensity_coeffs.resize(pixels_nb * data_nb_per_pixel);
    memset(m_intensity_coeffs.data(), 0, sizeof(double) * m_intensity_coeffs.size());

    const int gain_position[SLS_NUMBER_OF_DARK_IMAGES] = {0, 2, 6};

    for(int gain_index = 0 ; gain_index < m_gains_coeffs.size() ; gain_index++)
    {
        const std::vector<double>   & gain_coeffs    = m_gains_coeffs   [gain_index];
        const std::vector<uint16_t> & pedestal_image = m_pedestal_images[gain_index]; 

        for(int pixel_index = 0 ; pixel_index < pixels_nb ; pixel_index++)
        {
            const int pos = (pixel_index * data_nb_per_pixel) + gain_position[gain_index];
            
            // first of all, store the dark image pixel
            m_intensity_coeffs[pos] = static_cast<double>(pedestal_image[pixel_index]);

            // secondly, store the gain coefficient
            m_intensity_coeffs[pos + 1] = gain_coeffs[pixel_index];
        }
    }
}

/*******************************************************************
 * \brief Checks if the intensity coeffs buffer is valid
 * \return true if the buffer is valid, false if invalid
 *******************************************************************/
bool Camera::areIntensityCoeffsValid(void) const
{
    // protecting the concurrent access
    lima::AutoMutex sdk_mutex = calibrationLock(); 

    return !m_intensity_coeffs.empty();
}

/*******************************************************************
 * \brief reset the intensity coeffs
 *******************************************************************/
void Camera::resetIntensityCoeffs(void)
{
    DEB_MEMBER_FUNCT();
    
    // protecting the concurrent access
    lima::AutoMutex sdk_mutex = calibrationLock(); 

    m_intensity_coeffs.clear();
}

//==================================================================
// Related to acquisition configuration backup and restoration
//==================================================================
/*******************************************************************
 * \brief Gets the acquisition configuration
 * \param out_configuration this structure will contain the configuration
 * \return true if the method was successful, else false
 *******************************************************************/
bool Camera::getAcqConf(Camera::CameraAcqConf & out_configuration) const
{
    DEB_MEMBER_FUNCT();

    // getting the current trigger mode index
    out_configuration.m_trigger_mode = m_detector_control->setTimingMode(SLS_GET_VALUE);

    // reading the number of frames per cycle
    out_configuration.m_nb_frames_per_cycle = m_detector_control->setNumberOfFrames(SLS_GET_VALUE);

    // reading the number of cycles
    out_configuration.m_nb_cycles = m_detector_control->setNumberOfCycles(SLS_GET_VALUE);

    // reading the exposure time in seconds
    out_configuration.m_exposures_sec = m_detector_control->setExposureTime  (SLS_GET_VALUE, true);

    // reading the exposure period in seconds
    out_configuration.m_periods_sec = m_detector_control->setExposurePeriod(SLS_GET_VALUE, true); // in seconds

    // getting the current gain mode index
    out_configuration.m_gain_mode = static_cast<int>(m_detector_control->setSettings(SLS_GET_VALUE));

    return true;
}

/*******************************************************************
 * \brief Gets the acquisition configuration
 * \param in_configuration this structure contains the configuration
 * \return true if the method was successful, else false
 *******************************************************************/
bool Camera::setAcqConf(const Camera::CameraAcqConf in_configuration)
{
    DEB_MEMBER_FUNCT();

    // initing the number of frames per cycle and  number of cycles 
    // to avoid problems during the trigger mode change.
    m_detector_control->setNumberOfFrames(1);
    m_detector_control->setNumberOfCycles(1);

    // apply the trigger change
    m_detector_control->setTimingMode(in_configuration.m_trigger_mode);

    // setting the number of cycles
    m_detector_control->setNumberOfCycles(in_configuration.m_nb_cycles);

    // setting the number of frames per cycle
    m_detector_control->setNumberOfFrames(in_configuration.m_nb_frames_per_cycle);

    // setting the exposure time in seconds
    m_detector_control->setExposureTime  (in_configuration.m_exposures_sec, true);

    // setting the exposure period in seconds
    m_detector_control->setExposurePeriod(in_configuration.m_periods_sec  , true);

    int result = m_detector_control->setSettings(static_cast<int>(in_configuration.m_gain_mode));

    if(result == slsDetectorDefs::FAIL)
    {
        return false;
    }

    return true;
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
