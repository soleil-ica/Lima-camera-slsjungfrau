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
 * @param in_config_file_name complete path to the configuration file
 ************************************************************************/
Camera::Camera(): m_thread(*this)
{
    DEB_CONSTRUCTOR();
}

/************************************************************************
 * \brief destructor
 ************************************************************************/
Camera::~Camera()
{
    DEB_DESTRUCTOR();
}

/************************************************************************
 * \brief inits the camera while setting the configuration file name
 * @param in_config_file_name complete path to the configuration file
 ************************************************************************/
void Camera::init(const std::string & in_config_file_name)
{
    DEB_MEMBER_FUNCT();

    // before, cleaning the shared memory
    cleanSharedMemory();

    // initing the class attributes
    m_config_file_name = in_config_file_name;

    // creating the detector control instance
    int id = 0;

    m_detector_control.reset(new slsDetectorUsers(id));

    // configuration file is used to properly configure advanced settings in the shared memory
    int result = m_detector_control->readConfigurationFile(m_config_file_name);

    if(result == CallResult::FAIL)
    {
        THROW_HW_ERROR(ErrorType::Error) << "readConfigurationFile failed! Could not initialize the camera!";
    }

    // Setting the detector online
    m_detector_control->setOnline(slsDetectorDefs::ONLINE_FLAG);

    // starting the acquisition thread
    m_thread.start();

    std::cout << "Module   Firmware Version : " << getModuleFirmwareVersion  () << std::endl;
    std::cout << "Detector Firmware Version : " << getDetectorFirmwareVersion() << std::endl;
    std::cout << "Detector Software Version : " << getDetectorSoftwareVersion() << std::endl;
}

/************************************************************************
 * \brief cleans the shared memory used by the camera
 ************************************************************************/
void Camera::cleanSharedMemory()
{
	DEB_MEMBER_FUNCT();
	string cmd = "for i in seq `ipcs -m | cut -d ' ' -f1`; do ipcrm -M $i; done;";
    std::system(cmd.c_str());
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
}

/*******************************************************************
 * \brief starts the acquisition (start/snap)
 *******************************************************************/
void Camera::startAcq()
{
    DEB_MEMBER_FUNCT();
}

/*******************************************************************
 * \brief stops the acquisition
 *******************************************************************/
void Camera::stopAcq()
{
    DEB_MEMBER_FUNCT();
}

//------------------------------------------------------------------
// status management
//------------------------------------------------------------------
/************************************************************************
 * \brief returns the current camera status
 * \return current hardware status
 ************************************************************************/
Camera::Status Camera::getStatus() const
{
    DEB_MEMBER_FUNCT();

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

    Camera::Status result;

    // getting the detector status
    int status = m_detector_control->getDetectorStatus();

    status = slsDetectorDefs::runStatus::IDLE;

    // ready to start acquisition
    if(status == slsDetectorDefs::runStatus::IDLE)
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
    // acquisition stopped externally, fifo full or unexpected error 
    if((status == slsDetectorDefs::runStatus::ERROR  ) ||
       (status == slsDetectorDefs::runStatus::STOPPED))
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

    return result;

/*    int thread_status = m_thread.getStatus();

    DEB_RETURN() << DEB_VAR1(thread_status);

    switch(thread_status)
    {
        case CameraThread::Ready:
            return Camera::Ready;
        case CameraThread::Exposure:
            return Camera::Exposure;
        case CameraThread::Readout:
            return Camera::Readout;
        case CameraThread::Latency:
            return Camera::Latency;
        default:
            throw LIMA_HW_EXC(Error, "Invalid thread status");
    }*/
}

//------------------------------------------------------------------
// Acquired frames management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets the number of acquired frames
 * \return current acquired frames number
 *******************************************************************/
int Camera::getNbHwAcquiredFrames() const
{
    DEB_MEMBER_FUNCT();
    return 0;
//    return (m_acq_frame_nb == -1) ? 0 : (m_acq_frame_nb + 1);
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
/*    DEB_RETURN() << DEB_VAR1(m_max_width);
    return m_max_width;*/
    return 0;
}

/*******************************************************************
 * \brief Gets the maximum image height
 * \return maximum image height
 *******************************************************************/
unsigned short Camera::getMaxHeight() const
{
    DEB_MEMBER_FUNCT();
//    DEB_RETURN() << DEB_VAR1(m_max_height);
//    return m_max_height;
    return 0;
}


//------------------------------------------------------------------
// current image type management
//------------------------------------------------------------------
/*******************************************************************
 * \brief gets the current image type
 * \return current image type
*******************************************************************/
ImageType Camera::getImageType() const
{
    DEB_MEMBER_FUNCT();

/*    switch(m_depth)
    {
        case 16: type = Bpp16;
            break;
        
        default:
            THROW_HW_ERROR(Error) << "This pixel format of the camera is not managed, only 16 bits cameras are already managed!";
            break;
    }*/
}

/*******************************************************************
 * \brief sets the current image type
 * @param in_type new image type
 *******************************************************************/
void Camera::setImageType(ImageType in_type)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setImageType - " << DEB_VAR1(in_type);

/*    switch(type)
    {
        case Bpp16:
            m_depth = 16;
            break;
        
        default:
            THROW_HW_ERROR(Error) << "This pixel format of the camera is not managed, only 16 bits cameras are already managed!";
            break;
    }*/
}

//------------------------------------------------------------------
// pixel size management
//------------------------------------------------------------------
/*******************************************************************
 * \brief gets the pixel size
 * @param out_x_size width pixel size
 * @param out_y_size height pixel size
 *******************************************************************/
void Camera::getPixelSize(double & out_x_size, double & out_y_size) const
{
	DEB_MEMBER_FUNCT();

// CCA:TODO
//		x_size = y_size = std::numeric_limits<double>::quiet_NaN();
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
    std::string type = "SlsJungfrau";
    DEB_RETURN() << type;
    return type;
}

/*******************************************************************
 * \brief gets the detector model
 * \return detector model
 *******************************************************************/
std::string Camera::getDetectorModel() const
{
    DEB_MEMBER_FUNCT();
    std::string model = "To be defined";
    DEB_RETURN() << model;
    return model;
}

/*******************************************************************
 * \brief converts a version id to a string
 * \return version in string format
 *******************************************************************/
std::string Camera::convertVersionToString(int64_t in_version)
{
    std::stringstream tempStream;
    tempStream << in_version;
    return tempStream.str();
}

/*******************************************************************
 * \brief gets Module Firmware Version
 * \return Module Firmware Version
 *******************************************************************/
std::string Camera::getModuleFirmwareVersion() const
{
    DEB_MEMBER_FUNCT();
    return convertVersionToString(m_detector_control->getModuleFirmwareVersion());
}

/*******************************************************************
 * \brief gets Detector Firmware Version
 * \return Detector Firmware Version
 *******************************************************************/
std::string Camera::getDetectorFirmwareVersion() const
{
    DEB_MEMBER_FUNCT();
    return convertVersionToString(m_detector_control->getDetectorFirmwareVersion());
}

/*******************************************************************
 * \brief gets Detector Software Version
 * \return Detector Software Version
 *******************************************************************/
std::string Camera::getDetectorSoftwareVersion() const
{
    DEB_MEMBER_FUNCT();
    return convertVersionToString(m_detector_control->getDetectorSoftwareVersion());
}

//==================================================================
// Related to HwSyncCtrlObj
//==================================================================
//------------------------------------------------------------------
// trigger mode management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Checks the trigger mode validity
 * \return true if passed trigger mode is supported
 *******************************************************************/
bool Camera::checkTrigMode(TrigMode in_trig_mode) const
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(in_trig_mode);
    bool valid_mode;    

    switch (in_trig_mode)
    {       
        case IntTrig:
        case IntTrigMult:
        case ExtTrigSingle:
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
 * @param in_mode needed trigger mode 
 *                (IntTrig, ExtTrigSingle, ExtTrigMultiple)
 *******************************************************************/
void Camera::setTrigMode(TrigMode in_mode)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setTrigMode - " << DEB_VAR1(in_mode);

/*    switch(mode)
    {
        case IntTrig:
            m_trigger_mode = 0x1; // 0x1 (int. trigger)
            break;

        case ExtTrigSingle:
            m_trigger_mode = 0x2; // 0x2  (ext. trigger)                
            break;

        default:
            THROW_HW_ERROR(Error) << "Cannot change the Trigger Mode of the camera, this mode is not managed !";
            break;
    }*/
}

/*******************************************************************
 * \brief Gets the trigger mode
 * \return trigger mode
 *******************************************************************/
TrigMode Camera::getTrigMode() const
{
    DEB_MEMBER_FUNCT();

    TrigMode mode = IntTrig;

/*    switch(m_trigger_mode)
    {
        case 0x1: //(int. trigger)
            mode = IntTrig;
            break;

        case 0x2: //(ext. trigger)         
            mode = ExtTrigSingle;
            break;
    }*/
    return IntTrig;
}

//------------------------------------------------------------------
// exposure time management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets the exposure time
 * \return exposure time
 *******************************************************************/
double Camera::getExpTime() const
{
    DEB_MEMBER_FUNCT();
    //exp_time = m_exposure_time;
    return 0.0;
}

/*******************************************************************
 * \brief Sets the exposure time
 * @param in_exp_time needed exposure time
*******************************************************************/
void Camera::setExpTime(double in_exp_time)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setExpTime - " << DEB_VAR1(in_exp_time) << " (s)";

//    m_exposure_time = exp_time;
}

//------------------------------------------------------------------
// latency time management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets the latency time
 * \return latency time
 *******************************************************************/
double Camera::getLatencyTime() const
{
    DEB_MEMBER_FUNCT();
    //latency_time = m_latency_time;
    return 0.0;
}

/*******************************************************************
 * \brief Sets the latency time
 * @param in_latency_time needed latency time
*******************************************************************/
void Camera::setLatencyTime(double in_latency_time) const
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setLatencyTime - " << DEB_VAR1(in_latency_time) << " (s)";

//    m_latency_time = latency_time;
}

//------------------------------------------------------------------
// number of frames management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Sets the number of frames
 * @param in_nb_frames number of needed frames
*******************************************************************/
void Camera::setNbFrames(int in_nb_frames)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setNbFrames - " << DEB_VAR1(in_nb_frames);
    if(in_nb_frames < 0)
        throw LIMA_HW_EXC(InvalidValue, "Invalid nb of frames");

//    m_nb_frames = nb_frames;
}

/*******************************************************************
 * \brief Gets the number of frames
 * \return number of frames
 *******************************************************************/
int Camera::getNbFrames() const
{
    DEB_MEMBER_FUNCT();
    int nb_frames = 0;
    //nb_frames = m_nb_frames;
    return nb_frames;
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
// CCA:TODO
	double          min_time = 10e-9;
	double          max_time = 10e+9;

    HwSyncCtrlObj::ValidRangesType valid_ranges;

	valid_ranges.min_exp_time = min_time;
	valid_ranges.max_exp_time = max_time;
	valid_ranges.min_lat_time = min_time;
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

//========================================================================================