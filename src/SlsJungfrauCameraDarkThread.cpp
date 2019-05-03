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
 *  \file   SlsJungfrauCameraDarkThread.h
 *  \brief  SlsJungfrau detector dark images generator thread class implementation 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************/

#include <cstdlib>

#include "lima/Exceptions.h"
#include "SlsJungfrauCameraDarkThread.h"
#include "SlsJungfrauCamera.h"

#include <slsDetectorUsers.h>
#include <sls_detector_defs.h>

using namespace lima;
using namespace lima::SlsJungfrau;

#include <cmath>


/************************************************************************
 * \brief constructor
 ************************************************************************/
CameraDarkThread::CameraDarkThread(Camera & cam) : m_cam(cam)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "CameraDarkThread::CameraDarkThread - BEGIN";
    m_force_stop = false;
    DEB_TRACE() << "CameraDarkThread::CameraDarkThread - END";
}

/************************************************************************
 * \brief destructor
 ************************************************************************/
CameraDarkThread::~CameraDarkThread()
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "CameraDarkThread::~CameraDarkThread";
}

/************************************************************************
 * \brief starts the thread
 ************************************************************************/
void CameraDarkThread::start()
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "CameraDarkThread::start - BEGIN";
    CmdThread::start();
    waitStatus(CameraDarkThread::Idle);
    DEB_TRACE() << "CameraDarkThread::start - END";
}

/************************************************************************
 * \brief inits the thread
 ************************************************************************/
void CameraDarkThread::init()
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "CameraDarkThread::init - BEGIN";
    setStatus(CameraDarkThread::Idle);
    DEB_TRACE() << "CameraDarkThread::init - END";
}

/************************************************************************
 * \brief aborts the thread
 ************************************************************************/
void CameraDarkThread::abort()
{
	DEB_MEMBER_FUNCT();
    CmdThread::abort();
}

/************************************************************************
 * \brief command execution
 * \param cmd command indentifier
************************************************************************/
void CameraDarkThread::execCmd(int cmd)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "CameraDarkThread::execCmd - BEGIN";
    int status = getStatus();

    try
    {
        switch (cmd)
        {
            case CameraDarkThread::StartCalibration:
                if (status == CameraDarkThread::Idle)
                    execStartCalibration();
                break;

            default:
                break;
        }
    }
    catch (...)
    {
    }

    DEB_TRACE() << "CameraDarkThread::execCmd - END";
}

/************************************************************************
 * \brief execute the stop calibration command
 ************************************************************************/
void CameraDarkThread::execStopCalibration()
{
    DEB_MEMBER_FUNCT();

    if(getStatus() == CameraDarkThread::Running)
    {
        m_force_stop = true;

        // Waiting for thread to finish or to be in error
        waitNotStatus(CameraDarkThread::Running);
    }
}

/************************************************************************
 * \brief execute the start calibration command
 ************************************************************************/
void CameraDarkThread::execStartCalibration()
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "executing execStartCalibration command...";

    m_force_stop = false;
    m_cam.m_calibration_running = true;

    // the thread is running a new calibration (it frees the Camera::startCalibration method)
    setStatus(CameraDarkThread::Running);

    // reset the previous dark images and erase the files
    m_cam.resetDarkImageFile();

    // reset the intensity coeffs
    m_cam.resetIntensityCoeffs();

    // backup the current acquisition configuration
    Camera::CameraAcqConf backup_configuration;

    if(!m_cam.getAcqConf(backup_configuration))
    {
        setStatus(CameraDarkThread::Error);
        m_cam.m_calibration_running = false;

        std::stringstream tempStream;
        tempStream << "getAcqConf failed!";
        DEB_TRACE() << tempStream.str();

        REPORT_EVENT(tempStream.str());
        return;
    }

    // create the dark images
    bool problem_occured = false;
    int  gain_index;

    for(gain_index ; gain_index < SLS_NUMBER_OF_DARK_IMAGES ; gain_index++)
    {
        if(m_force_stop)
            break;

        if(!createDarkImage(gain_index))
        {
            problem_occured = true;
            break;
        }
    }

    // if there was a problem during the dark images creation, we reset the dark images which could have been created
    // and also we erase the created files
    if(problem_occured)
    {
        m_cam.resetDarkImageFile();
    }

    // restore the acquisition configuration
    if(!m_cam.setAcqConf(backup_configuration))
    {
        setStatus(CameraDarkThread::Error);
        m_cam.m_calibration_running = false;

        std::stringstream tempStream;
        tempStream << "setAcqConf failed!";
        DEB_TRACE() << tempStream.str();

        REPORT_EVENT(tempStream.str());
        return;
    }

    if((problem_occured)&&(!m_force_stop))
    {
        setStatus(CameraDarkThread::Error);
        m_cam.m_calibration_running = false;

        std::stringstream tempStream;
        tempStream << "createDarkImage failed for gain index " << gain_index << "!";
        DEB_TRACE() << tempStream.str();

        REPORT_EVENT(tempStream.str());
        return;
    }

    // update the dark images
    m_cam.updateDarkImagesData();

    // rebuild the intensity coeffs buffer
    m_cam.updateIntensityCoeffs();

    // change the thread status only if the thread is not in error
    if(getStatus() == CameraDarkThread::Running)
    {
        setStatus(CameraDarkThread::Idle);
        m_cam.m_calibration_running = false;
    }
}

/************************************************************************
 * \brief create a dark image
 * \param in_gain_index gain index
************************************************************************/
bool CameraDarkThread::createDarkImage(int in_gain_index)
{
    DEB_MEMBER_FUNCT();

    // create a buffer to accumulate the acquired images and compute the average
    std::vector<uint32_t> accumulation_buffer;
    accumulation_buffer.resize(m_cam.m_width * m_cam.m_height);
    memset(accumulation_buffer.data(), 0, m_cam.m_width * m_cam.m_height * sizeof(uint32_t));

    // define a container to store the gain modes to use for each gain index
    std::vector<slsDetectorDefs::detectorSettings> gain_modes{slsDetectorDefs::detectorSettings::DYNAMICGAIN  ,
                                                              slsDetectorDefs::detectorSettings::FORCESWITCHG1,
                                                              slsDetectorDefs::detectorSettings::FORCESWITCHG2};

    // change the current acquisition configuration
    Camera::CameraAcqConf dark_image_configuration;

    dark_image_configuration.m_nb_frames_per_cycle = m_cam.m_pedestal_nb_frames[in_gain_index];
    dark_image_configuration.m_nb_cycles           = 1LL;
    dark_image_configuration.m_trigger_mode        = lima::TrigMode::IntTrig;
    dark_image_configuration.m_exposures_sec       = m_cam.m_pedestal_exposures_sec[in_gain_index];
    dark_image_configuration.m_periods_sec         = m_cam.m_pedestal_periods_sec[in_gain_index];
    dark_image_configuration.m_gain_mode           = gain_modes[in_gain_index];
    
    if(!m_cam.setAcqConf(dark_image_configuration))
    {
        DEB_TRACE() << "setAcqConf failed in createDarkImage(...)!";
        return false;
    }
    
    // reset the number of caught frames in the sdk
    m_cam.m_detector_control->resetFramesCaughtInReceiver();

    // clear the frames containers
    m_cam.getFrameManager().clear();

    const double sleep_time_sec = 0.5; // sleep the thread in seconds

    // starting receiver listening mode
    if(m_cam.startReceiver() == slsDetectorDefs::FAIL)
    {
        DEB_TRACE() << "Can not start the receiver listening mode!";
        return false;
    }

    // starting real time acquisition in non blocking mode
    // returns OK if all detectors are properly started, FAIL otherwise
    if(m_cam.startAcquisition() == slsDetectorDefs::FAIL)
    {
        m_cam.stopReceiver();

        DEB_TRACE() << "Can not start real time acquisition!";
        return false;
    }

    CameraFrames & frame_manager = m_cam.getFrameManager();

    // Main acquisition loop
    // m_force_stop can be set to true by the execStopCalibration call to abort an acquisition
    // m_force_stop can be set to true also with an error hardware camera status
    // the loop can also end when all the frames are acquired
    while ((!m_force_stop)&&(frame_manager.getNbTreatedFrames() < dark_image_configuration.m_nb_frames_per_cycle))
    {
        // checking if there is a complete frame available
        while(frame_manager.completeAvailable())
        {
            {
                // getting a reference to the first complete frame from complete frames container
                const CameraFrame & complete_frame = frame_manager.getFirstComplete();

                // getting access to the image in the complete frame
                const std::vector<uint16_t> & image = complete_frame.getImage();

                accumulateImage(accumulation_buffer.data(), image.data());
            }

            // the complete frame has been treated, so we move it to the final container
            frame_manager.moveFirstCompleteToTreated();
        }

        // checking if the hardware acquisition is running and if there is no more frame to treat in the containers
        Camera::Status status = m_cam.getDetectorStatus();

        if((status != Camera::Waiting) && (status != Camera::Running))
        {
            lima::Sleep(sleep_time_sec); // sleep the thread in seconds because a data callback can be activated

            if(frame_manager.NoMoreFrameToTreat())
            {
                // we stop the treatment - seems we have lost frame(s)
                break;
            }
        }
        else
        // hardware acquisition is running, we are waiting for new frames not using the cpu during this time
        {
            lima::Sleep(sleep_time_sec); // sleep the thread in seconds
        }
    }

    // acquisition was aborted
    if((m_force_stop) || (frame_manager.getNbTreatedFrames() < dark_image_configuration.m_nb_frames_per_cycle))
    {
        Camera::Status status = m_cam.getDetectorStatus();

        // checking if the hardware acquisition is running or in error
        if((status == Camera::Waiting) || 
           (status == Camera::Running))
        {
            // stop detector acquisition
            m_cam.stopAcquisition();
        }

        // stop receiver listening mode
        m_cam.stopReceiver();

        // problem occured during the acquisition ?
        if(!m_force_stop)
        {
            size_t lost_frames_nb = dark_image_configuration.m_nb_frames_per_cycle - frame_manager.getNbTreatedFrames();

            // checking again because sometimes the camera status can change to idle before we receive the frame
            if(lost_frames_nb > 0)
            {
                size_t received;
                size_t complete;
                size_t treated ;
                
                m_cam.getNbFrames(received, complete, treated);

                DEB_TRACE() << "frames received (" << received << ")";
                DEB_TRACE() << "frames complete (" << complete << ")";
                DEB_TRACE() << "frames treated ("  << treated  << ")";

                std::stringstream tempStream;
                tempStream << "Lost " << lost_frames_nb << " frames during real time acquisition!";
                DEB_TRACE() << tempStream.str();

                return false;
            }
        }
    }
    else
    // no problem occured during the acquisition
    {
        // stopping receiver listening mode
        if(m_cam.stopReceiver() == slsDetectorDefs::FAIL)
        {
            DEB_TRACE() << "Could not stop real time acquisition!";
            return false;
        }

        // protecting the concurrent access
        lima::AutoMutex sdk_mutex = m_cam.calibrationLock();

        // computing the average image
        m_cam.m_pedestal_images.resize(m_cam.m_pedestal_images.size() + 1);
        std::vector<uint16_t> & pedestal_image = m_cam.m_pedestal_images[in_gain_index];
        pedestal_image.resize(m_cam.m_width * m_cam.m_height);

        averageImage(pedestal_image.data(),
                     accumulation_buffer.data(),
                     dark_image_configuration.m_nb_frames_per_cycle);

        // save the dark image
        if(!m_cam.saveDarkImageFile(m_cam.m_pedestal_file_names[in_gain_index],  pedestal_image))
        {
            DEB_TRACE() << "Could not save dark image during calibration!";
            return false;
        }
    }

    // waiting for an idle or error hardware status
    for(;;)
    {
        Camera::Status status = m_cam.getDetectorStatus();

        // checking if the hardware acquisition is running
        if((status != Camera::Waiting) && 
           (status != Camera::Running))
        {
            break;
        }
        else
        {
            lima::Sleep(sleep_time_sec); // sleep the thread in seconds
        }
    }

    return true;
}

/************************************************************************
 * \brief accumulate an image
 * \param out_acc_buffer  : accumulation buffer (32 bits)
 * \param in_image_buffer : image to accumulate (16 bits)
************************************************************************/
void CameraDarkThread::accumulateImage(uint32_t *       out_acc_buffer ,
                                       const uint16_t * in_image_buffer)
{
    uint32_t nb = m_cam.m_width * m_cam.m_height;

    while(nb--)
    {
        *out_acc_buffer += SLS_GET_ADU_FROM_PIXEL(*in_image_buffer);
        out_acc_buffer++;
        in_image_buffer++;
    }
}

/************************************************************************
 * \brief compute the average with an accumulation buffer
 * \param out_image_buffer : image to store the average (16 bits)
 * \param in_acc_buffer    : accumulation buffer (32 bits)
 * \param in_acc_images_nb : number of images which were accumulated
************************************************************************/
void CameraDarkThread::averageImage(uint16_t *       out_image_buffer,
                                    const uint32_t * in_acc_buffer   ,
                                    long             in_acc_images_nb)
{
    uint32_t nb = m_cam.m_width * m_cam.m_height;

    while(nb--)
    {
        *out_image_buffer = static_cast<uint16_t>((*in_acc_buffer) / in_acc_images_nb);
        out_image_buffer++;
        in_acc_buffer++;
    }
}

//========================================================================================
