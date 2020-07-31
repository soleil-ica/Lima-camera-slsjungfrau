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
 *  \file   SlsJungfrauCameraThread.h
 *  \brief  SlsJungfrau detector acquisition thread class implementation 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************/

#include <cstdlib>

#include "lima/Exceptions.h"
#include "SlsJungfrauCameraThread.h"
#include "SlsJungfrauCamera.h"

#include <slsDetectorUsers.h>
#include <sls_detector_defs.h>

using namespace lima;
using namespace lima::SlsJungfrau;

#include <cmath>

// define it if you want an absolute timestamp in the frame data.
// let it commented if you want a relative timestamp in the frame data (starting at 0).
//#define CAMERA_THREAD_USE_ABSOLUTE_TIMESTAMP

/************************************************************************
 * \brief constructor
 ************************************************************************/
CameraThread::CameraThread(Camera & cam) : m_cam(cam)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "CameraThread::CameraThread - BEGIN";
    m_force_stop = false;
    DEB_TRACE() << "CameraThread::CameraThread - END";
}

/************************************************************************
 * \brief destructor
 ************************************************************************/
CameraThread::~CameraThread()
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "CameraThread::~CameraThread";
}

/************************************************************************
 * \brief starts the thread
 ************************************************************************/
void CameraThread::start()
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "CameraThread::start - BEGIN";
    CmdThread::start();
    waitStatus(CameraThread::Idle);
    DEB_TRACE() << "CameraThread::start - END";
}

/************************************************************************
 * \brief inits the thread
 ************************************************************************/
void CameraThread::init()
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "CameraThread::init - BEGIN";
    setStatus(CameraThread::Idle);
    DEB_TRACE() << "CameraThread::init - END";
}

/************************************************************************
 * \brief aborts the thread
 ************************************************************************/
void CameraThread::abort()
{
	DEB_MEMBER_FUNCT();
    CmdThread::abort();
}

/************************************************************************
 * \brief command execution
 * \param cmd command indentifier
************************************************************************/
void CameraThread::execCmd(int cmd)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "CameraThread::execCmd - BEGIN";
    int status = getStatus();

    try
    {
        switch (cmd)
        {
            case CameraThread::StartAcq:
                if (status == CameraThread::Idle)
                    execStartAcq();
                break;

            default:
                break;
        }
    }
    catch (...)
    {
    }

    DEB_TRACE() << "CameraThread::execCmd - END";
}

/************************************************************************
 * \brief execute the stop command
 ************************************************************************/
void CameraThread::execStopAcq()
{
    DEB_MEMBER_FUNCT();

    if(getStatus() == CameraThread::Running)
    {
        m_force_stop = true;

        // Waiting for thread to finish or to be in error
        waitNotStatus(CameraThread::Running);
    }
}

/************************************************************************
 * \brief execute the start command
 ************************************************************************/
void CameraThread::execStartAcq()
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "executing StartAcq command...";

    const double sleep_time_sec = 0.5; // sleep the thread in seconds

    m_force_stop = false;

    // the thread is running a new acquisition (it frees the Camera::startAcq method)
    setStatus(CameraThread::Running);

    // checking if we are running an acquisition which will build images with 24 bits intensities
    if(m_cam.areGainCoeffsLoaded())
    {
        if(!m_cam.areIntensityCoeffsValid())
        {
            setStatus(CameraThread::Error);

            std::string errorText = "Can not start the acquisition, you should calibrate the detector first!";
            REPORT_EVENT(errorText);
            return;
        }
    }

    // making an alias on buffer manager
    StdBufferCbMgr & buffer_mgr = m_cam.m_buffer_ctrl_obj.getBuffer();

    // starting receiver listening mode
    if(m_cam.startReceiver() == slsDetectorDefs::FAIL)
    {
        setStatus(CameraThread::Error);

        std::string errorText = "Can not start the receiver listening mode!";
        REPORT_EVENT(errorText);
        return;
    }

    // starting real time acquisition in non blocking mode
    // returns OK if all detectors are properly started, FAIL otherwise
    if(m_cam.startAcquisition() == slsDetectorDefs::FAIL)
    {
        m_cam.stopReceiver();

        setStatus(CameraThread::Error);

        std::string errorText = "Can not start real time acquisition!";
        REPORT_EVENT(errorText);
        return;
    }

    // setting the start timestamp
    Timestamp start_timestamp = Timestamp::now();
    buffer_mgr.setStartTimestamp(start_timestamp);

    // Main acquisition loop
    // m_force_stop can be set to true by the execStopAcq call to abort an acquisition
    // m_force_stop can be set to true also with an error hardware camera status
    // the loop can also end when all the frames are acquired
    while ((!m_force_stop)&&(m_cam.getNbAcquiredFrames() < m_cam.getInternalNbFrames()))
    {
        CameraFrames & frame_manager = m_cam.getFrameManager();

        // treating all complete frames
        treatCompleteFrames(start_timestamp, buffer_mgr);

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
    if((m_force_stop) || (m_cam.getNbAcquiredFrames() < m_cam.getInternalNbFrames()))
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

        // treating all complete frames which were not treated yet
        treatCompleteFrames(start_timestamp, buffer_mgr);

        // problem occured during the acquisition ?
        if(!m_force_stop)
        {
            size_t lost_frames_nb = m_cam.getInternalNbFrames() - m_cam.getNbAcquiredFrames();

            // checking again because sometimes the camera status can change to idle before we receive the frame
            if(lost_frames_nb > 0)
            {
                setStatus(CameraThread::Error);

                size_t received;
                size_t complete;
                size_t treated ;
                
                m_cam.getNbFrames(received, complete, treated );

                DEB_TRACE() << "frames received (" << received << ")";
                DEB_TRACE() << "frames complete (" << complete << ")";
                DEB_TRACE() << "frames treated ("  << treated  << ")";

                std::stringstream tempStream;
                tempStream << "Lost " << lost_frames_nb << " frames during real time acquisition!";
                REPORT_EVENT(tempStream.str());
                return;
            }
        }
    }
    else
    // no problem occured during the acquisition
    {
        // stopping receiver listening mode
        if(m_cam.stopReceiver() == slsDetectorDefs::FAIL)
        {
            setStatus(CameraThread::Error);

            std::string errorText = "Could not stop real time acquisition!";
            REPORT_EVENT(errorText);
            return;
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

    // change the thread status only if the thread is not in error
    if(getStatus() == CameraThread::Running)
    {
        setStatus(CameraThread::Idle);
    }
}

/************************************************************************
 * \brief treat all complete frames
 * \param in_start_timestamp start timestamp
 * \param in_buffer_mgr buffer manager
************************************************************************/
void CameraThread::treatCompleteFrames(Timestamp        in_start_timestamp,
                                       StdBufferCbMgr & in_buffer_mgr     )
{
    DEB_MEMBER_FUNCT();

    CameraFrames & frame_manager = m_cam.getFrameManager();

    // checking if there is a complete frame available
    while(frame_manager.completeAvailable())
    {
        HwFrameInfoType frame_info;
        double frame_timestamp_sec;

        {
            // getting a reference to the first complete frame from complete frames container
            const CameraFrame & complete_frame = frame_manager.getFirstComplete();

            frame_info.acq_frame_nb = complete_frame.getIndex();

            // converting the timestamp which is in 10MHz to seconds
            frame_timestamp_sec = static_cast<double>(complete_frame.getTimestamp()) / 10000000.0;

            // do we need to build a 24 bits intensity image ?
            if(m_cam.areGainCoeffsLoaded())
            {
                lima::FrameDim frame_dim = in_buffer_mgr.getFrameDim();

                // checking the frame size 
                int         mem_size  = frame_dim.getMemSize();
                std::size_t data_size = m_cam.m_width * m_cam.m_height * 4; // 24 bits store into 32 bits -> 4 bytes

                if(mem_size != data_size)
                {
                    DEB_TRACE() << "Incoherent sizes during frame build process : " << 
                                   "frame size (" << data_size << ")" <<
                                   "lima size ("  << mem_size  << ")";

                    return;
                }

                // build the frame
                uint32_t * dest_buffer = static_cast<uint32_t *>(in_buffer_mgr.getFrameBufferPtr(frame_info.acq_frame_nb));

                // getting access to the image in the complete frame
                const std::vector<uint16_t> & image = complete_frame.getImage();

                buildIntensityImage(dest_buffer, image.data(),  m_cam.m_intensity_coeffs.data());
            }
        }

    #ifdef CAMERA_THREAD_USE_ABSOLUTE_TIMESTAMP
        frame_info.frame_timestamp = in_start_timestamp + Timestamp(frame_timestamp_sec);
    #else
        frame_info.frame_timestamp = Timestamp(frame_timestamp_sec);
    #endif

        //Pushing the image buffer through Lima 
        DEB_TRACE() << "New Frame Ready [ " << frame_info.acq_frame_nb << ", " << frame_timestamp_sec << " ]";
        in_buffer_mgr.newFrameReady(frame_info);

        // the complete frame has been treated, so we move it to the final container
        frame_manager.moveFirstCompleteToTreated();
    }
}

/************************************************************************
 * \brief build an intensity image in 24 bits
 * \param out_dest_buffer : destination image buffer (32 bits)
 * \param in_source_image : source image buffer (16 bits)
 * \param in_intensity_coeffs : container used to store the dark images and gain coefficients for each gain type 
************************************************************************/
void CameraThread::buildIntensityImage(uint32_t       * out_dest_image     ,
                                       const uint16_t * in_source_image    ,
                                       const double   * in_intensity_coeffs)
{
DEB_MEMBER_FUNCT();

    uint32_t nb = m_cam.m_width * m_cam.m_height;
    uint32_t intensity;
    double   temp;
    
uint32_t i = 0;

    while(nb--)
    {
        // compute the position for the gain index in the pixel
        in_intensity_coeffs += (2 * SLS_GET_GAIN_FROM_PIXEL(*in_source_image));

        // compute the intensity (adu - dark) * gain coeff
        temp      = (static_cast<double>(SLS_GET_ADU_FROM_PIXEL(*in_source_image)) - *in_intensity_coeffs) * (*(in_intensity_coeffs + 1)) + 0.5;
double   temp2 = temp;
        temp      = fmax(temp, 0); // removing negative numbers
        intensity = static_cast<uint32_t>(temp);

        *out_dest_image++ = intensity;

if(SLS_GET_GAIN_FROM_PIXEL(*in_source_image) != 3)
{
    DEB_TRACE() << "-----------------";
    DEB_TRACE() << "in_source_image = " << *in_source_image;
    DEB_TRACE() << "gain (" << i << ")= " << SLS_GET_GAIN_FROM_PIXEL(*in_source_image);
    DEB_TRACE() << "adu (" << i << ")= " << SLS_GET_ADU_FROM_PIXEL(*in_source_image);
    DEB_TRACE() << "dark = " << *in_intensity_coeffs;
    DEB_TRACE() << "coeff = " << *(in_intensity_coeffs + 1);
    DEB_TRACE() << "temp2 = " << temp2;
    DEB_TRACE() << "intensity = " << intensity;
}
i++;
/*      
        *out_dest_image++ = (intensity & 0x00FF0000) >> 16;
        *out_dest_image++ = (intensity & 0x0000FF00) >> 8 ;
        *out_dest_image++ = (intensity & 0x000000FF);
*/

        // compute the position for the first gain index of the next pixel
        in_intensity_coeffs += (2 * (4 - SLS_GET_GAIN_FROM_PIXEL(*in_source_image)));
        in_source_image++;
    }
}

//========================================================================================
