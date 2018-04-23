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
                if (status != CameraThread::Idle)
                    throw LIMA_HW_EXC(InvalidValue, "Not Ready to StartAcq");
                execStartAcq();
                break;

            case CameraThread::StopAcq:
                if (status != CameraThread::Running)
                    throw LIMA_HW_EXC(InvalidValue, "Not Ready to StopAcq");
                execStopAcq();
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
    m_force_stop = true;
}

/************************************************************************
 * \brief execute the start command
 ************************************************************************/
void CameraThread::execStartAcq()
{
    DEB_MEMBER_FUNCT();
	DEB_TRACE() << "executing StartAcq command...";

    m_force_stop = false;

    // the tread is running a new acquisition
    setStatus(CameraThread::Running);

    // making an alias on buffer manager
    StdBufferCbMgr & buffer_mgr = m_cam.m_buffer_ctrl_obj.getBuffer();

    // getting an alias on the detector control instance
    lima::AutoPtr<slsDetectorUsers> detector_control = m_cam.m_detector_control;

    // starting receiver listening mode
    if(detector_control->startReceiver() == slsDetectorDefs::FAIL)
	{
        setStatus(CameraThread::Error);

        std::string errorText = "CameraThread::execStartAcq - can not start the receiver listening mode!";
        REPORT_EVENT(errorText);
        return;
	}

    // starting real time acquisition in non blocking mode
    // returns OK if all detectors are properly started, FAIL otherwise
    if(detector_control->startAcquisition() == slsDetectorDefs::FAIL)
	{
        detector_control->stopReceiver();

        setStatus(CameraThread::Error);

        std::string errorText = "CameraThread::execStartAcq - can not start real time acquisition!";
        REPORT_EVENT(errorText);
        return;
	}

    // setting the start timestamp
    Timestamp start_timestamp = Timestamp::now();
	buffer_mgr.setStartTimestamp(start_timestamp);

	// Main acquisition loop
    // m_force_stop can be set to true by the StopAcq command to abort an acquisition
    // m_force_stop can be set to true also with an error hardware camera status
    // the loop can also end if the number of 
    while ((!m_force_stop)&&(m_cam.getNbAcquiredFrames() < m_cam.getInternalNbFrames()))
    {
        CameraFrames & frame_manager = m_cam.getFrameManager();
        CameraFrame    complete_frame;

        // getting the first complete frame from complete frames container
        while(frame_manager.getFirstComplete(complete_frame))
        {
            // TO DO - manage the packets lost
            // complete_frame.getPacketNumber();

            HwFrameInfoType frame_info;
            frame_info.acq_frame_nb    = complete_frame.getIndex();
            frame_info.frame_timestamp = start_timestamp + Timestamp(complete_frame.getTimestamp());

            //Pushing the image buffer through Lima 
            DEB_TRACE() << "New Frame Ready (" << frame_info.acq_frame_nb << ")";
            buffer_mgr.newFrameReady(frame_info);

            // the complete frame has been treated, so we move it to the final container
            frame_manager.moveFirstCompleteToTreated();
        }

        // checking if the hardware acquisition is running and if there is no more frame to treat in the containers
        Camera::Status status = m_cam.getStatus();

        if((status != Camera::Waiting) && 
           (status == Camera::Running) &&
           (frame_manager.NoMoreFrameToTreat()))
        {
            // we stop the treatment - seems we have lost frame(s)
            break;
        }
        
        lima::Sleep(1); // sleep the tread for one second
    }

    // acquisition was aborted
    if(m_force_stop)
    {
        Camera::Status status = m_cam.getStatus();

        // checking if the hardware acquisition is running
        if((status == Camera::Waiting) || 
           (status == Camera::Running))
        {
            // stop detector acquisition
            if(detector_control->stopAcquisition() == slsDetectorDefs::FAIL)
	        {
                detector_control->stopReceiver(); // try to stop receiver listening mode

                setStatus(CameraThread::Error);

                std::string errorText = "CameraThread::execStartAcq - can not stop real time acquisition!";
                REPORT_EVENT(errorText);
                return;
	        }
        }
    }
    else
    // waiting for an idle hardware status
    {
        for(;;)
        {
            Camera::Status status = m_cam.getStatus();

            // checking if the hardware acquisition is running
            if((status != Camera::Waiting) && 
               (status == Camera::Running))
            {
                break;
            }
            else
            {
                lima::Sleep(1); // sleep the tread for one second
            }
        }
    }

    // stopping receiver listening mode
    if(detector_control->stopReceiver() == slsDetectorDefs::FAIL)
    {
        setStatus(CameraThread::Error);

        std::string errorText = "CameraThread::execStartAcq - can not stop real time acquisition!";
        REPORT_EVENT(errorText);
        return;
    }

    setStatus(CameraThread::Idle);

    // Start acquisition on aviex detector
//    m_cam._armDetector();

    /////////////////////////////////////////////////////////////////////////////////////////
    // Loop while :
    //                m_nb_frames is not reached OR 
    //                stop() is requested OR 
    //                to infinity in "live mode"
    /////////////////////////////////////////////////////////////////////////////////////////
/*    DEB_TRACE() << "CameraThread::execStartAcq - Loop while 'Nb. acquired frames' < " << (m_cam.m_nb_frames) << " ...";
    bool continueAcq = true;
    while(continueAcq && (!m_cam.m_nb_frames || m_cam.m_acq_frame_nb < (m_cam.m_nb_frames - 1)))
    {
        //Force quit the thread if command stop() is launched by client
        if(m_force_stop)
        {
            //abort the current acquisition et set internal driver state to IDLE
            continueAcq = false;
            m_force_stop = false;
            goto ForceTheStop;
        }

        //To prevent flooding of traces
        if(!bTraceAlreadyDone)
        {
            DEB_TRACE() << "\n";
            DEB_TRACE() << "Waiting for the Mx Image Acquisition ...";
            bTraceAlreadyDone = true;
        }

        mx_status_type mx_status;
        long last_frame_num = -1;
        long total_frame_num = -1;

        //get the total nulmber of frames in order to check if a frame is available !
        mx_status = mx_area_detector_get_total_num_frames(m_cam.m_mx_record, &total_frame_num);
        CHECK_MX_STATUS(mx_status, "Camera::execStartAcq()");

        //compute the last frame number
        last_frame_num = total_frame_num - initial_fram_num - 1;

        //Wait while new Frame is not ready 
        if(last_frame_num <= m_cam.m_acq_frame_nb)
        {
            mx_msleep(1); //sleep 1ms
            continue;
        }

        DEB_TRACE() << "\t- total_frame_num  = " << total_frame_num;
        DEB_TRACE() << "\t- last_frame_num  = " << last_frame_num;

        //New image(s) is ready
        while(last_frame_num > m_cam.m_acq_frame_nb)
        {
            int current_frame_nb = m_cam.m_acq_frame_nb + 1;
            //Prepare frame Lima Ptr ...
            bTraceAlreadyDone = false;
            DEB_TRACE() << "\t- Prepare the Lima Frame ptr - " << DEB_VAR1(current_frame_nb);
            setStatus(Readout);
            void *ptr = buffer_mgr.getFrameBufferPtr(current_frame_nb);

            //Prepare frame Mx Ptr ...
            DEB_TRACE() << "\t- Prepare the Mx Frame ptr - " << DEB_VAR1(current_frame_nb);
            MX_IMAGE_FRAME *image_frame = NULL;

            //Send corrections flags
            DEB_TRACE() << "\t- Send the list of corrections flags to Mx library - correction_flags = " << m_cam.m_correction_flags;
            mx_area_detector_set_correction_flags(m_cam.m_mx_record, m_cam.m_correction_flags);

            //Get the last image
            DEB_TRACE() << "\t- Get the last Frame From Mx - " << DEB_VAR1(current_frame_nb);
            //utility function that make : setup_frame() + readout_frame() + correct_frame() + transfer_frame().
            MX_AREA_DETECTOR* ad = (MX_AREA_DETECTOR*) (m_cam.m_mx_record->record_class_struct);
            mx_status = mx_area_detector_get_frame(m_cam.m_mx_record, current_frame_nb, &(ad->image_frame));
            CHECK_MX_STATUS(mx_status, "Camera::execStartAcq()");
            image_frame = ad->image_frame;

            //compute the timestamp of current image
            DEB_TRACE() << "\t- Compute the Timestamp for the image - " << DEB_VAR1(current_frame_nb);
            double timestamp = m_cam.computeTimestamp(image_frame, current_frame_nb);

            //copy from the Mx buffer to the Lima buffer
            DEB_TRACE() << "\t- Copy Frame From Mx Ptr into the Lima ptr :";
            size_t nb_bytes_to_copy = m_cam.m_frame_size.getWidth() * m_cam.m_frame_size.getHeight() * sizeof (unsigned short);

            size_t nb_bytes_copied;
            mx_status = mx_image_copy_1d_pixel_array(image_frame,
                                                     (unsigned short *) ptr,
                                                     nb_bytes_to_copy,
                                                     &nb_bytes_copied);
            CHECK_MX_STATUS(mx_status, "Camera::execStartAcq()");

            DEB_TRACE() << "\t- Timestamp  = " << std::fixed << timestamp << " (s)";
            DEB_TRACE() << "\t- Frame size  = " << m_cam.m_frame_size;
            DEB_TRACE() << "\t- NB. Bytes to Copy = " << nb_bytes_to_copy;
            DEB_TRACE() << "\t- NB. Copied Bytes  = " << nb_bytes_copied;


            Timestamp computed_timestamp(timestamp);
            buffer_mgr.setStartTimestamp(computed_timestamp);

            //Push the image buffer through Lima 
            DEB_TRACE() << "\t- Declare a Lima new Frame Ready (" << current_frame_nb << ")";
            HwFrameInfoType frame_info;
            frame_info.acq_frame_nb = current_frame_nb;
            frame_info.frame_timestamp = computed_timestamp;
            buffer_mgr.newFrameReady(frame_info);
            m_cam.m_acq_frame_nb = current_frame_nb;
            DEB_TRACE() << "\n";
        }
    } // End while
*/

/*    ForceTheStop: 
    {
       // stop acquisition
       DEB_TRACE() << "\n";
       DEB_TRACE() << "Stop the Acquisition.";

//       mx_status = mx_area_detector_stop(m_cam.m_mx_record);
//       CHECK_MX_STATUS(mx_status, "Camera::execStartAcq()")

       setStatus(CameraThread::Ready);
       DEB_TRACE() << "CameraThread::execStartAcq - END";
    }*/
}

//========================================================================================
