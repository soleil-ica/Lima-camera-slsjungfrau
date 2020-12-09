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
 *  \brief  SlsJungfrau detector acquisition thread class interface 
 *  \author Cedric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************/

#ifndef SLSJUNGFRAUCAMERATHREAD_H
#define SLSJUNGFRAUCAMERATHREAD_H

#include "lima/Debug.h"
#include "SlsJungfrauCompatibility.h"
#include "lima/Constants.h"
#include "lima/HwBufferMgr.h"
#include "lima/ThreadUtils.h"
#include "lima/HwEventCtrlObj.h"

/*************************************************************************/
#define REPORT_EVENT(desc)  {   \
                                Event *my_event = new Event(Hardware,Event::Info, Event::Camera, Event::Default,desc); \
                                m_cam.getEventCtrlObj()->reportEvent(my_event); \
                            }
/*************************************************************************/

namespace lima
{
    namespace SlsJungfrau
    {
        /**********************************************************************/
        // defines the SlsJungfrau camera class for direct access
        class Camera;

        /***********************************************************************
         * \class CameraThread
         * \brief used to handle some specific tasks (startAcq, stopAcq, ...)
         ***********************************************************************/

        class CameraThread : public CmdThread
        {
            DEB_CLASS_NAMESPC(DebModCamera, "CameraThread", "SlsJungfrau");

        public:
			// Status
            enum
			{ 
				Idle    = MaxThreadStatus, // ready to manage acquisition
                Running                  , // acquisition is running 
                Error                    , // unexpected error
			};

            // Cmd
            enum
            { 
                StartAcq = MaxThreadCmd, // command used to start acquisition
            };

            // constructor
            CameraThread(Camera & cam);

            // destructor
            virtual ~CameraThread();

            // starts the thread
            virtual void start();

            // aborts the thread
            virtual void abort();

            // execute the stop command
            void execStopAcq();

        private:
            // execute the start command
            void execStartAcq();

            // treat all complete frames
            void treatCompleteFrames(Timestamp        in_start_timestamp,
                                     StdBufferCbMgr & in_buffer_mgr     );

            // build an intensity image in 24 bits
            void buildIntensityImage(uint32_t       * out_dest_image     ,
                                     const uint16_t * in_source_image    ,
                                     const double   * in_intensity_coeffs);

        private :
            volatile bool m_force_stop;

        protected:
            // inits the thread
            virtual void init();

            // command execution
            virtual void execCmd(int cmd);

        private:
            // direct access to camera
            Camera & m_cam;
        };
    }
}
#endif // SLSJUNGFRAUCAMERATHREAD_H

/*************************************************************************/