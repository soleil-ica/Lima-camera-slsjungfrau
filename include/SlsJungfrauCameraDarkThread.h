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
 *  \brief  SlsJungfrau detector dark images generator thread class interface
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************/

#ifndef SLSJUNGFRAUCAMERADARKTHREAD_H
#define SLSJUNGFRAUCAMERADARKTHREAD_H

#include "SlsJungfrauCameraThread.h"

namespace lima
{
    namespace SlsJungfrau
    {
        /**********************************************************************/
        // defines the SlsJungfrau camera class for direct access
        class Camera;

        /***********************************************************************
         * \class CameraDarkThread
         * \brief used to handle some specific tasks (Calibrate,...)
         ***********************************************************************/

        class CameraDarkThread : public CmdThread
        {
            DEB_CLASS_NAMESPC(DebModCamera, "CameraDarkThread", "SlsJungfrau");

        public:
			// Status
            enum
			{ 
				Idle    = MaxThreadStatus, // ready to manage acquisition
                Running                  , // calibration is running 
                Error                    , // unexpected error
			};

            // Cmd
            enum
            { 
                StartCalibration = MaxThreadCmd, // command used to start the calibration process
            };

            // constructor
            CameraDarkThread(Camera & cam);

            // destructor
            virtual ~CameraDarkThread();

            // starts the thread
            virtual void start();

            // aborts the thread
            virtual void abort();

            // execute the stop the calibration command
            void execStopCalibration();

        private:
            // execute the start calibration command
            void execStartCalibration();

            // create a dark image
            bool createDarkImage(int in_gain_index);

            // accumulate an image
            void accumulateImage(uint32_t *       out_acc_buffer ,
                                 const uint16_t * in_image_buffer);

            // compute the average with an accumulation buffer
            void averageImage(uint16_t *       out_image_buffer,
                              const uint32_t * in_acc_buffer   ,
                              long             in_acc_images_nb);

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
#endif // SLSJUNGFRAUCAMERADARKTHREAD_H

/*************************************************************************/