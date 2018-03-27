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
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************/

#ifndef SLSJUNGFRAUCAMERATHREAD_H
#define SLSJUNGFRAUCAMERATHREAD_H

#include "lima/Debug.h"
#include "SlsJungfrauCompatibility.h"
#include "lima/Constants.h"
#include "lima/HwBufferMgr.h"
#include "lima/ThreadUtils.h"

namespace lima
{
    namespace SlsJungfrau
    {
        class Camera;

        /***********************************************************************
         * \class CameraThread
         * \brief used to handle some specific tasks (startAcq, stopAcq, ...)
         ***********************************************************************/

        class CameraThread : public CmdThread
        {
            DEB_CLASS_NAMESPC(DebModCamera, "CameraThread", "SlsJungfrau");

        public:
            enum
            { // Status
                Ready = MaxThreadStatus, Exposure, Readout, Latency,
            };

            enum
            { // Cmd
                StartAcq = MaxThreadCmd, StopAcq,
            };

            // constructor
            CameraThread(Camera & cam);

            // starts the thread
            virtual void start();

            // returns the acquired frame number
            int getNbHwAcquiredFrames();

        public :
            bool m_force_stop;

        protected:
            // inits the thread
            virtual void init();

            // command execution
            virtual void execCmd(int cmd);

        private:
            // execute the start command
            void execStartAcq();

        private:
            Camera * m_cam;
        };
    }
}
#endif // SLSJUNGFRAUCAMERATHREAD_H

/*************************************************************************/