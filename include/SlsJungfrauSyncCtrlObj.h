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
 *  \file   SlsJungfrauSyncCtrlObj.h
 *  \brief  SlsJungfrau synchronization class interface 
 *  \author Cedric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************/

#ifndef SLSJUNGFRAUSYNCCTRLOBJ_H
#define SLSJUNGFRAUSYNCCTRLOBJ_H

#include "lima/Debug.h"
#include "SlsJungfrauCompatibility.h"
#include "lima/HwSyncCtrlObj.h"
#include "lima/HwInterface.h"

namespace lima
{
    namespace SlsJungfrau
    {
        class Camera;

        /***********************************************************************
         * \class SyncCtrlObj
         * \brief Control object providing SlsJungfrau synchronization interface
         ***********************************************************************/

        class LIBSLSJUNGFRAU_API SyncCtrlObj: public HwSyncCtrlObj
        {
            DEB_CLASS_NAMESPC(DebModCamera, "SyncCtrlObj", "SlsJungfrau");

        public:
            // constructor
            SyncCtrlObj(Camera & cam);

            // destructor
            virtual ~SyncCtrlObj();

            //==================================================================
            // trigger mode management
            //==================================================================
            // Checks the trigger mode validity
            virtual bool checkTrigMode(TrigMode trig_mode);

            // Sets the trigger mode
            virtual void setTrigMode(TrigMode trig_mode);

            // Gets the trigger mode
            virtual void getTrigMode(TrigMode & trig_mode);

            //==================================================================
            // exposure time management
            //==================================================================
            // Sets the exposure time
            virtual void setExpTime(double exp_time);

            // Gets the exposure time
            virtual void getExpTime(double & exp_time);

            //==================================================================
            // latency time management
            //==================================================================
            // Sets the latency time
            virtual void setLatTime(double lat_time);

            // Gets the latency time
            virtual void getLatTime(double & lat_time);

            //==================================================================
            // number of frames management
            //==================================================================
            // Sets the number of frames
            virtual void setNbHwFrames(int nb_frames);

            // Gets the number of frames
            virtual void getNbHwFrames(int & nb_frames);

            //==================================================================
            // valid ranges management
            //==================================================================
            // Gets exposure and latency ranges
            virtual void getValidRanges(ValidRangesType & valid_ranges);

        private:
            Camera & m_cam;
        };
    } // namespace SlsJungfrau
} // namespace lima

#endif // SLSJUNGFRAUSYNCCTRLOBJ_H

/*************************************************************************/