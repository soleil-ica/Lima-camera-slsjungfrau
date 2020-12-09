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
 *  \file   SlsJungfrauInterface.h
 *  \brief  SlsJungfrau hardware class interface 
 *  \author Cedric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************/

#ifndef SLSJUNGFRAUINTERFACE_H
#define SLSJUNGFRAUINTERFACE_H

#include "lima/Debug.h"
#include "SlsJungfrauCompatibility.h"
#include "SlsJungfrauDetInfoCtrlObj.h"
#include "SlsJungfrauSyncCtrlObj.h"
#include "lima/HwInterface.h"
#include "lima/HwBufferMgr.h"

using namespace std;

namespace lima
{
    namespace SlsJungfrau
    {
        class Camera;

        /*******************************************************************
         * \class Interface
         * \brief SlsJungfrau hardware interface
         *******************************************************************/

        class LIBSLSJUNGFRAU_API Interface: public HwInterface
        {
            DEB_CLASS_NAMESPC(DebModCamera, "SlsJungfrauInterface", "SlsJungfrau");

        public:
            // constructor
            Interface(Camera & cam);

            // destructor
            virtual ~Interface();

            //==================================================================
            // capability list management
            //==================================================================
            // Gets the capability list
            virtual void getCapList(CapList & cap_list) const;

            //==================================================================
            // reset management
            //==================================================================
            // resets the interface and stop the acquisition
            virtual void reset(ResetLevel reset_level);

            //==================================================================
            // acquisition management
            //==================================================================
            // prepares the acquisition
            virtual void prepareAcq();

            // starts the acquisition (start/snap)
            virtual void startAcq();

            // stops the acquisition
            virtual void stopAcq();

            //==================================================================
            // status management
            //==================================================================
            // Gets the current status
            virtual void getStatus(StatusType & status);

            //==================================================================
            // Acquired frames management
            //==================================================================
            // Gets the number of acquired frames
            virtual int getNbHwAcquiredFrames();
            
            //==================================================================
            // camera access methods
            //==================================================================
            // camera instance access for the client
            Camera & getCamera();

            // camera instance const access for the client
            const Camera& getCamera() const;

        private:
            Camera         & m_cam     ; // hardware class
            CapList          m_cap_list; // capability list
            DetInfoCtrlObj   m_det_info; // detector info control object
            SyncCtrlObj      m_sync    ; // synchronization data control object
            HwEventCtrlObj * m_event   ;
        };
    } // namespace SlsJungfrau
} // namespace lima

#endif // SLSJUNGFRAUINTERFACE_H

/*************************************************************************/