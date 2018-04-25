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

/********************************************************************************/
/*! 
 *  \file   SlsJungfrauCameraReceiverInfo.h
 *  \brief  SlsJungfrau receiver user data class interface 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/********************************************************************************/

#ifndef SLSJUNGFRAUCAMERARECEIVERUSERDATA_H
#define SLSJUNGFRAUCAMERARECEIVERUSERDATA_H

#include "lima/Debug.h"
#include "lima/Constants.h"

namespace lima
{
    namespace SlsJungfrau
    {
        /**********************************************************************/
        // pre-defines the CameraReceivers class for CameraReceiverUserData friend links
        class CameraReceivers;

        /***********************************************************************
         * \class CameraReceiverUserData
         * \brief used as user data in sls sdk callbacks
         *
         * allows the callbacks :
         * - to access to the CameraReceivers object
         * - to know the receiver index which is not always given in
         *   the callback data
         ***********************************************************************/

        class CameraReceiverUserData
        {
            DEB_CLASS_NAMESPC(DebModCamera, "CameraReceiverUserData", "SlsJungfrau");

        public:
            //==================================================================
            // constructor
            CameraReceiverUserData();

            //==================================================================
            // gets the access to the CameraReceivers object 
            lima::AutoPtr<CameraReceivers > getReceivers();

            // sets the access to the CameraReceivers object
            void setReceivers(lima::AutoPtr<CameraReceivers > in_receivers);

            // gets the receiver index
            int getReceiverIndex() const;

            // sets the receiver index
            void setReceiverIndex(const int in_receiver_index);

        private:
            int                            m_receiver_index; // receiver index in m_receivers_info container
            lima::AutoPtr<CameraReceivers> m_receivers     ; // direct access to the CameraReceivers object
        }; 
    }
}
#endif // SLSJUNGFRAUCAMERARECEIVERUSERDATA_H

/*************************************************************************/