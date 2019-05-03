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
 *  \file   SlsJungfrauCameraFrame.h
 *  \brief  SlsJungfrau detector frame class interface 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/********************************************************************************/

#ifndef SLSJUNGFRAUCAMERAFRAME_H
#define SLSJUNGFRAUCAMERAFRAME_H

#include "lima/Debug.h"
#include "lima/Constants.h"

namespace lima
{
    namespace SlsJungfrau
    {
        /***********************************************************************
         * \class CameraFrame
         * \brief defines an info class used to store frame informations 
         ***********************************************************************/

        class CameraFrame
        {
            DEB_CLASS_NAMESPC(DebModCamera, "CameraFrame", "SlsJungfrau");

        public:
            //==================================================================
            // constructor with parameters
            CameraFrame(uint64_t in_index        ,  // frame index (starts at 0)
                        uint32_t in_packet_number,  // number of packets caught for this frame
                        uint64_t in_timestamp    ); // time stamp in 10MHz clock

            // constructor without parameter
            CameraFrame();

            //==================================================================
            // gets the frame index
            uint64_t getIndex() const;

            // gets the packet number
            uint32_t getPacketNumber() const;

            // gets the timestamp
            uint64_t getTimestamp() const;

            // fill the internal image
            void setImage(const char *   in_data_pointer,
                          const uint32_t in_data_size   );

            // clear the internal image to free the memory
            void clearImage();

            // get a reference to the internal image
            const std::vector<uint16_t> & getImage() const;

        private:
            uint64_t              m_index        ; // frame index (starts at 0)
            uint32_t              m_packet_number; // number of packets caught for this frame
            uint64_t              m_timestamp    ; // time stamp in 10MHz clock
            std::vector<uint16_t> m_image        ; // buffer used to store the image sent by the detector 
        }; 
    }
}
#endif // SLSJUNGFRAUCAMERAFRAME_H

/*************************************************************************/