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

/*************************************************************************************/
/*! 
 *  \file   SlsJungfrauCameraFrame.h
 *  \brief  SlsJungfrau detector frame class implementation 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************************/

#include <string.h>
#include "SlsJungfrauCameraFrame.h"

using namespace lima::SlsJungfrau;

/************************************************************************
 * \brief constructor with parameters
 * \param in_index frame index (starts at 0)
 * \param in_packet_number number of packets caught for this frame
 * \param in_timestamp time stamp in 10MHz clock
 ************************************************************************/
CameraFrame::CameraFrame(uint64_t in_index        ,
                         uint32_t in_packet_number,
                         uint64_t in_timestamp    )
{
    m_index         = in_index        ;
    m_packet_number = in_packet_number;
    m_timestamp     = in_timestamp    ;
}

/************************************************************************
 * \brief constructor without parameter
 ************************************************************************/
CameraFrame::CameraFrame()
{
    m_index         = 0;
    m_packet_number = 0;
    m_timestamp     = 0;
}

/************************************************************************
 * \brief gets the frame index
 * \return frame index value
 ************************************************************************/
uint64_t CameraFrame::getIndex() const
{
    return m_index;
}

/************************************************************************
 * \brief gets the packet number
 * \return packet number value
 ************************************************************************/
uint32_t CameraFrame::getPacketNumber() const
{
    return m_packet_number;
}

/************************************************************************
 * \brief gets the timestamp
 * \return timestamp value
 ************************************************************************/
uint64_t CameraFrame::getTimestamp() const
{
    return m_timestamp;
}

/************************************************************************
 * \brief fill the internal image
 * \param in_data_pointer frame image pointer
 * \param in_data_size frame image size 
 ************************************************************************/
void CameraFrame::setImage(const char *   in_data_pointer,
                           const uint32_t in_data_size   )
{
    m_image.resize(in_data_size / sizeof(uint16_t));
    memcpy(m_image.data(), in_data_pointer, in_data_size);
}

/************************************************************************
 * \brief clear the internal image to free the memory
 * \return none
 ************************************************************************/
void CameraFrame::clearImage()
{
    m_image.clear();
}

/************************************************************************
 * \brief get a reference to the internal image
 * \return reference to the internal image
 ************************************************************************/
const std::vector<uint16_t> & CameraFrame::getImage() const
{
    return m_image;
}

//========================================================================================
