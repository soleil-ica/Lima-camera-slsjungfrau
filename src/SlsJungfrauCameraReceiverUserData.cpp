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
 *  \file   SlsJungfrauCameraReceiverInfo.h
 *  \brief  SlsJungfrau receiver user data class implementation 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************************/

#include "SlsJungfrauCameraReceiverUserData.h"
#include "SlsJungfrauCameraReceivers.h"

using namespace lima::SlsJungfrau;

/************************************************************************
 * \brief constructor without parameter
 ************************************************************************/
CameraReceiverUserData::CameraReceiverUserData()
{
    m_receivers      = NULL;
    m_receiver_index = 0   ;
}

/************************************************************************
 * \brief gets the access to the CameraReceivers object 
 * \return CameraReceivers smart pointer
 ************************************************************************/
lima::AutoPtr<CameraReceivers > CameraReceiverUserData::getReceivers()
{
    return m_receivers;
}

/************************************************************************
 * \brief sets the access to the CameraReceivers object
 * \param in_receivers CameraReceivers smart pointer
 ************************************************************************/
void CameraReceiverUserData::setReceivers(lima::AutoPtr<CameraReceivers > in_receivers)
{
    m_receivers = in_receivers;
}

/************************************************************************
 * \brief gets the receiver index
 * \return receiver index value
 ************************************************************************/
int CameraReceiverUserData::getReceiverIndex() const
{
    return m_receiver_index;
}

/************************************************************************
 * \brief sets the receiver index
 * \param in_receiver_index new receiver index 
 ************************************************************************/
void CameraReceiverUserData::setReceiverIndex(const int in_receiver_index)
{
    m_receiver_index = in_receiver_index;
}

//========================================================================================
