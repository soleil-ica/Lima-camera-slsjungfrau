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
 *  \brief  SlsJungfrau receiver informations class implementation 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************************/

#include "SlsJungfrauCameraReceiverInfo.h"

#include <slsReceiverUsers.h>

using namespace lima::SlsJungfrau;

/************************************************************************
 * \brief constructor without parameter
 ************************************************************************/
CameraReceiverInfo::CameraReceiverInfo()
{
    m_receiver   = NULL;
    m_host_name  = "undefined";
    m_tcpip_port = 0;
}

/************************************************************************
 * \brief gets the receiver
 * \return receiver smart pointer
 ************************************************************************/
lima::AutoPtr<slsReceiverUsers > CameraReceiverInfo::getReceiver()
{
    return m_receiver;
}

/************************************************************************
 * \brief sets the receiver
 * \param in_receiver new receiver smart pointer
 ************************************************************************/
void CameraReceiverInfo::setReceiver(lima::AutoPtr<slsReceiverUsers > in_receiver)
{
    m_receiver = in_receiver;
}

/************************************************************************
 * \brief gets the host name
 * \return host name value
 ************************************************************************/
const std::string & CameraReceiverInfo::getHostName() const
{
    return m_host_name;
}

/************************************************************************
 * \brief sets the host name
 * \param in_host_name new host name value
 ************************************************************************/
void CameraReceiverInfo::setHostName(const std::string & in_host_name)
{
    m_host_name = in_host_name;
}

/************************************************************************
 * \brief gets the tcpip port
 * \return tcpip port value
 ************************************************************************/
int CameraReceiverInfo::getTcpipPort() const
{
    return m_tcpip_port;
}

/************************************************************************
 * \brief sets the tcpip port
 * \param in_tcpip_port new tcpip port value
 ************************************************************************/
void CameraReceiverInfo::setTcpipPort(const int in_tcpip_port)
{
    m_tcpip_port = in_tcpip_port;
}

//========================================================================================
