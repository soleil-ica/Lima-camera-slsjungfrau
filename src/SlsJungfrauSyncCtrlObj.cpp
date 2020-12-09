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
 *  \file   SlsJungfrauSyncCtrlObj.cpp
 *  \brief  SlsJungfrau synchronization class implementation 
 *  \author Cedric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************/

#include "SlsJungfrauCamera.h"
#include "SlsJungfrauSyncCtrlObj.h"

using namespace lima;
using namespace lima::SlsJungfrau;

/*******************************************************************
 * \brief constructor
 *******************************************************************/
SyncCtrlObj::SyncCtrlObj(Camera & cam) : HwSyncCtrlObj(), m_cam(cam)
{
    DEB_CONSTRUCTOR();
}

/*******************************************************************
 * \brief destructor
 *******************************************************************/
SyncCtrlObj::~SyncCtrlObj()
{
}

//==================================================================
// trigger mode management
//==================================================================
/*******************************************************************
 * \brief Checks the trigger mode validity
 *******************************************************************/
bool SyncCtrlObj::checkTrigMode(TrigMode trig_mode)
{
    DEB_MEMBER_FUNCT();
    return m_cam.checkTrigMode(trig_mode);
}

/*******************************************************************
 * \brief Sets the trigger mode
 *******************************************************************/
void SyncCtrlObj::setTrigMode(TrigMode trig_mode)
{
    DEB_MEMBER_FUNCT();

    if (!checkTrigMode(trig_mode))
        THROW_HW_ERROR(InvalidValue) << "Invalid trigger mode:" << DEB_VAR1(trig_mode);

    m_cam.setTrigMode(trig_mode);
}

/*******************************************************************
 * \brief Gets the trigger mode
 *******************************************************************/
void SyncCtrlObj::getTrigMode(TrigMode & trig_mode)
{
    DEB_MEMBER_FUNCT();
    trig_mode = m_cam.getTrigMode();
}

//==================================================================
// exposure time management
//==================================================================
/*******************************************************************
 * \brief Sets the exposure time
 *******************************************************************/
void SyncCtrlObj::setExpTime(double exp_time)
{
    DEB_MEMBER_FUNCT();
    
    // no need to convert exp_time because 
    // - camera exp time is in seconds
    // - lima exp time is also in seconds (but shows in ms)
    m_cam.setExpTime(exp_time);
}

/*******************************************************************
 * \brief Gets the exposure time
 *******************************************************************/
void SyncCtrlObj::getExpTime(double & exp_time)
{
    DEB_MEMBER_FUNCT();

    // no need to convert exp_time because 
    // - camera exp time is in seconds
    // - lima exp time is also in seconds (but shows in ms)
    exp_time = m_cam.getExpTime();
}

//==================================================================
// latency time management
//==================================================================
/*******************************************************************
 * \brief Sets the latency time
 *******************************************************************/
void SyncCtrlObj::setLatTime(double lat_time)
{
    DEB_MEMBER_FUNCT();

    // no need to convert lat_time because 
    // - camera latency time is in seconds
    // - lima latency time is also in seconds (but shows in ms)
    m_cam.setLatencyTime(lat_time);
}

/*******************************************************************
 * \brief Gets the latency time
 *******************************************************************/
void SyncCtrlObj::getLatTime(double & lat_time)
{
    DEB_MEMBER_FUNCT();

    // no need to convert lat_time because 
    // - camera latency time is in seconds
    // - lima latency time is also in seconds (but shows in ms)
    lat_time = m_cam.getLatencyTime();
}

//==================================================================
// number of frames management
//==================================================================
/*******************************************************************
 * \brief Sets the number of frames
 *******************************************************************/
void SyncCtrlObj::setNbHwFrames(int nb_frames)
{
    DEB_MEMBER_FUNCT();
    m_cam.setNbFrames(nb_frames);
}

/*******************************************************************
 * \brief Gets the number of frames
 *******************************************************************/
void SyncCtrlObj::getNbHwFrames(int & nb_frames)
{
    DEB_MEMBER_FUNCT();
    nb_frames = m_cam.getNbFrames();
}

//==================================================================
// valid ranges management
//==================================================================
/*******************************************************************
 * \brief Gets exposure and latency ranges
 *******************************************************************/
void SyncCtrlObj::getValidRanges(ValidRangesType & valid_ranges)
{
    DEB_MEMBER_FUNCT();
    valid_ranges = m_cam.getValidRanges();
}

/*************************************************************************/