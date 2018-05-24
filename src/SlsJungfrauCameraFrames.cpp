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
 *  \file   SlsJungfrauCameraFrames.h
 *  \brief  SlsJungfrau detector frames manager class implementation 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************************/

#include <cstdlib>
#include <algorithm>

#include "lima/Exceptions.h"
#include "lima/RegExUtils.h"

#include "SlsJungfrauCamera.h"
#include "SlsJungfrauCameraFrames.h"

using namespace lima;
using namespace lima::SlsJungfrau;

/************************************************************************
 * \brief constructor
 ************************************************************************/
CameraFrames::CameraFrames(Camera & cam) : m_cam(cam)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "CameraFrames::CameraFrames - BEGIN";
    DEB_TRACE() << "CameraFrames::CameraFrames - END";
}

/************************************************************************
 * \brief destructor
 ************************************************************************/
CameraFrames::~CameraFrames()
{
    DEB_DESTRUCTOR();
}

/************************************************************************
 * \brief add a new received frame
 * \param m_receiver_index receiver index
 * \param in_frame frame instance to treat
 ************************************************************************/
void CameraFrames::addReceived(const int           in_receiver_index,
                               const CameraFrame & in_frame         )
{
    DEB_MEMBER_FUNCT();

    // protecting the containers access
    lima::AutoMutex container_mutex = containersLock();

    // the received frames container management is not usefull when the camera has only one receiver
    // and we do not need to rebuild the image with image parts.
    // But in the case of an upgrade of Jungfrau camera, it will be easier to manage several receivers.

    // searching the frame in the received frames container
    {
        ReceivedFramesContainer::iterator received_search = m_received_frames.find(in_frame.getIndex());
        
        // this should never happen.
        if(received_search != m_received_frames.end()) 
        {
            DEB_TRACE() << "CameraFrames::addReceived - unkown problem - frame (" << DEB_VAR1(in_frame.getIndex()) << ") already received by the callback!";
            return;
        }
    }

    // inserting the frame in the received container.
    // will be usefull only with several receivers.
    {
        std::pair<ReceivedFramesContainer::iterator, bool> result = 
            m_received_frames.insert(std::make_pair(in_frame.getIndex(), in_frame));

        // this should never happen.
        if(!result.second)
        {
            DEB_TRACE() << "CameraFrames::addReceived - unkown problem - frame (" << DEB_VAR1(in_frame.getIndex()) << ") already received by the callback!";
            return;
        }
    }

    // moving the complete frame to complete frames container
    {
        ReceivedFramesContainer::iterator received_search = m_received_frames.find(in_frame.getIndex());
        
        // this should never happen.
        if(received_search == m_received_frames.end()) 
        {
            DEB_TRACE() << "CameraFrames::addReceived - unkown problem - frame (" << DEB_VAR1(in_frame.getIndex()) << ") should be in the received container!";
            return;
        }

        // insert the frame in the complete frame container
        const CameraFrame & frame = (*received_search).second;
        
        DEB_TRACE() << "New complete frame [ " << frame.getIndex()        << ", "
                                               << frame.getPacketNumber() << ", " 
                                               << frame.getTimestamp()    << " ]"; 

        m_complete_frames.push_back(frame);

        // removing the frame from the received frame container
        m_received_frames.erase(received_search);
    }
}

/************************************************************************
 * \brief get the first complete frame from complete frames container
 *        It does not remove the frame from the container.
 * \param out_frame frame instance filled with frame data
 * \return true if a complete frame was treated by this method
 ************************************************************************/
bool CameraFrames::getFirstComplete(CameraFrame & out_frame)
{
    DEB_MEMBER_FUNCT();

    bool result = false;

    // protecting the containers access
    lima::AutoMutex container_mutex = containersLock();

    if(!m_complete_frames.empty())
    {
        out_frame = m_complete_frames.front();
        result = true;
    }

    return result;
}

/************************************************************************
 * \brief move the first complete frame to the treated frame container
 ************************************************************************/
void CameraFrames::moveFirstCompleteToTreated()
{
    DEB_MEMBER_FUNCT();

    // protecting the containers access
    lima::AutoMutex container_mutex = containersLock();

    if(!m_complete_frames.empty())
    {
        CameraFrame frame = m_complete_frames.front();
        m_complete_frames.pop_front();
        m_treated_frames.push_back(frame);

        DEB_TRACE() << "New treated frame [ " << frame.getIndex()        << ", "
                                              << frame.getPacketNumber() << ", " 
                                              << frame.getTimestamp()    << " ]"; 
    }
    else
    // this should never happen.
    {
        DEB_TRACE() << "CameraFrames::moveFirstCompleteToTreated - unkown problem - container is empty!";
    }
}

/************************************************************************
 * \brief get the number of frames in the containers
 * \return the number of frames by type
 ************************************************************************/
void CameraFrames::getNbFrames(size_t & out_received, 
                               size_t & out_complete,
                               size_t & out_treated ) const
{
    DEB_MEMBER_FUNCT();

    // protecting the containers access
    lima::AutoMutex container_mutex = containersLock();

    out_received = m_received_frames.size();
    out_complete = m_complete_frames.size();
    out_treated  = m_treated_frames.size ();
}

/************************************************************************
 * \brief get the number of treated frames in the container
 * \return the number of treated frames
 ************************************************************************/
size_t CameraFrames::getNbTreatedFrames() const
{
    DEB_MEMBER_FUNCT();

    // protecting the containers access
    lima::AutoMutex container_mutex = containersLock();

    return m_treated_frames.size();
}

/************************************************************************
 * \brief get the number of not treated frames in the containers
 * \return the number of not treated frames
 ************************************************************************/
size_t CameraFrames::getNbNotTreatedFrames() const
{
    DEB_MEMBER_FUNCT();

    // protecting the containers access
    lima::AutoMutex container_mutex = containersLock();

    return m_received_frames.size() + m_complete_frames.size();
}

/************************************************************************
 * \brief checks if there is no more frame to treat in the containers
 * \return true is there is no more frame to treat in the containers
 ************************************************************************/
bool CameraFrames::NoMoreFrameToTreat() const
{
    DEB_MEMBER_FUNCT();

    return (getNbNotTreatedFrames() == 0);
}

/************************************************************************
 * \brief clear the containers
 ************************************************************************/
void CameraFrames::clear()
{
    DEB_MEMBER_FUNCT();

    // protecting the containers access
    lima::AutoMutex container_mutex = containersLock();

    m_received_frames.clear();
    m_complete_frames.clear();
    m_treated_frames.clear ();

    m_first_frame_index       = 0LL  ;
    m_first_timestamp         = 0LL  ;
    m_is_first_frame_received = false;
}

/************************************************************************
 * \brief store first frame data to compute relative data
 * \param in_absolute_frame_index index which cames with the callback
 * \param in_absolute_timestamp timestamp which cames with the callback
 ************************************************************************/
void CameraFrames::manageFirstFrameTreatment(const uint64_t in_absolute_frame_index,
                                             const uint64_t in_absolute_timestamp  )
{
    if(!m_is_first_frame_received)
    {
        m_first_frame_index       = in_absolute_frame_index;
        m_first_timestamp         = in_absolute_timestamp  ;
        m_is_first_frame_received = true;
    }
}

/************************************************************************
 * \brief compute the relative frame index (which starts at 0)
 * \param in_absolute_frame_index index which cames with the callback
 * \return relative frame index
 ************************************************************************/
uint64_t CameraFrames::computeRelativeFrameIndex(const uint64_t in_absolute_frame_index)
{
    DEB_MEMBER_FUNCT();

    return (in_absolute_frame_index - m_first_frame_index);
}   

/************************************************************************
 * \brief compute the relative timestamp (which starts at 0)
 * \param in_absolute_timestamp timestamp which cames with the callback
 * \return relative timestamp
 ************************************************************************/
uint64_t CameraFrames::computeRelativeTimestamp(const uint64_t in_absolute_timestamp)
{
    DEB_MEMBER_FUNCT();

    return (in_absolute_timestamp - m_first_timestamp);
}   

/************************************************************************
 * \brief creates an autolock mutex for containers methods access
 ************************************************************************/
lima::AutoMutex CameraFrames::containersLock() const
{
    DEB_MEMBER_FUNCT();
    return lima::AutoMutex(m_containers_cond.mutex());
}

//========================================================================================
