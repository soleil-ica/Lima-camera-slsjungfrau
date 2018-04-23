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
 *  \file   SlsJungfrauCameraFrames.h
 *  \brief  SlsJungfrau detector frames manager class interface 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/********************************************************************************/

#ifndef SLSJUNGFRAUCAMERAFRAMES_H
#define SLSJUNGFRAUCAMERAFRAMES_H

#include "lima/Debug.h"
#include "lima/Constants.h"
#include "lima/HwBufferMgr.h"

#include <vector>
#include <list>

#include "SlsJungfrauCompatibility.h"
#include "SlsJungfrauCameraFrame.h"

namespace lima
{
    namespace SlsJungfrau
    {
        /**********************************************************************/
        // defines the SlsJungfrau camera class for direct access
        class Camera;

        /***********************************************************************
         * \class CameraFrames
         * \brief used to manage the sls frames
         ***********************************************************************/

        class CameraFrames
        {
            DEB_CLASS_NAMESPC(DebModCamera, "CameraFrames", "SlsJungfrau");

        public:
            //==================================================================
            // constructor
            CameraFrames(Camera & cam);

            // destructor (no need to be virtual)
            ~CameraFrames();

            //==================================================================
            // add a new received frame
            void addReceived(const int           in_receiver_index,
                             const CameraFrame & in_frame         );

            // get the first complete frame from complete frames container
            bool getFirstComplete(CameraFrame & out_frame);

            // move the first complete frame to the treated frame container
            void moveFirstCompleteToTreated();

            // get the number of frames in the containers
            void getNbFrames(size_t & out_received, 
                             size_t & out_complete,
                             size_t & out_treated ) const;

            // get the number of treated frames in the container
            size_t getNbTreatedFrames() const;

            // get the number of not treated frames in the containers
            size_t getNbNotTreatedFrames() const;

            // checks if there is no more frame to treat in the containers
            bool NoMoreFrameToTreat() const;

            // clear the containers
            void clear();

        private:
            // type of container which contains the frames not complete
        	typedef std::map<uint64_t, CameraFrame> ReceivedFramesContainer;

            // type of container which contains the complete frames not passed to Lima
        	typedef std::list<CameraFrame> CompleteFramesContainer;

            // type of container which contains the frames already passed to Lima newFrameReady method
        	typedef std::list<CameraFrame> TreatedFramesContainer ;

        private:
            // direct access to camera
            Camera & m_cam;

            // important : a frame should always be in one of these tree containers.
            // container which contains the frames not complete
            ReceivedFramesContainer m_received_frames; 

            // container which contains the complete frames not passed to Lima
            CompleteFramesContainer m_complete_frames; 

            // container which contains the frames already passed to Lima newFrameReady method
            TreatedFramesContainer  m_treated_frames;

            // used to protect the containers access
            // mutable keyword is used to allow const methods even if they use this class member
            mutable lima::Cond m_cond;
        };
    }
}
#endif // SLSJUNGFRAUCAMERAFRAMES_H

/*************************************************************************/