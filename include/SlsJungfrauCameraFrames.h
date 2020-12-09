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
 *  \author Cedric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
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
                             const CameraFrame & in_frame         ,
                             const char *        in_data_pointer  = NULL,
                             const uint32_t      in_data_size     = 0   );

            // tell if there is an available complete frame in the complete frames container
            bool completeAvailable(void);

            // get a reference to the first complete frame from complete frames container
            const CameraFrame & getFirstComplete(void);

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

            // store first frame data to compute relative data
            void manageFirstFrameTreatment(const uint64_t in_absolute_frame_index,
                                           const uint64_t in_absolute_timestamp  );

            // compute the relative frame index (which starts at 0)
            uint64_t computeRelativeFrameIndex(const uint64_t in_absolute_frame_index);

            // compute the relative timestamp (which starts at 0)
            uint64_t computeRelativeTimestamp(const uint64_t in_absolute_timestamp);

        private:
            // type of container which contains the frames not complete
        	typedef std::map<uint64_t, CameraFrame> ReceivedFramesContainer;

            // type of container which contains the complete frames not passed to Lima
        	typedef std::list<CameraFrame> CompleteFramesContainer;

            // type of container which contains the frames already passed to Lima newFrameReady method
        	typedef std::list<CameraFrame> TreatedFramesContainer ;

            // creates an autolock mutex for containers methods access
            lima::AutoMutex containersLock() const;

        private:
            // direct access to camera
            Camera & m_cam;

            //==================================================================
            // important : a frame should always be in one of these tree containers.
            // container which contains the frames not complete
            ReceivedFramesContainer m_received_frames; 

            // container which contains the complete frames not passed to Lima
            CompleteFramesContainer m_complete_frames; 

            // container which contains the frames already passed to Lima newFrameReady method
            TreatedFramesContainer  m_treated_frames;

            //==================================================================
            // used to protect the containers access
            // mutable keyword is used to allow const methods even if they use this class member
            mutable lima::Cond m_containers_cond;

            //==================================================================
            // true if the first frame index has been received, else false.
            bool m_is_first_frame_received;

            // the frame index in the sls data callback is not reset for each new acquisition.
            // so we need to store the first frame index to compute a relative frame index (which starts at 0)
            uint64_t m_first_frame_index;
            
            // the frame timestamp in the sls data callback is not reset for each new acquisition.
            // so we need to store the first timestamp to compute a relative timestamp (which starts at 0)
            uint64_t m_first_timestamp;
        };
    }
}
#endif // SLSJUNGFRAUCAMERAFRAMES_H

/*************************************************************************/