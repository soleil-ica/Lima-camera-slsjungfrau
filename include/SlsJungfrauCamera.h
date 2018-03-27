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
 *  \file   SlsJungfrauCamera.h
 *  \brief  SlsJungfrau detector hardware class interface 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************/

#ifndef SLSJUNGFRAUCAMERA_H
#define SLSJUNGFRAUCAMERA_H

#include "lima/Debug.h"
#include "SlsJungfrauCompatibility.h"
#include "lima/Constants.h"
#include "lima/HwBufferMgr.h"
#include "lima/ThreadUtils.h"
#include "lima/HwSyncCtrlObj.h"
#include "SlsJungfrauCameraThread.h"

namespace lima
{
    namespace SlsJungfrau
    {
        /***********************************************************************
         * \class Camera
         * \brief Hardware control object interface
         ***********************************************************************/

        class LIBSLSJUNGFRAU_API Camera
        {
            DEB_CLASS_NAMESPC(DebModCamera, "Camera", "SlsJungfrau");
            friend class Interface;

        public:
            enum Status
            {
	            Idle, Waiting, Running, Error,
            };

            // constructor
            Camera(const std::string & in_config_file_name);

            // destructor (no need to be virtual)
            ~Camera();

            //==================================================================
            // Related to HwInterface
            //==================================================================
                //------------------------------------------------------------------
                // acquisition management
                //------------------------------------------------------------------
                // prepares the acquisition
                void prepareAcq();

                // starts the acquisition (start/snap)
                void startAcq();

                // stops the acquisition
                void stopAcq();

                //------------------------------------------------------------------
                // status management
                //------------------------------------------------------------------
                // returns the current camera status
                Status getStatus() const;

                //------------------------------------------------------------------
                // Acquired frames management
                //------------------------------------------------------------------
                // Gets the number of acquired frames
                int getNbHwAcquiredFrames() const;

            //==================================================================
            // Related to HwDetInfoCtrlObj
            //==================================================================
                //------------------------------------------------------------------
                // image size management
                //------------------------------------------------------------------
                // Gets the maximum image width
                unsigned short getMaxWidth() const;

                // Gets the maximum image height
                unsigned short getMaxHeight() const;

                //------------------------------------------------------------------
                // current image type management
                //------------------------------------------------------------------
                // gets the current image type
                ImageType getImageType() const;

                // sets the current image type
                void setImageType(ImageType in_type);

                //------------------------------------------------------------------
                // pixel size management
                //------------------------------------------------------------------
                // gets the pixel size
                void getPixelSize(double & out_x_size, double & out_y_size) const;

                //------------------------------------------------------------------
                // detector info management
                //------------------------------------------------------------------
                // gets the detector type
                std::string getDetectorType() const;

                // gets the detector model
                std::string getDetectorModel() const;

            //==================================================================
            // Related to HwSyncCtrlObj
            //==================================================================
                //------------------------------------------------------------------
                // trigger mode management
                //------------------------------------------------------------------
                // Checks the trigger mode validity
                bool checkTrigMode(TrigMode in_trig_mode) const;

                // Sets the trigger mode
                void setTrigMode(TrigMode in_mode);

                // Gets the trigger mode
                TrigMode getTrigMode() const;

                //------------------------------------------------------------------
                // exposure time management
                //------------------------------------------------------------------
                // Gets the exposure time
                double getExpTime() const;

                // Sets the exposure time
                void setExpTime(double in_exp_time);

                //------------------------------------------------------------------
                // latency time management
                //------------------------------------------------------------------
                // Gets the latency time
                double getLatencyTime() const;

                // Sets the latency time
                void setLatencyTime(double in_latency_time) const;

                //------------------------------------------------------------------
                // number of frames management
                //------------------------------------------------------------------
                // Sets the number of frames
                void setNbFrames(int in_nb_frames);

                // Gets the number of frames
                int getNbFrames() const;

                //------------------------------------------------------------------
                // valid ranges management
                //------------------------------------------------------------------
                // Gets exposure and latency ranges
                HwSyncCtrlObj::ValidRangesType getValidRanges() const;

            //==================================================================
            // Related to BufferCtrl object
            //==================================================================
            // Gets the internal buffer manager
            HwBufferCtrlObj * getBufferCtrlObj();

        private:
            friend class CameraThread;

            //------------------------------------------------------------------
            // Lima buffer control object
            //------------------------------------------------------------------
            SoftBufferCtrlObj   m_buffer_ctrl_obj;
            unsigned short    * m_frame          ;
            unsigned short    * m_pr_buffer      ;

            //------------------------------------------------------------------
            // camera stuff
            //------------------------------------------------------------------
            std::string m_config_file_name;
            Camera::Status m_state;
            double m_exposure_time;
            double m_latency_time;
            int m_nb_frames;
            int m_trigger_mode;
            long m_max_width;
            long m_max_height;
            long m_depth;
            std::string m_acq_mode_name;
            std::string m_status;
            Size m_frame_size;

            //------------------------------------------------------------------
            // main acquisition thread
            //------------------------------------------------------------------
            CameraThread m_thread;
            int m_acq_frame_nb;

            //------------------------------------------------------------------
            // mutex stuff
            //------------------------------------------------------------------
            mutable Cond m_cond;
        };
    }
}
#endif // SLSJUNGFRAUCAMERA_H

/*************************************************************************/