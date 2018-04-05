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
 *  \author C�dric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
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
#include <yat/memory/SharedPtr.h>

// defines the sls slsDetectorUsers class
// Class for detector functionalities to embed the detector controls in the users custom interface e.g. EPICS, Lima etc.
class slsDetectorUsers;

namespace lima
{
    namespace SlsJungfrau
    {
        static const int SLS_GET_VALUE = -1; // special value used to call set/get sls methods into get mode

        /***********************************************************************
         * \class Camera
         * \brief Hardware control object interface
         ***********************************************************************/

        class LIBSLSJUNGFRAU_API Camera
        {
            DEB_CLASS_NAMESPC(DebModCamera, "Camera", "SlsJungfrau");
            friend class Interface;

        public:
            // status values
            enum Status
            {
	            Idle   , // ready to start acquisition
                Waiting, // waiting for trigger or gate signal  
                Running, // acquisition is running 
                Error  , // acquisition stopped externally, fifo full or unexpected error 
            };

            // clock divider values
            enum ClockDivider
            {
                FullSpeed, HalfSpeed, QuarterSpeed, SuperSlowSpeed,
            };

            // return values of sls methods 
            enum CallResult 
            {
                OK          , // function succeeded
                FAIL        , // function failed
                FINISHED    , // acquisition finished
                FORCE_UPDATE
            };

            //==================================================================
            // constructor
            Camera();

            // destructor (no need to be virtual)
            ~Camera();

            //==================================================================
            // Specifics methods management
            //==================================================================
            // inits the camera while setting the configuration file name
            void init(const std::string & in_config_file_name);

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

                // Gets the image width
                unsigned short getWidth() const;

                // Gets the image height
                unsigned short getHeight() const;

                //------------------------------------------------------------------
                // current image type management
                //------------------------------------------------------------------
                // gets the default image type
                lima::ImageType getDefImageType() const;

                // gets the current image type
                lima::ImageType getImageType() const;

                // sets the current image type
                void setImageType(lima::ImageType in_type);

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

                // gets Module Firmware Version
                std::string getModuleFirmwareVersion() const;

                // gets Detector Firmware Version
                std::string getDetectorFirmwareVersion() const;

                 // gets Detector Software Version
                std::string getDetectorSoftwareVersion() const;

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
            //==================================================================
            // Specifics methods management
            //==================================================================
            // converts a version id to a string
            static std::string convertVersionToString(int64_t in_version);

            // cleans the shared memory used by the camera
            void cleanSharedMemory();

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
            // complete config file path
            std::string m_config_file_name;

            // Class for detector functionalities to embed the detector controls in the users custom interface e.g. EPICS, Lima etc.
            yat::SharedPtr<slsDetectorUsers> m_detector_control;

            // current bit depth
            int m_bit_depth;

            // maximum width
            int m_max_width ;

            // maximum height
            int m_max_height;

            // current width
            int m_width ;

            // current height
            int m_height;

/*          Camera::Status m_state ;
            std::string    m_status;
            double m_exposure_time;
            double m_latency_time;
            int m_nb_frames;
            int m_trigger_mode;
            std::string m_acq_mode_name;
            Size m_frame_size;*/

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