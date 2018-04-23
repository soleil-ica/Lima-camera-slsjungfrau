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
#include "SlsJungfrauCameraFrames.h"
#include "SlsJungfrauCameraReceivers.h"

/**********************************************************************/
// defines the SLS slsDetectorUsers class
// Class for detector functionalities to embed the detector controls in the users custom interface e.g. EPICS, Lima etc.
class slsDetectorUsers;

namespace lima
{
    namespace SlsJungfrau
    {
        static const int SLS_GET_VALUE = -1; // special value used to call set/get sls methods into get mode

        static const std::string SLS_TRIGGER_MODE_AUTO    = "auto"   ; // there is no triggers enums in the current sls sdk
        static const std::string SLS_TRIGGER_MODE_TRIGGER = "trigger"; // there is no triggers enums in the current sls sdk

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

                // Acquisition data management
                void acquisitionDataReady(const int      in_receiver_index,
                                          const uint64_t in_frame_index   ,
                                          const uint32_t in_packet_number ,
                                          const uint64_t in_timestamp     ,
                                          const char *   in_data_pointer  ,
                                          const uint32_t in_data_size     );

                //------------------------------------------------------------------
                // status management
                //------------------------------------------------------------------
                // returns the current camera status
                Camera::Status getStatus() const;

                //------------------------------------------------------------------
                // Acquired frames management
                //------------------------------------------------------------------
                // Gets the number of acquired frames
                uint64_t getNbAcquiredFrames() const;

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
                bool checkTrigMode(lima::TrigMode in_trig_mode) const;

                // Sets the trigger mode
                void setTrigMode(lima::TrigMode in_mode);

                // Gets the trigger mode
                // can not be const because internal member are updated during the call
                lima::TrigMode getTrigMode();

                //------------------------------------------------------------------
                // exposure time management
                //------------------------------------------------------------------
                // Gets the exposure time
                // can not be const because internal member are updated during the call
                double getExpTime();

                // Sets the exposure time
                void setExpTime(double in_exp_time);

                //------------------------------------------------------------------
                // latency time management
                //------------------------------------------------------------------
                // Gets the latency time
                // can not be const because internal member are updated during the call
                double getLatencyTime();

                // Sets the latency time
                void setLatencyTime(double in_latency_time);

                //------------------------------------------------------------------
                // number of frames management
                //------------------------------------------------------------------
                // Sets the number of frames
                void setNbFrames(int64_t in_nb_frames);

                // Gets the number of frames
                // can not be const because internal member are updated during the call
                int64_t getNbFrames();

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

            //==================================================================
            // Related to commands (put & get)
            //==================================================================
            // Executes a set command
            std::string setCmd(const std::string & in_command);

            // Executes a get command
            std::string getCmd(const std::string & in_command);

            //==================================================================
            // Related to specifics attributes
            //==================================================================
            // Gets the threshold energy in eV
            int getThresholdEnergy();

            // Sets the threshold energy in eV
            void setThresholdEnergy(int in_threshold_energy_eV);

            // Gets the clock divider
            lima::SlsJungfrau::Camera::ClockDivider getClockDivider();

            // Sets the clock divider
            void setClockDivider(lima::SlsJungfrau::Camera::ClockDivider in_clock_divider);

            // Gets the delay after trigger (in seconds)
            double getDelayAfterTrigger();

            // Sets the delay after trigger (in seconds)
            void setDelayAfterTrigger(double in_delay_after_trigger);

            //==================================================================
            // Related to event control object
            //==================================================================
            // Gets the Lima event control object
            HwEventCtrlObj* getEventCtrlObj();

        private:
            //==================================================================
            // Specifics methods management
            //==================================================================
            // converts a version id to a string
            static std::string convertVersionToString(int64_t in_version);

            // cleans the shared memory used by the camera
            void cleanSharedMemory();

            // Updates exposure & latency times using camera data 
            void updateTimes();

            // Updates trigger data (mode, frame, cycle) using camera data 
            void updateTriggerData();

            // Converts a standard string to args arguments
            void convertStringToArgs(const std::string & in_command,
                                     char  * *         & out_argv  ,
                                     int               & out_argc  );
            // Releases args arguments
            void releaseArgs(char * * & in_out_argv  ,
                             int      & in_out_argc  );

            // Gets the internal number of frames (for thread access)
            uint64_t getInternalNbFrames();

            // Gets the frame manager const access
            const CameraFrames & getFrameManager() const;

            // Gets the frame manager access
            CameraFrames & getFrameManager();

        private:
            friend class CameraThread; // for getFrameManager(), getInternalNbFrames() and m_buffer_ctrl_obj accesses

            //------------------------------------------------------------------
            // Lima event control object
            //------------------------------------------------------------------
            HwEventCtrlObj m_event_ctrl_obj;

            //------------------------------------------------------------------
            // Lima buffer control object
            //------------------------------------------------------------------
            SoftBufferCtrlObj   m_buffer_ctrl_obj;
//            unsigned short    * m_frame          ;
            //unsigned short    * m_pr_buffer      ;

            //------------------------------------------------------------------
            // camera stuff
            //------------------------------------------------------------------
            // complete config file path
            std::string m_config_file_name;

            // Class for detector functionalities to embed the detector controls in the users custom interface e.g. EPICS, Lima etc.
            lima::AutoPtr<slsDetectorUsers> m_detector_control;

            // Controller class for detector receivers functionalities
            lima::AutoPtr<CameraReceivers> m_detector_receivers;

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

            // exposure time
            double m_exposure_time;

            // latency time
            double m_latency_time;

            // trigger mode label from sls sdk
            std::string m_trig_mode_label;

            // trigger mode 
            lima::TrigMode m_trig_mode;

            // number of frames per cycle from sls sdk
            int64_t m_nb_frames_per_cycle;

            // number of cycle from sls sdk
            int64_t m_nb_cycles;

            // number total of frames
            int64_t m_nb_frames;

            //------------------------------------------------------------------
            // main acquisition thread
            //------------------------------------------------------------------
            CameraThread m_thread;

            //------------------------------------------------------------------
            // frames manager
            //------------------------------------------------------------------
            CameraFrames m_frames_manager;

            //------------------------------------------------------------------
            // mutex stuff
            //------------------------------------------------------------------
            mutable Cond m_cond;
        };
    }
}
#endif // SLSJUNGFRAUCAMERA_H

/*************************************************************************/