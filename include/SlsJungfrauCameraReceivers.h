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
 *  \file   SlsJungfrauCameraReceivers.h
 *  \brief  SlsJungfrau detector acquisition receivers controller class interface 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/********************************************************************************/

#ifndef SLSJUNGFRAUCAMERARECEIVERS_H
#define SLSJUNGFRAUCAMERARECEIVERS_H

#include "lima/Debug.h"
#include "SlsJungfrauCompatibility.h"
#include "lima/Constants.h"
#include "lima/HwBufferMgr.h"
#include <vector>

#include "SlsJungfrauCameraReceiver.h"

/**********************************************************************/
// defines the SLS slsReceiverUsers class
// Class for implementing the SLS data receiver in the users application.
// Callbacks can be defined for processing and/or saving data
class slsReceiverUsers;

namespace lima
{
    namespace SlsJungfrau
    {
        /**********************************************************************/
        // defines the SlsJungfrau camera class for direct access
        class Camera;

        /***********************************************************************
         * \class CameraReceivers
         * \brief used to control the sls receivers
         ***********************************************************************/

        class CameraReceivers
        {
            DEB_CLASS_NAMESPC(DebModCamera, "CameraReceivers", "SlsJungfrau");

        public:
            //==================================================================
            // constructor
            CameraReceivers(Camera & cam);

            // destructor (no need to be virtual)
            ~CameraReceivers();

            // inits the receivers using the configuration file name
            void init(const std::string & in_config_file_name);

        private:
            friend class CameraReceiver;

            // creates the receivers container
            void createReceivers(const std::string           & in_config_file_name,
                                 std::vector<CameraReceiver> & out_receivers      );

            // removes leading, trailing and extra spaces
            std::string trimString(const std::string & in_string) const;

            // automatically resizes the receivers container 
            void manageReceiversResize(const size_t                & in_element_index,
                                       std::vector<CameraReceiver> & in_out_receivers);

            // converts a string to an integer and checks the value.
            int convertStringToInteger(const std::string & in_value,
                                       const std::string & in_label) const;

            //------------------------------------------------------------------
            // internal callbacks management
            //------------------------------------------------------------------
            // Started acquisition management
            void startedAcquisition(int in_receiver_index);

            // Finished acquisition management
            void finishedAcquisition(int      in_receiver_index,
                                     uint64_t in_frames_nb     );

            // Acquisition data management
            void acquisitionDataReady(const int      in_receiver_index,
                                      uint64_t       in_frame_index   ,
                                      const uint32_t in_packet_number ,
                                      const uint64_t in_timestamp     ,
                                      const char *   in_data_pointer  ,
                                      const uint32_t in_data_size     );

            //------------------------------------------------------------------
            // sls sdk callbacks
            //------------------------------------------------------------------
            // Start Acquisition Call back
            static int startedAcquisitionCallBack(char     * in_file_path ,
                                                  char     * in_file_name ,
                                                  uint64_t   in_file_index,
                                                  uint32_t   in_data_size ,
                                                  void     * in_user_data );

            // Acquisition Finished Call back
            static void finishedAcquisitionCallBack(uint64_t   in_frames_nb,
                                                    void     * in_user_data);

            // Get Receiver Data Call back
            static void acquisitionDataReadyCallBack(char     * in_meta_data   ,
                                                     char     * in_data_pointer,
                                                     uint32_t   in_data_size   ,
                                                     void     * in_user_data   );

        private:
            // direct access to camera
            Camera & m_cam;

            // c-array which contains elements used as user data in sls sdk callbacks
            // I prefer to use a fixed container to get fixed pointers on the elements.
            CameraReceiver * m_receivers   ;
            size_t           m_nb_receivers;
        };
    }
}
#endif // SLSJUNGFRAUCAMERARECEIVERS_H

/*************************************************************************/