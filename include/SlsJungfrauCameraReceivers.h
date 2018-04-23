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

        /**********************************************************************/
        // pre-defines the CameraReceivers class for CameraReceiverInfo friend
        // and CameraReceiverUserData friend links
        class CameraReceivers;

        /***********************************************************************
         * \class CameraReceiverInfo
         * \brief used to control the sls receivers
         *
         * defines an info class used to store Receiver informations 
         * which were read from the configuration file.
         ***********************************************************************/
        class CameraReceiverInfo
        {
            // class CameraReceivers is a friend class of class CameraReceiverInfo
            friend class CameraReceivers;

        private :
            lima::AutoPtr<slsReceiverUsers > m_receiver  ; // sls receiver instance from sls sdk
            std::string                      m_host_name ; // receiver host name  (part of the hostname value in config file)
            int                              m_tcpip_port; // receiver tcpip port (value of rx_tcpport in config file)
        }; 

        /***********************************************************************
         * \struct CameraReceiverUserData
         * \brief used as user data in sls sdk callbacks
         *
         * allows the callbacks :
         * - to access to the CameraReceivers object
         * - to know the receiver index which is not always given in
         *   the callback data
         ***********************************************************************/
        class CameraReceiverUserData
        {
            // class CameraReceivers is a friend class of class CameraReceiverUserData
            friend class CameraReceivers;

        private :
            int                            m_receiver_index; // receiver index in m_receivers_info container
            lima::AutoPtr<CameraReceivers> m_receivers     ; // direct access to the CameraReceivers object
        };

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
            // in_detector_receivers is used to set the callback user data.
            // as the CameraReceivers in already managed by a shared pointer
            // we can not use *this* to set the callbacks.
            void init(const std::string              & in_config_file_name  ,
                      lima::AutoPtr<CameraReceivers>   in_detector_receivers);

        public :

        protected:

        private:
            friend class CameraReceiverUserData;

            // creates the receiver info container
            void createReceiversInfo(const std::string & in_config_file_name);

            // removes leading, trailing and extra spaces
            std::string trimString(const std::string & in_string) const;

            // automatically resizes the receivers informations container 
            void manageReceiversInfoResize(const size_t & in_element_index);

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
            static void acquisitionDataReadyCallBack(uint64_t   in_frame_number  ,
                                                     uint32_t   in_exp_length    ,
                                                     uint32_t   in_packet_number ,
                                                     uint64_t   in_bunch_id      ,
                                                     uint64_t   in_timestamp     ,
                                                     uint16_t   in_mod_id        ,
                                                     uint16_t   in_x_coord       ,
                                                     uint16_t   in_y_coord       ,
                                                     uint16_t   in_z_coord       ,
                                                     uint32_t   in_debug         ,
                                                     uint16_t   in_round_r_number,
                                                     uint8_t    in_det_type      ,
                                                     uint8_t    in_version       ,
                                                     char     * in_data_pointer  ,
                                                     uint32_t   in_data_size     ,
                                                     void     * in_user_data     );

        private:
            // direct access to camera
            Camera & m_cam;

            // config file receivers informations container
            std::vector<CameraReceiverInfo> m_receivers_info; 

            // c-array which contains elements used as user data in sls sdk callbacks
            // I prefer to use a fixed container to get fixed pointers on the elements.
            CameraReceiverUserData * m_receivers_user_data;
        };
    }
}
#endif // SLSJUNGFRAUCAMERARECEIVERS_H

/*************************************************************************/