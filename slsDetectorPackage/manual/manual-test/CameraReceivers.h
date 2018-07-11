/********************************************************************************/
/*! 
 *  \file   CameraReceivers.h
 *  \brief  detector acquisition receivers controller class interface 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/********************************************************************************/

#ifndef CAMERARECEIVERS_H
#define CAMERARECEIVERS_H

#include <string>
#include <vector>
#include <stdint.h>
#include "CameraReceiver.h"

/**********************************************************************/
// defines the SLS slsReceiverUsers class
// Class for implementing the SLS data receiver in the users application.
// Callbacks can be defined for processing and/or saving data
class slsReceiverUsers;

namespace Detector_ns
{
    /***********************************************************************
     * \class CameraException
     ***********************************************************************/
    struct CameraException : public std::exception
    {
       std::string s;

       CameraException(std::string ss) : s(ss) {}

       ~CameraException() throw () {} // Updated

       const char* what() const throw() { return s.c_str(); }
    };

    /**********************************************************************/
    // defines the  camera class for direct access
    class Camera;

    /***********************************************************************
     * \class CameraReceivers
     * \brief used to control the sls receivers
     ***********************************************************************/

    class CameraReceivers
    {
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

        // c-array which contains elements used as user data in sls sdk callbacks
        // I prefer to use a fixed container to get fixed pointers on the elements.
        CameraReceiver * m_receivers   ;
        size_t           m_nb_receivers;
    };
}
#endif // CAMERARECEIVERS_H

/*************************************************************************/