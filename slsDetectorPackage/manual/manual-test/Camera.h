/*************************************************************************/
/*! 
 *  \file   Camera.h
 *  \brief  detector hardware class interface 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************/

#ifndef CAMERA_H
#define CAMERA_H

#include <string>
#include <stdint.h>
#include "CameraReceivers.h"

/**********************************************************************/
// defines the SLS slsDetectorUsers class
// Class for detector functionalities to embed the detector controls in the users custom interface e.g. EPICS, Lima etc.
class slsDetectorUsers;

namespace Detector_ns
{
    static const int SLS_GET_VALUE = -1; // special value used to call set/get sls methods into get mode

    static const std::string SLS_TRIGGER_MODE_AUTO    = "auto"   ; // there is no triggers enums in the current sls sdk
    static const std::string SLS_TRIGGER_MODE_TRIGGER = "trigger"; // there is no triggers enums in the current sls sdk

    /***********************************************************************
     * \class Camera
     * \brief Hardware control object interface
     ***********************************************************************/

    class Camera
    {
    public:
        // status values
        enum Status
        {
            Idle   , // ready to start acquisition
            Waiting, // waiting for trigger or gate signal  
            Running, // acquisition is running 
            Error  , // acquisition stopped externally, fifo full or unexpected error 
        };

        //==================================================================
        // constructor
        Camera(const std::string & in_config_file_name   ,  // complete path to the configuration file
               const long          in_frame_packet_number); // Number of packets we should get in each receiver frame

        // destructor (no need to be virtual)
        ~Camera();

        //------------------------------------------------------------------
        // acquisition management
        //------------------------------------------------------------------
        void acquisition(long        in_receiver_fifo_depth    ,
                         double      in_exposure_time_sec      ,  
                         double      in_exposure_period_sec    ,
                         double      in_delay_after_trigger_sec,
                         std::string in_trig_mode              ,
                         int64_t     in_nb_frames_per_cycle    ,
                         int64_t     in_nb_cycles              ,
                         int         in_clock_divider          );

        // Acquisition data management
        void acquisitionDataReady(const int      in_receiver_index,
                                  const uint64_t in_frame_index   ,
                                  const uint32_t in_packet_number ,
                                  const uint64_t in_timestamp     ,
                                  const char *   in_data_pointer  ,
                                  const uint32_t in_data_size     );

        //==================================================================
        // Related to commands (put & get)
        //==================================================================
        // Executes a set command
        std::string setCmd(const std::string & in_command);

        // Executes a get command
        std::string getCmd(const std::string & in_command);

    private:
        //==================================================================
        // Specifics methods management
        //==================================================================
        // inits the camera while setting the configuration file name
        void init(const std::string & in_config_file_name);

        // converts a version id to a string
        static std::string convertVersionToString(int64_t in_version);

        // cleans the shared memory used by the camera
        void cleanSharedMemory();

        // Converts a standard string to args arguments
        void convertStringToArgs(const std::string & in_command,
                                 char  * *         & out_argv  ,
                                 int               & out_argc  );
        // Releases args arguments
        void releaseArgs(char * * & in_out_argv  ,
                         int      & in_out_argc  );

        //------------------------------------------------------------------
        // status management
        //------------------------------------------------------------------
        // returns the current detector status
        Camera::Status getDetectorStatus();

    private:
        //------------------------------------------------------------------
        // camera stuff
        //------------------------------------------------------------------
        // complete config file path
        std::string m_config_file_name;

        // Number of packets we should get in each receiver frame
        uint32_t m_frame_packet_number;

        // Class for detector functionalities to embed the detector controls in the users custom interface e.g. EPICS, Lima etc.
        slsDetectorUsers * m_detector_control;

        // Controller class for detector receivers functionalities
        CameraReceivers * m_detector_receivers;
    };
}
#endif // CAMERA_H

/*************************************************************************/