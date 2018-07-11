/*************************************************************************/
/*! 
 *  \file   Camera.cpp
 *  \brief  detector hardware class implementation
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************/

#include <stdint.h>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <cmath>
#include <unistd.h>

#include "Camera.h"

#include "sls_detector_defs.h"
#include "slsDetectorUsers.h"

using namespace Detector_ns;

/************************************************************************
 * \brief constructor
 * \param in_config_file_name complete path to the configuration file
 * \param in_frame_packet_number Number of packets we should get in each receiver frame
 ************************************************************************/
Camera::Camera(const std::string & in_config_file_name   ,
               const long          in_frame_packet_number)
{
    m_detector_control    = NULL;
    m_detector_receivers  = NULL;
    m_frame_packet_number = static_cast<uint32_t>(in_frame_packet_number);

    init(in_config_file_name);
}

/************************************************************************
 * \brief destructor
 ************************************************************************/
Camera::~Camera()
{
    if(m_detector_control != NULL)
    {
        m_detector_control->setReceiverOnline(slsDetectorDefs::OFFLINE_FLAG);
        m_detector_control->setOnline(slsDetectorDefs::OFFLINE_FLAG);

        delete m_detector_control;
        m_detector_control = NULL;
    }

    if(m_detector_receivers != NULL)
    {
        delete m_detector_receivers;
        m_detector_receivers = NULL;
    }
}

/************************************************************************
 * \brief inits the camera while setting the configuration file name
 * \param in_config_file_name complete path to the configuration file
 ************************************************************************/
void Camera::init(const std::string & in_config_file_name)
{
    int bit_depth;
    int max_width ;
    int max_height;
    int width ;
    int height;
    std::string detector_type;
    std::string detector_model;
    std::string detector_firmware_version;
    std::string detector_software_version;
    std::string module_firmware_version;

    std::cout << "config file name :" << in_config_file_name << std::endl;

    int result;

    // before, cleaning the shared memory
    cleanSharedMemory();

    // initing the class attributes
    m_config_file_name = in_config_file_name;

    // creating the controller class for detector receivers functionalities
    // CameraReceivers needs direct access to camera (*this)
    m_detector_receivers = new CameraReceivers(*this);

    // creating the receivers (just one for Jungfrau)
    m_detector_receivers->init(m_config_file_name);

    // creating the detector control instance
    int id = 0;

    m_detector_control = new slsDetectorUsers(id);

    // configuration file is used to properly configure advanced settings in the shared memory
    result = m_detector_control->readConfigurationFile(m_config_file_name);

    if(result == slsDetectorDefs::FAIL)
    {
		throw CameraException("readConfigurationFile failed! Could not initialize the camera!");
    }

    // Setting the detector online
    m_detector_control->setOnline(slsDetectorDefs::ONLINE_FLAG);

    // Connecting to the receiver
    m_detector_control->setReceiverOnline(slsDetectorDefs::ONLINE_FLAG);

    // getting the bit depth of the camera
    bit_depth = m_detector_control->setBitDepth(SLS_GET_VALUE);

    // getting the maximum detector size
    result = m_detector_control->getMaximumDetectorSize(max_width, max_height);

    if(result == slsDetectorDefs::FAIL)
    {
		throw CameraException("getMaximumDetectorSize failed! Could not initialize the camera!");
    }

    // getting the detector size to be sure
    int x0;
    int y0;

    result = m_detector_control->getDetectorSize(x0, y0, width, height);

    if(result == slsDetectorDefs::FAIL)
    {
		throw CameraException("getDetectorSize failed! Could not initialize the camera!");
    }

    // disabling the file write by the camera
    m_detector_control->enableWriteToFile(slsDetectorDefs::DISABLED);

    // initing some const data
    // Module firmware version does not exist for Jungfrau
    detector_type             = m_detector_control->getDetectorDeveloper();
    detector_model            = m_detector_control->getDetectorType();
    detector_firmware_version = convertVersionToString(m_detector_control->getDetectorFirmwareVersion());
    detector_software_version = convertVersionToString(m_detector_control->getDetectorSoftwareVersion());

    // logging some versions informations
    std::cout << "Detector developer : " << detector_type << std::endl;
    std::cout << "Detector type : " << detector_model << std::endl;
    std::cout << "Detector Firmware Version : " << detector_firmware_version << std::endl;
    std::cout << "Detector Software Version : " << detector_software_version << std::endl;
    std::cout << "Bit depth : "  << bit_depth << std::endl;
    std::cout << "Max width : "  << max_width << std::endl;
    std::cout << "Max height : "  << max_height << std::endl;
    std::cout << "width : "  << width << std::endl;
    std::cout << "height : "  << height << std::endl;
}

/************************************************************************
 * \brief cleans the shared memory used by the camera
 ************************************************************************/
void Camera::cleanSharedMemory()
{
    std::string cmd = "ipcs -m | grep -E '^0x000016[0-9a-z]{2}' | "
                      "awk '{print $2}' | while read m; do ipcrm -m $m; done";

    int result = std::system(cmd.c_str());
    std::cout << "clean shared memory result : " << result << std::endl;
}

/*******************************************************************
 * \brief acquisition
 * \param in_receiver_fifo_depth Number of frames in the receiver memory
 * \param in_trig_mode SLS_TRIGGER_MODE_AUTO or SLS_TRIGGER_MODE_TRIGGER
 *******************************************************************/
void Camera::acquisition(long        in_receiver_fifo_depth    ,
                         double      in_exposure_time_sec      ,  
                         double      in_exposure_period_sec    ,
                         double      in_delay_after_trigger_sec,
                         std::string in_trig_mode              ,
                         int64_t     in_nb_frames_per_cycle    ,
                         int64_t     in_nb_cycles              ,
                         int         in_clock_divider          )
{
    std::string trig_mode_label;

    double exposure_time  ;
    double exposure_period;
    double delay_after_trigger;

    int64_t nb_frames_per_cycle;
    int64_t nb_cycles;
    int64_t nb_frames;

    int clock_divider;

    //----------------------------------------------------------------------------------------------------
    // setting the receiver fifo depth (number of frames in the receiver memory)
    // in a next sdk release, we could have a method to call.
    // At the moment, we should use the put command.
    {
        std::stringstream tempStream;
        tempStream << "rx_fifodepth " << in_receiver_fifo_depth;
        setCmd(tempStream.str());
    }

    //----------------------------------------------------------------------------------------------------
    m_detector_control->setExposureTime  (in_exposure_time_sec  , true); // in seconds
    m_detector_control->setExposurePeriod(in_exposure_period_sec, true); // in seconds
    m_detector_control->setDelayAfterTrigger(in_delay_after_trigger_sec, true); // in seconds

    exposure_time       = m_detector_control->setExposureTime  (SLS_GET_VALUE, true); // in seconds
    exposure_period     = m_detector_control->setExposurePeriod(SLS_GET_VALUE, true); // in seconds
    delay_after_trigger = m_detector_control->setDelayAfterTrigger(SLS_GET_VALUE, true); // in seconds

    //----------------------------------------------------------------------------------------------------
    // initing the number of frames per cycle and  number of cycles 
    // to avoid problems during the trigger mode change.
    m_detector_control->setNumberOfFrames(1);
    m_detector_control->setNumberOfCycles(1);

    // conversion of trigger mode label to trigger mode index
    int trigger_mode_index = slsDetectorUsers::getTimingMode(in_trig_mode);

    // apply the trigger change
    m_detector_control->setTimingMode(trigger_mode_index);

    // converting trigger mode index to trigger mode label
    trig_mode_label = slsDetectorUsers::getTimingMode(trigger_mode_index);

    // setting the number of cycles
    nb_cycles = m_detector_control->setNumberOfCycles(in_nb_cycles);

    // setting the number of frames per cycle
    nb_frames_per_cycle = m_detector_control->setNumberOfFrames(in_nb_frames_per_cycle);

    // computing the number of frames
    nb_frames = nb_cycles * nb_frames_per_cycle;

    //----------------------------------------------------------------------------------------------------
    // clock divider
    m_detector_control->setClockDivider(in_clock_divider);
    clock_divider = m_detector_control->setClockDivider(SLS_GET_VALUE);

    //----------------------------------------------------------------------------------------------------
    std::cout << "receiver fifo depth : " << in_receiver_fifo_depth << std::endl;
    std::cout << "Exposure time in seconds : " << exposure_time << std::endl;
    std::cout << "Exposure period in seconds : " << exposure_period << std::endl;
    std::cout << "Delay after trigger in seconds : " << delay_after_trigger << std::endl;
    std::cout << "Trigger mode : " << trig_mode_label << std::endl;
    std::cout << "Nb frames per cycle : " << nb_frames_per_cycle << std::endl;
    std::cout << "Nb cyles : " << nb_cycles << std::endl;
    std::cout << "Nb frames : " << nb_frames << std::endl;
    std::cout << "Clock divider : " << clock_divider << std::endl;
    std::cout << "Estimated frame rate : " << (1.0 / exposure_period) << std::endl;

    //----------------------------------------------------------------------------------------------------
    // reset the number of caught frames in the sdk
    // in the next sdk release, we will use the method int resetFramesCaughtInReceiver().
    // At the moment, we should use the put command.
    // the given value 0 is just because put requires an argument, so it can be anything.
    setCmd("resetframescaught 0");

    //----------------------------------------------------------------------------------------------------
    const unsigned int sleep_time_sec = 1; // sleep the thread in seconds

    // starting receiver listening mode

    if(m_detector_control->startReceiver() == slsDetectorDefs::FAIL)
    {
		throw CameraException("Can not start the receiver listening mode!");
    }

    // starting real time acquisition in non blocking mode
    // returns OK if all detectors are properly started, FAIL otherwise
    if(m_detector_control->startAcquisition() == slsDetectorDefs::FAIL)
    {
        m_detector_control->stopReceiver();
		throw CameraException("Can not start real time acquisition!");
    }

    for(;;)
    {
        // checking if the hardware acquisition is running
        Camera::Status status = getDetectorStatus();

        if((status != Camera::Waiting) && (status != Camera::Running))
        {
            // we stop the treatment
            break;
        }
        else
        // hardware acquisition is running, we are waiting for new frames not using the cpu during this time
        {
            sleep(sleep_time_sec); // sleep the thread in seconds
        }
    }

    // stopping receiver listening mode
    if(m_detector_control->stopReceiver() == slsDetectorDefs::FAIL)
    {
		throw CameraException("Could not stop real time acquisition!");
    }
}

/************************************************************************
 * \brief Acquisition data management
 * \param m_receiver_index receiver index
 * \param in_frame_index frame index (starts at 0)
 * \param in_packet_number number of packets caught for this frame
 * \param in_timestamp time stamp in 10MHz clock
 * \param in_data_pointer frame image pointer
 * \param in_data_size frame image size 
 ************************************************************************/
void Camera::acquisitionDataReady(const int      in_receiver_index,
                                  const uint64_t in_frame_index   ,
                                  const uint32_t in_packet_number ,
                                  const uint64_t /*in_timestamp     */,
                                  const char *   /*in_data_pointer  */,
                                  const uint32_t /*in_data_size     */)
{
    // ckecking if there is no packet lost.
    if(in_packet_number != m_frame_packet_number)
        std::cout << "Rejected Frame [ " << in_receiver_index << ", " << in_frame_index << ", " << in_packet_number << " ]" << std::endl;
}

//------------------------------------------------------------------
// status management
//------------------------------------------------------------------
/************************************************************************
 * \brief returns the current detector status
 * \return current hardware status
 ************************************************************************/
Camera::Status Camera::getDetectorStatus()
{
    Camera::Status result;

    // getting the detector status
    int status = m_detector_control->getDetectorStatus();

    // ready to start acquisition or acquisition stopped externally
    if((status == slsDetectorDefs::IDLE   ) ||
       (status == slsDetectorDefs::STOPPED))
    {
        result = Camera::Idle;
    }
    else
    // waiting for trigger or gate signal 
    if(status == slsDetectorDefs::WAITING)
    {
        result = Camera::Waiting;
    }
    else
    // acquisition is running 
    if(status == slsDetectorDefs::RUNNING)
    {
        result = Camera::Running;
    }
    else
    // fifo full or unexpected error 
    if(status == slsDetectorDefs::ERROR)
    {
        result = Camera::Error;
    }
    else
    // these states do not apply to Jungfrau
    // RUN_FINISHED : acquisition not running but data in memory 
    // TRANSMITTING : acquisition running and data in memory 
    if((status == slsDetectorDefs::RUN_FINISHED) ||
       (status == slsDetectorDefs::TRANSMITTING))
    {
		throw CameraException("Camera::getStatus failed! An unexpected state was returned!");
    }
    else
    // impossible state 
    {
		throw CameraException("Camera::getStatus failed! An unknown state was returned!");
    }

    return result;
}

/*******************************************************************
 * \brief converts a version id to a string
 * \return version in string format (uppercase & hexa)
 *******************************************************************/
std::string Camera::convertVersionToString(int64_t in_version)
{
    std::stringstream tempStream;
    tempStream << "0x" << std::uppercase << std::hex << in_version;
    return tempStream.str();
}

//==================================================================
// Related to commands (put & get)
//==================================================================
/*******************************************************************
 * \brief Converts a standard string to args arguments
 * \param in_command command in command line format
 * \param out_argv output c-strings c-array
 * \param out_argc output number of arguments of out_argv
 *******************************************************************/
void Camera::convertStringToArgs(const std::string & in_command,
                                 char  * *         & out_argv  ,
                                 int               & out_argc  )
{
    out_argv = NULL;
    out_argc = 0   ;

    // filling a string vector with the command line elements
    std::vector<std::string> elements;
    std::stringstream ss(in_command);

	while (ss) 
    {
        std::string element;
		ss >> element;

        if(element.size() > 0)
        {
            elements.push_back(element);
        }
	}

    // setting argc value
    out_argc = elements.size();

    // allocating argv array
	out_argv = new char * [out_argc];
    
    // filling argv array
	for (int element_index = 0; element_index < out_argc; element_index++)
    {
        out_argv[element_index] = new char[elements[element_index].size() + 1]; // adding the allocation of end of c-string 
        strcpy(out_argv[element_index], elements[element_index].c_str()); // copying the string including the eos
    }
}

/*******************************************************************
 * \brief Releases args arguments
 * \param in_out_argv output c-strings c-array
 * \param in_out_argc output number of arguments of out_argv
 *******************************************************************/
void Camera::releaseArgs(char * * & in_out_argv  ,
                         int      & in_out_argc  )
{
    if(in_out_argv != NULL)
    {
        // releasing the c_strings array content
        for (int element_index = 0; element_index < in_out_argc; element_index++)
        {
            delete [] in_out_argv[element_index];
        }

        // releasing the c_strings array
        delete [] in_out_argv;

        in_out_argv = NULL;
        in_out_argc = 0   ;
    }
}

/*******************************************************************
 * \brief Executes a set command
 * \param in_command command in command line format
 * \return the command result
 *******************************************************************/
std::string Camera::setCmd(const std::string & in_command)
{
    std::cout << "Camera::setCmd - execute set command:\"" << in_command << "\"" << std::endl;

    char  * *   argv  ;
    int         argc  ;
    std::string result;

    convertStringToArgs(in_command, argv, argc);

    if(argc > 0)
    {
        result = m_detector_control->putCommand(argc, argv);
    }

    releaseArgs(argv, argc);

	std::cout << "result=\"" << result << "\"" << std::endl;
    return result;
}

/*******************************************************************
 * \brief Executes a get command
 * \param in_command command in command line format
 * \return the command result
 *******************************************************************/
std::string Camera::getCmd(const std::string & in_command)
{
    std::cout << "Camera::getCmd - execute get command:\"" << in_command << "\"" << std::endl;

    char  * *   argv  ;
    int         argc  ;
    std::string result;

    convertStringToArgs(in_command, argv, argc);

    if(argc > 0)
    {
        result = m_detector_control->getCommand(argc, argv);
    }

    releaseArgs(argv, argc);

	std::cout << "result=\"" << result << "\"" << std::endl;
    return result;
}

//========================================================================================
