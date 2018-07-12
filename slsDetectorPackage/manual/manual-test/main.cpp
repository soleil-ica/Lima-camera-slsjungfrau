//------------------------------------------------------------------------------------------------------
#include <iostream>
#include <vector>
#include <string>

#include <stdint.h>
#include <algorithm> 
#include <functional> 
#include <cctype>
#include <locale>
#include <sstream>

#include "Camera.h"
#include "regExUtils.h"

using namespace Detector_ns;

//------------------------------------------------------------------------------------------------------
// main test
//------------------------------------------------------------------------------------------------------
int main (int /*argc*/, char** /*argv[]*/)
{
    try
    {
        std::string config_file_name        = "jungfrau_nanoscopium_switch.config";
        long        frame_packet_number     = 128;
        long        receiver_fifo_depth     = 2500;
        double      exposure_time_sec       = 0.005;
        double      exposure_period_sec     = 1.0;
        double      delay_after_trigger_sec = 0.0;
        std::string trig_mode               = SLS_TRIGGER_MODE_AUTO;
        int64_t     nb_frames_per_cycle     = 10000;
        int64_t     nb_cycles               = 1;
        int         clock_divider           = 1;

        Camera cam(config_file_name, frame_packet_number);

        cam.acquisition(receiver_fifo_depth    ,
                        exposure_time_sec      ,  
                        exposure_period_sec    ,
                        delay_after_trigger_sec,
                        trig_mode              ,
                        nb_frames_per_cycle    ,
                        nb_cycles              ,
                        clock_divider          );
    }
    catch (CameraException & ex)
    {
        std::cout << "camera exception : " << ex.what() << std::endl;
    }
    catch (RegExException & ex)
    {
        std::cout << "regex exception : " << ex.what() << std::endl;
    }
    catch (...)
    {
        std::cout << "unknown exception!" << std::endl;
    }

    std::cout << "*********** ENDING ********************" << std::endl;

    return 0;
}
//------------------------------------------------------------------------------------------------------

