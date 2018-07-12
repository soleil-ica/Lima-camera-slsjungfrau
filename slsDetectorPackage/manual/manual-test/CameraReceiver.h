/********************************************************************************/
/*! 
 *  \file   CameraReceiver.h
 *  \brief  receiver informations class interface 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/********************************************************************************/

#ifndef CAMERARECEIVER_H
#define CAMERARECEIVER_H

#include <string>
#include <stdint.h>

/**********************************************************************/
// defines the SLS slsReceiverUsers class
// Class for implementing the SLS data receiver in the users application.
// Callbacks can be defined for processing and/or saving data
class slsReceiverUsers;

namespace Detector_ns
{
    /**********************************************************************/
    // pre-defines the CameraReceivers class for CameraReceiver friend links
    class CameraReceivers;

    /***********************************************************************
     * \class CameraReceiver
     * \brief used to store Receiver informations
     *
     * defines an info class used to store Receiver informations 
     * which were read from the configuration file.
     *
     * allows the callbacks :
     * - to access to the CameraReceivers object
     * - to know the receiver index which is not always given in
     *   the callback data
     ***********************************************************************/

    class CameraReceiver
    {
    public:
        //==================================================================
        // constructor
        CameraReceiver();

        // destructor (no need to be virtual)
        ~CameraReceiver();

        //==================================================================
        // gets the receiver
        slsReceiverUsers * getReceiver();

        // sets the receiver
        void setReceiver(slsReceiverUsers * in_receiver);

        // gets the host name
        const std::string & getHostName() const;

        // sets the host name
        void setHostName(const std::string & in_host_name);

        // gets the tcpip port
        int getTcpipPort() const;

        // sets the tcpip port
        void setTcpipPort(const int in_tcpip_port);

        // gets the access to the CameraReceivers object 
        CameraReceivers * getReceivers();

        // sets the access to the CameraReceivers object
        void setReceivers(CameraReceivers * in_receivers);

        // gets the receiver index
        int getReceiverIndex() const;

        // sets the receiver index
        void setReceiverIndex(const int in_receiver_index);
        
    private:
        slsReceiverUsers * m_receiver     ; // sls receiver instance from sls sdk

        std::string       m_host_name     ; // receiver host name  (part of the hostname value in config file)
        int               m_tcpip_port    ; // receiver tcpip port (value of rx_tcpport in config file)

        int               m_receiver_index; // receiver index in m_receivers_info container
        CameraReceivers * m_receivers     ; // direct access to the CameraReceivers object
    }; 
}
#endif // CAMERARECEIVER_H

/*************************************************************************/