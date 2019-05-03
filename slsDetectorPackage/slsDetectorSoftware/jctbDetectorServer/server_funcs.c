#include "sls_detector_defs.h"
#include "sls_receiver_defs.h"
#include "server_funcs.h"
#include "server_defs.h"
#include "firmware_funcs.h"
#include "mcb_funcs.h"
#include "slow_adc.h"
#include "registers_m.h"
#include "gitInfoMoench.h"
#include "blackfin.h"

#define FIFO_DATA_REG_OFF     0x50 << MEM_MAP_SHIFT
// Global variables


int (*flist[256])(int);



//defined in the detector specific file
/* #ifdef MYTHEND */
/* const enum detectorType myDetectorType=MYTHEN; */
/* #elif GOTTHARDD */
/* const enum detectorType myDetectorType=GOTTHARD; */
/* #elif EIGERD */
/* const enum detectorType myDetectorType=EIGER; */
/* #elif PICASSOD */
/* const enum detectorType myDetectorType=PICASSO; */
/* #elif MOENCHD */
/* const enum detectorType myDetectorType=MOENCH; */
/* #else */
enum detectorType myDetectorType=GENERIC;
/* #endif */


extern int nModX;
extern int nModY;
extern int dataBytes;
extern int nSamples;
extern int dynamicRange;
extern int  storeInRAM;

extern int lockStatus;
extern char lastClientIP[INET_ADDRSTRLEN];
extern char thisClientIP[INET_ADDRSTRLEN];
extern int differentClients;

/* global variables for optimized readout */
extern unsigned int *ram_values;
char *dataretval=NULL;
int nframes, iframes, dataret;
char mess[1000]; 

int digitalTestBit = 0;

extern int withGotthard;


int adcvpp=0x4;

/** for jungfrau reinitializing macro later */
int N_CHAN=NCHAN;
int N_CHIP=NCHIP;
int N_DAC=24;
int N_ADC=NADC;
int N_PWR=5;
int N_CHANS=NCHANS;


int init_detector(int b, int checkType) {
  
  int i;
  if (mapCSP0()==FAIL) { printf("Could not map memory\n");
    exit(1);  
  }

  // defineGPIOpins();
  //checktype
  if (checkType) {
    printf("Bus test... (checktype is %d; b is %d)",checkType,b );
    for (i=0; i<1000000; i++) {
      bus_w(SET_DELAY_LSB_REG, i*100);
      bus_r(FPGA_VERSION_REG);
      if (i*100!=bus_r(SET_DELAY_LSB_REG))
	printf("ERROR: wrote 0x%x, read 0x%x\n",i*100,bus_r(SET_DELAY_LSB_REG));
    }
    printf("Finished\n");
  }else
    printf("(checktype is %d; b is %d)",checkType,b );


  //confirm the detector type
  switch ((bus_r(PCB_REV_REG) & DETECTOR_TYPE_MASK)>>DETECTOR_TYPE_OFFSET) {
  case MOENCH03_MODULE_ID:
    myDetectorType=MOENCH;
    printf("This is a MOENCH03 module %d\n",MOENCH);
    N_DAC=8;
    N_PWR=0;
    break;

  case JUNGFRAU_MODULE_ID:
    myDetectorType=JUNGFRAU;
    printf("This is a Jungfrau module %d\n Not supported: exiting!", JUNGFRAU);
    N_DAC=8;
    N_PWR=0;
    exit(1);
    break;

  case JUNGFRAU_CTB_ID:
    myDetectorType=JUNGFRAUCTB;
    printf("This is a CTB %d\n", JUNGFRAUCTB);
    N_DAC=24;
    N_PWR=5;
    break;

  default:
    myDetectorType=GENERIC;
    printf("Unknown detector type %02x\n",(bus_r(PCB_REV_REG) & DETECTOR_TYPE_MASK)>>DETECTOR_TYPE_OFFSET);
    N_DAC=8;
    N_PWR=0;
    break;

  }
  printf("Detector type is %d\n", myDetectorType);
  if (b)
    initDetector();

  //common for both control and stop server
  strcpy(mess,"dummy message");
  strcpy(lastClientIP,"none");
  strcpy(thisClientIP,"none1");
  lockStatus=0;
  // getDynamicRange();

  /* both these functions setROI and allocateRAM should go into the control server part. */

  return OK;
}


int decode_function(int file_des) {
  int fnum,n;
  int retval=FAIL;
#ifdef VERBOSE
  printf( "receive data\n");
#endif
  n = receiveDataOnly(file_des,&fnum,sizeof(fnum));
  if (n <= 0) {
#ifdef VERBOSE
    printf("ERROR reading from socket %d, %d %d\n", n, fnum, file_des);
#endif
    return FAIL;
  }
#ifdef VERBOSE
  else
    printf("size of data received %d\n",n);
#endif
  printf("calling function=%d (%s)\n", fnum, getFunctionName((enum detFuncs)fnum));
#ifdef VERBOSE
  printf( "calling function fnum = %d %x %x %x\n",fnum,(unsigned int)(flist[fnum]), (unsigned int)(flist[F_READ_REGISTER]),(unsigned int)(&read_register));
#endif
  if (fnum<0 || fnum>255)
    fnum=255;
  retval=(*flist[fnum])(file_des);
  printf(" function=%d (%s) returned: %d\n", fnum, getFunctionName((enum detFuncs)fnum), retval);
  if (retval==FAIL)
    printf( "Error executing the function = %d \n",fnum);
  return retval;
}


const char* getFunctionName(enum detFuncs func) {
	switch (func) {
	case F_EXEC_COMMAND:					return "F_EXEC_COMMAND";
	case F_GET_ERROR:						return "F_GET_ERROR";
	case F_GET_DETECTOR_TYPE:				return "F_GET_DETECTOR_TYPE";
	case F_SET_NUMBER_OF_MODULES:			return "F_SET_NUMBER_OF_MODULES";
	case F_GET_MAX_NUMBER_OF_MODULES:		return "F_GET_MAX_NUMBER_OF_MODULES";
	case F_SET_EXTERNAL_SIGNAL_FLAG:		return "F_SET_EXTERNAL_SIGNAL_FLAG";
	case F_SET_EXTERNAL_COMMUNICATION_MODE:	return "F_SET_EXTERNAL_COMMUNICATION_MODE";
	case F_GET_ID:							return "F_GET_ID";
	case F_DIGITAL_TEST:					return "F_DIGITAL_TEST";
	case F_ANALOG_TEST:						return "F_ANALOG_TEST";
	case F_ENABLE_ANALOG_OUT:				return "F_ENABLE_ANALOG_OUT";
	case F_CALIBRATION_PULSE:				return "F_CALIBRATION_PULSE";
	case F_SET_DAC:							return "F_SET_DAC";
	case F_GET_ADC:							return "F_GET_ADC";
	case F_WRITE_REGISTER:					return "F_WRITE_REGISTER";
	case F_READ_REGISTER:					return "F_READ_REGISTER";
	case F_WRITE_MEMORY:					return "F_WRITE_MEMORY";
	case F_READ_MEMORY:						return "F_READ_MEMORY";
	case F_SET_CHANNEL:						return "F_SET_CHANNEL";
	case F_GET_CHANNEL:						return "F_GET_CHANNEL";
	case F_SET_ALL_CHANNELS:				return "F_SET_ALL_CHANNELS";
	case F_SET_CHIP:						return "F_SET_CHIP";
	case F_GET_CHIP:						return "F_GET_CHIP";
	case F_SET_ALL_CHIPS:					return "F_SET_ALL_CHIPS";
	case F_SET_MODULE:						return "F_SET_MODULE";
	case F_GET_MODULE:						return "F_GET_MODULE";
	case F_SET_ALL_MODULES:					return "F_SET_ALL_MODULES";
	case F_SET_SETTINGS:					return "F_SET_SETTINGS";
	case F_GET_THRESHOLD_ENERGY:			return "F_GET_THRESHOLD_ENERGY";
	case F_SET_THRESHOLD_ENERGY:			return "F_SET_THRESHOLD_ENERGY";
	case F_START_ACQUISITION:				return "F_START_ACQUISITION";
	case F_STOP_ACQUISITION:				return "F_STOP_ACQUISITION";
	case F_START_READOUT:					return "F_START_READOUT";
	case F_GET_RUN_STATUS:					return "F_GET_RUN_STATUS";
	case F_START_AND_READ_ALL:				return "F_START_AND_READ_ALL";
	case F_READ_FRAME:						return "F_READ_FRAME";
	case F_READ_ALL:						return "F_READ_ALL";
	case F_SET_TIMER:						return "F_SET_TIMER";
	case F_GET_TIME_LEFT:					return "F_GET_TIME_LEFT";
	case F_SET_DYNAMIC_RANGE:				return "F_SET_DYNAMIC_RANGE";
	case F_SET_READOUT_FLAGS:				return "F_SET_READOUT_FLAGS";
	case F_SET_ROI:							return "F_SET_ROI";
	case F_SET_SPEED:						return "F_SET_SPEED";
	case F_EXECUTE_TRIMMING:				return "F_EXECUTE_TRIMMING";
	case F_EXIT_SERVER:						return "F_EXIT_SERVER";
	case F_LOCK_SERVER:						return "F_LOCK_SERVER";
	case F_GET_LAST_CLIENT_IP:				return "F_GET_LAST_CLIENT_IP";
	case F_SET_PORT:						return "F_SET_PORT";
	case F_UPDATE_CLIENT:					return "F_UPDATE_CLIENT";
	case F_CONFIGURE_MAC:					return "F_CONFIGURE_MAC";
	case F_LOAD_IMAGE:						return "F_LOAD_IMAGE";
	case F_SET_MASTER:						return "F_SET_MASTER";
	case F_SET_SYNCHRONIZATION_MODE:		return "F_SET_SYNCHRONIZATION_MODE";
	case F_READ_COUNTER_BLOCK:				return "F_READ_COUNTER_BLOCK";
	case F_RESET_COUNTER_BLOCK:				return "F_RESET_COUNTER_BLOCK";
	case F_CALIBRATE_PEDESTAL:				return "F_CALIBRATE_PEDESTAL";
	case F_ENABLE_TEN_GIGA:					return "F_ENABLE_TEN_GIGA";
	case F_SET_ALL_TRIMBITS:				return "F_SET_ALL_TRIMBITS";
	case F_SET_CTB_PATTERN:					return "F_SET_CTB_PATTERN";
	case F_WRITE_ADC_REG:					return "F_WRITE_ADC_REG";
	case F_SET_COUNTER_BIT:					return "F_SET_COUNTER_BIT";
	case F_PULSE_PIXEL:						return "F_PULSE_PIXEL";
	case F_PULSE_PIXEL_AND_MOVE:			return "F_PULSE_PIXEL_AND_MOVE";
	case F_PULSE_CHIP:						return "F_PULSE_CHIP";
	case F_SET_RATE_CORRECT:				return "F_SET_RATE_CORRECT";
	case F_GET_RATE_CORRECT:				return "F_GET_RATE_CORRECT";
	case F_SET_NETWORK_PARAMETER:			return "F_SET_NETWORK_PARAMETER";
	case F_PROGRAM_FPGA:					return "F_PROGRAM_FPGA";
	case F_RESET_FPGA:						return "F_RESET_FPGA";
	case F_POWER_CHIP:						return "F_POWER_CHIP";
	case F_ACTIVATE:						return "F_ACTIVATE";
	case F_PREPARE_ACQUISITION:				return "F_PREPARE_ACQUISITION";
	case F_CLEANUP_ACQUISITION:				return "F_CLEANUP_ACQUISITION";
	default:								return "Unknown Function";
	}
}


int function_table() {
  int i;
  for (i=0;i<256;i++){
    flist[i]=&M_nofunc;
  }
  flist[F_EXIT_SERVER]=&exit_server;
  flist[F_EXEC_COMMAND]=&exec_command;
  flist[F_GET_DETECTOR_TYPE]=&get_detector_type;
  flist[F_SET_NUMBER_OF_MODULES]=&set_number_of_modules;
  flist[F_GET_MAX_NUMBER_OF_MODULES]=&get_max_number_of_modules;
  flist[F_SET_EXTERNAL_SIGNAL_FLAG]=&set_external_signal_flag;
  flist[F_SET_EXTERNAL_COMMUNICATION_MODE]=&set_external_communication_mode;
  flist[F_GET_ID]=&get_id;
  flist[F_DIGITAL_TEST]=&digital_test;
  flist[F_WRITE_REGISTER]=&write_register;
  flist[F_READ_REGISTER]=&read_register;
  flist[F_SET_DAC]=&set_dac;
  flist[F_GET_ADC]=&get_adc;
  flist[F_SET_CHANNEL]=&set_channel;
  flist[F_SET_CHIP]=&set_chip;
  flist[F_SET_MODULE]=&set_module;
  flist[F_GET_CHANNEL]=&get_channel;
  flist[F_GET_CHIP]=&get_chip;
  flist[F_GET_MODULE]=&get_module;
  flist[F_GET_THRESHOLD_ENERGY]=&get_threshold_energy;
  flist[F_SET_THRESHOLD_ENERGY]=&set_threshold_energy;  
  flist[F_SET_SETTINGS]=&set_settings;
  flist[F_START_ACQUISITION]=&start_acquisition;
  flist[F_STOP_ACQUISITION]=&stop_acquisition;
  flist[F_START_READOUT]=&start_readout;
  flist[F_GET_RUN_STATUS]=&get_run_status;
  flist[F_READ_FRAME]=&read_frame;
  flist[F_READ_ALL]=&read_all;
  flist[F_START_AND_READ_ALL]=&start_and_read_all;
  flist[F_SET_TIMER]=&set_timer;
  flist[F_GET_TIME_LEFT]=&get_time_left;
  flist[F_SET_DYNAMIC_RANGE]=&set_dynamic_range;
  flist[F_SET_ROI]=&set_roi;
  flist[F_SET_SPEED]=&set_speed;
  flist[F_SET_READOUT_FLAGS]=&set_readout_flags;
  flist[F_EXECUTE_TRIMMING]=&execute_trimming;
  flist[F_LOCK_SERVER]=&lock_server;
  flist[F_SET_PORT]=&set_port;
  flist[F_GET_LAST_CLIENT_IP]=&get_last_client_ip;
  flist[F_UPDATE_CLIENT]=&update_client;
  flist[F_CONFIGURE_MAC]=&configure_mac;
  flist[F_LOAD_IMAGE]=&load_image;
  flist[F_SET_MASTER]=&set_master;
  flist[F_SET_SYNCHRONIZATION_MODE]=&set_synchronization;
  flist[F_READ_COUNTER_BLOCK]=&read_counter_block;
  flist[F_RESET_COUNTER_BLOCK]=&reset_counter_block;
  flist[F_CALIBRATE_PEDESTAL]=&calibrate_pedestal;
  flist[F_SET_CTB_PATTERN]=&set_ctb_pattern;
  flist[F_WRITE_ADC_REG]=&write_adc_register;
  flist[F_PROGRAM_FPGA]=&program_fpga;
  flist[F_POWER_CHIP]=&power_chip;
  flist[F_RESET_FPGA]=&reset_fpga;
  flist[F_ACTIVATE]=&activate;
  flist[F_PREPARE_ACQUISITION]=&prepare_acquisition;
  flist[F_CLEANUP_ACQUISITION]=&cleanup_acquisition;
  return OK;
}


int  M_nofunc(int file_des){
  
  int ret=FAIL;
  sprintf(mess,"Unrecognized Function\n");
  printf(mess);

  sendDataOnly(file_des,&ret,sizeof(ret));
  sendDataOnly(file_des,mess,sizeof(mess));
  return FAIL;
}


int exit_server(int file_des) {
  int retval=FAIL;
  sendDataOnly(file_des,&retval,sizeof(retval));
  printf("closing server.");
  sprintf(mess,"closing server");
  sendDataOnly(file_des,mess,sizeof(mess));
  return GOODBYE;
}

int exec_command(int file_des) {
  char cmd[MAX_STR_LENGTH];
  char answer[MAX_STR_LENGTH];
  int retval=OK;
  int sysret=0;
  int n=0;

 /* receive arguments */
  n = receiveDataOnly(file_des,cmd,MAX_STR_LENGTH);
  if (n < 0) {
    sprintf(mess,"Error reading from socket\n");
    retval=FAIL;
  }

  /* execute action if the arguments correctly arrived*/
  if (retval==OK) {
#ifdef VERBOSE
    printf("executing command %s\n", cmd);
#endif
    if (lockStatus==0 || differentClients==0)
      sysret=system(cmd);

    //should be replaced by popen
    if (sysret==0) {
      sprintf(answer,"Succeeded\n");
      if (lockStatus==1 && differentClients==1)
	sprintf(answer,"Detector locked by %s\n", lastClientIP);
    } else {
      sprintf(answer,"Failed\n");
      retval=FAIL;
    }
  } else {
    sprintf(answer,"Could not receive the command\n");
  }
  
  /* send answer */
  n = sendDataOnly(file_des,&retval,sizeof(retval));
  n = sendDataOnly(file_des,answer,MAX_STR_LENGTH);
  if (n < 0) {
    sprintf(mess,"Error writing to socket");
    retval=FAIL;
  }


  /*return ok/fail*/
  return retval; 
 
}



int get_detector_type(int file_des) {
  int n=0;
  enum detectorType ret;
  int retval=OK;
  
  sprintf(mess,"Can't return detector type\n");


  /* receive arguments */
  /* execute action */
  ret=JUNGFRAUCTB;//myDetectorType;

#ifdef VERBOSE
    printf("Returning detector type %d\n",ret);
#endif

  /* send answer */  
  /* send OK/failed */
  if (differentClients==1)
    retval=FORCE_UPDATE;

  n += sendDataOnly(file_des,&retval,sizeof(retval));      
  if (retval!=FAIL) {
    /* send return argument */
    n += sendDataOnly(file_des,&ret,sizeof(ret));
  } else {
    n += sendDataOnly(file_des,mess,sizeof(mess));
  }
  /*return ok/fail*/
  return retval; 
  

}


int set_number_of_modules(int file_des) {
  int n;
  int arg[2], ret=0; 
  int retval=OK;
  int dim, nm;
  
  sprintf(mess,"Can't set number of modules\n");

  /* receive arguments */
  n = receiveDataOnly(file_des,&arg,sizeof(arg));
  if (n < 0) {
    sprintf(mess,"Error reading from socket %d", n);
    retval=GOODBYE;
  }
  if (retval==OK) {
    dim=arg[0];
    nm=arg[1];

    /* execute action */
#ifdef VERBOSE
    printf("Setting the number of modules in dimension %d to %d\n",dim,nm );
#endif
    
    //if (nm!=GET_FLAG) {
      if (dim!=X && nm!=GET_FLAG) {
	retval=FAIL;
	sprintf(mess,"Can't change module number in dimension %d\n",dim);
      } else  {
	if (lockStatus==1 && differentClients==1 && nm!=GET_FLAG) {
	  sprintf(mess,"Detector locked by %s\n", lastClientIP);
	  retval=FAIL;
	} else {
	  ret=setNMod(nm);
	  if (nModX==nm || nm==GET_FLAG) {
	    retval=OK;
	    if (differentClients==1)
	      retval=FORCE_UPDATE;
	  } else
	    retval=FAIL;
	}
      }
  }
  /*} else {
    if (dim==Y) {
      ret=nModY;
    } else if (dim==X) {
      ret=setNMod(-1);
    }
  }
  */
  
  /* send answer */
  /* send OK/failed */
  n = sendDataOnly(file_des,&retval,sizeof(retval));
  if (retval!=FAIL) {
    /* send return argument */
    n += sendDataOnly(file_des,&ret,sizeof(ret));
  } else {
    n += sendDataOnly(file_des,mess,sizeof(mess));
  }
  /*return ok/fail*/
  return retval; 
  
}


int get_max_number_of_modules(int file_des) {
  int n;
  int ret; 
  int retval=OK;
  enum dimension arg;
  
  sprintf(mess,"Can't get max number of modules\n");
  /* receive arguments */
  n = receiveDataOnly(file_des,&arg,sizeof(arg));
  if (n < 0) {
    sprintf(mess,"Error reading from socket\n");
    retval=FAIL;
  }
    /* execute action */
#ifdef VERBOSE
    printf("Getting the max number of modules in dimension %d \n",arg);
#endif


    switch (arg) {
    case X:
      ret=getNModBoard();
      break;
    case Y:
      ret=NMAXMODY;
      break;
    default:
      ret=FAIL;
      retval=FAIL;
      break;
    }
#ifdef VERBOSE
    printf("Max number of module in dimension %d is %d\n",arg,ret );
#endif  



    if (differentClients==1 && retval==OK) {
      retval=FORCE_UPDATE;
    }

  /* send answer */
  /* send OK/failed */
  n = sendDataOnly(file_des,&retval,sizeof(retval));
  if (retval!=FAIL) {
    /* send return argument */
    n += sendDataOnly(file_des,&ret,sizeof(ret));
  } else {
    n += sendDataOnly(file_des,mess,sizeof(mess));
  }



  /*return ok/fail*/
  return retval; 
}


//index 0 is in gate
//index 1 is in trigger
//index 2 is out gate
//index 3 is out trigger

int set_external_signal_flag(int file_des) {
	int n;
	int arg[2];
	int ret=OK;
	int signalindex;
	enum externalSignalFlag flag, retval;

	sprintf(mess,"Can't set external signal flag\n");

	/* receive arguments */
	n = receiveDataOnly(file_des,&arg,sizeof(arg));
	if (n < 0) {
		sprintf(mess,"Error reading from socket\n");
		ret=FAIL;
	}
	retval=SIGNAL_OFF;
	if (ret==OK) {
		signalindex=arg[0];
		flag=arg[1];
		/* execute action */
		switch (flag) {
		case GET_EXTERNAL_SIGNAL_FLAG:
			retval=getExtSignal(signalindex);
			break;

		default:
			if (differentClients==0 || lockStatus==0) {
				retval=setExtSignal(signalindex,flag);
			} else {
				if (lockStatus!=0) {
					ret=FAIL;
					sprintf(mess,"Detector locked by %s\n", lastClientIP);
				}
			}

		}

#ifdef VERBOSE
		printf("Setting external signal %d to flag %d\n",signalindex,flag );
		printf("Set to flag %d\n",retval);
#endif

	} else {
		ret=FAIL;
	}

	if (ret==OK && differentClients!=0)
		ret=FORCE_UPDATE;


	/* send answer */
	/* send OK/failed */
	n = sendDataOnly(file_des,&ret,sizeof(ret));
	if (ret!=FAIL) {
		/* send return argument */
		n += sendDataOnly(file_des,&retval,sizeof(retval));
	} else {
		n += sendDataOnly(file_des,mess,sizeof(mess));
	}


	/*return ok/fail*/
	return ret;

}


int set_external_communication_mode(int file_des) {
	int n;
	enum externalCommunicationMode arg, ret=GET_EXTERNAL_COMMUNICATION_MODE;
	int retval=OK;

	sprintf(mess,"Can't set external communication mode\n");


	/* receive arguments */
	n = receiveDataOnly(file_des,&arg,sizeof(arg));
	if (n < 0) {
		sprintf(mess,"Error reading from socket\n");
		retval=FAIL;
	}
	/*
	enum externalCommunicationMode{
	  GET_EXTERNAL_COMMUNICATION_MODE,
	  AUTO,
	  TRIGGER_EXPOSURE_SERIES,
	  TRIGGER_EXPOSURE_BURST,
	  TRIGGER_READOUT,
	  TRIGGER_COINCIDENCE_WITH_INTERNAL_ENABLE,
	  GATE_FIX_NUMBER,
	  GATE_FIX_DURATION,
	  GATE_WITH_START_TRIGGER,
	  GATE_COINCIDENCE_WITH_INTERNAL_ENABLE
	};
	 */
	if (retval==OK) {
		/* execute action */

		ret=setTiming(arg);

		/*     switch(arg) { */
		/*     default: */
		/*       sprintf(mess,"The meaning of single signals should be set\n"); */
		/*       retval=FAIL; */
		/*     } */


#ifdef VERBOSE
		printf("Setting external communication mode to %d\n", arg);
#endif
	} else
		ret=FAIL;

	/* send answer */
	/* send OK/failed */
	n = sendDataOnly(file_des,&retval,sizeof(retval));
	if (retval!=FAIL) {
		/* send return argument */
		n += sendDataOnly(file_des,&ret,sizeof(ret));
	} else {
		n += sendDataOnly(file_des,mess,sizeof(mess));
	}

	/*return ok/fail*/
	return retval;
}



int get_id(int file_des) {
  // sends back 64 bits!
  int64_t retval=-1;
  int ret=OK;
  int n=0;
  enum idMode arg;
  
  sprintf(mess,"Can't return id\n");

  /* receive arguments */
  n = receiveDataOnly(file_des,&arg,sizeof(arg));
  if (n < 0) {
    sprintf(mess,"Error reading from socket\n");
    ret=FAIL;
  }

#ifdef VERBOSE
      printf("Getting id %d\n", arg);
#endif  

  switch (arg) {
  case DETECTOR_SERIAL_NUMBER:
    retval=getDetectorNumber();
    break;
  case DETECTOR_FIRMWARE_VERSION:
    retval=getFirmwareSVNVersion();
    retval=(retval <<32) | getFirmwareVersion();
    break;
  case DETECTOR_SOFTWARE_VERSION:
	retval= GITREV;
	retval= (retval <<32) | GITDATE;
    break;
  default:
    printf("Required unknown id %d \n", arg);
    ret=FAIL;
    retval=FAIL;
    break;
  }
  
#ifdef VERBOSE
  printf("Id is %llx\n", retval);
#endif  
  
  if (differentClients==1)
    ret=FORCE_UPDATE;

  /* send answer */
  /* send OK/failed */
  n = sendDataOnly(file_des,&ret,sizeof(ret));
  if (ret!=FAIL) {
    /* send return argument */
    n += sendDataOnly(file_des,&retval,sizeof(retval));
  } else {
    n += sendDataOnly(file_des,mess,sizeof(mess));
  }

  /*return ok/fail*/
  return ret; 

}

int digital_test(int file_des) {

  int retval;
  int ret=OK;
  int imod=-1;
  int n=0;
  int ibit=0;
  int ow;
  int ival;
  enum digitalTestMode arg; 
  
  sprintf(mess,"Can't send digital test\n");

  n = receiveDataOnly(file_des,&arg,sizeof(arg));
  if (n < 0) {
    sprintf(mess,"Error reading from socket\n");
    ret=FAIL;
  }

#ifdef VERBOSE
  printf("Digital test mode %d\n",arg );
#endif  

  switch (arg) {
  case  CHIP_TEST:
    n = receiveDataOnly(file_des,&imod,sizeof(imod));
    if (n < 0) {
      sprintf(mess,"Error reading from socket\n");
      retval=FAIL;
    }
#ifdef VERBOSE
      printf("of module %d\n", imod);
#endif  
      retval=0;

    break;
  case MODULE_FIRMWARE_TEST:
    retval=0x2;
    break;
  case DETECTOR_FIRMWARE_TEST:
    retval=testFpga();
    break;
  case DETECTOR_MEMORY_TEST:
    ret=testRAM();
    break;
  case DETECTOR_BUS_TEST:
    retval=testBus();
      break;
  case DETECTOR_SOFTWARE_TEST:
    retval=testFpga();
    break;
  case DIGITAL_BIT_TEST:
	  n = receiveDataOnly(file_des,&ival,sizeof(ival));
	  if (n < 0) {
		  sprintf(mess,"Error reading from socket\n");
		  retval=FAIL;
	  }
#ifdef VERBOSE
	  printf("with value %d\n", ival);
#endif
	  if (differentClients==1 && lockStatus==1) {
		  ret=FAIL;
		  sprintf(mess,"Detector locked by %s\n",lastClientIP);
		  break;
	  }
	  digitalTestBit = ival;
	  retval=digitalTestBit;
	  break;
  default:
    printf("Unknown digital test required %d\n",arg);
    ret=FAIL;
    retval=FAIL;
    break;
  }
  
#ifdef VERBOSE
  printf("digital test result is 0x%x\n", retval);
#endif  
  //Always returns force update such that the dynamic range is always updated on the client

  // if (differentClients==1 && ret==OK)
  ret=FORCE_UPDATE;

  /* send answer */
  /* send OK/failed */
  n = sendDataOnly(file_des,&ret,sizeof(ret));
  if (ret!=FAIL) {
    /* send return argument */
    n += sendDataOnly(file_des,&retval,sizeof(retval));
  } else {
    n += sendDataOnly(file_des,mess,sizeof(mess));
  }

  /*return ok/fail*/
  return ret; 

}

int write_register(int file_des) {

  int retval;
  int ret=OK;
  int arg[2]; 
  int addr, val;
  int n;
  u_int32_t address;

  sprintf(mess,"Can't write to register\n");

  n = receiveDataOnly(file_des,arg,sizeof(arg));
  if (n < 0) {
    sprintf(mess,"Error reading from socket\n");
    ret=FAIL;
  }
  addr=arg[0];
  val=arg[1];

#ifdef VERBOSE
  printf("writing to register 0x%x data 0x%x\n", addr, val);
#endif  

  if (differentClients==1 && lockStatus==1) {
    ret=FAIL;
    sprintf(mess,"Detector locked by %s\n",lastClientIP);    
  }


  if(ret!=FAIL){
    address=(addr << MEM_MAP_SHIFT);
    if((address==FIFO_DATA_REG_OFF)||(address==CONTROL_REG))
    	ret = bus_w16(address,val);
    else
    	ret=bus_w(address,val);
    if(ret==OK){
    	if((address==FIFO_DATA_REG_OFF)||(address==CONTROL_REG))
        	retval=bus_r16(address);
        else
        	retval=bus_r(address);
    }
  }
  

#ifdef VERBOSE
  printf("Data set to 0x%x\n",  retval);
#endif  
  if (retval==val) {
    ret=OK;
    if (differentClients)
      ret=FORCE_UPDATE;
  } else {
    ret=FAIL;
    sprintf(mess,"Writing to register 0x%x failed: wrote 0x%x but read 0x%x\n", addr, val, retval);
  }

  /* send answer */
  /* send OK/failed */
  n = sendDataOnly(file_des,&ret,sizeof(ret));
  if (ret!=FAIL) {
    /* send return argument */
    n += sendDataOnly(file_des,&retval,sizeof(retval));
  } else {
    n += sendDataOnly(file_des,mess,sizeof(mess));
  }

  /*return ok/fail*/
  return ret; 

}

int read_register(int file_des) {

  int retval;
  int ret=OK;
  int arg; 
  int addr;
  int n;
  u_int32_t address;

  sprintf(mess,"Can't read register\n");

  n = receiveDataOnly(file_des,&arg,sizeof(arg));
  if (n < 0) {
    sprintf(mess,"Error reading from socket\n");
    ret=FAIL;
  }
  addr=arg;

 

  //#ifdef VERBOSE
  printf("reading  register 0x%x\n", addr);
  //#endif

  if(ret!=FAIL){
	  address=(addr << MEM_MAP_SHIFT);
	  if((address==FIFO_DATA_REG_OFF)||(address==CONTROL_REG))
		  retval=bus_r16(address);
	  else
		  retval=bus_r(address);
  }



#ifdef VERBOSE
  printf("Returned value 0x%x\n",  retval);
#endif  
  if (ret==FAIL) {
    ret=FAIL;
    printf("Reading register 0x%x failed\n", addr);
  } else if (differentClients)
    ret=FORCE_UPDATE;


  /* send answer */
  /* send OK/failed */
  n = sendDataOnly(file_des,&ret,sizeof(ret));
  if (ret!=FAIL) {
    /* send return argument */
    n += sendDataOnly(file_des,&retval,sizeof(retval));
  } else {
    n += sendDataOnly(file_des,mess,sizeof(mess));
  }

  /*return ok/fail*/
  return ret; 

}

int set_dac(int file_des) {
	//default:all mods
	int retval, retval1;
	int ret=OK;
	int arg[3];
	enum dacIndex ind;
	int imod;
	int n;
	int val;
	int mV=0;
	int v;
	sprintf(mess,"Can't set DAC\n");

	n = receiveDataOnly(file_des,arg,sizeof(arg));
	if (n < 0) {
		sprintf(mess,"Error reading from socket\n");
		ret=FAIL;
	}
	ind=arg[0];
	
	imod=arg[1];
	
	mV=arg[2];
	
	if (mV)
	  printf("DAC will be set in mV %d!\n",mV);
	else
	  printf("DAC will be set in DACu! %d\n", mV);
	  
	n = receiveDataOnly(file_des,&val,sizeof(val));
	if (n < 0) {
		sprintf(mess,"Error reading from socket\n");
		ret=FAIL;
	}

	//#ifdef VERBOSE
	printf("Setting DAC %d of module %d to %d , mode %d\n", ind, imod, val, mV);
	//#endif 

	if (imod>=getNModBoard())
		ret=FAIL;
	if (imod<0)
		imod=ALLMOD;





	if (ret==OK) {
		if (differentClients==1 && lockStatus==1) {
			ret=FAIL;
			sprintf(mess,"Detector locked by %s\n",lastClientIP);
		} else{		  
		  if (ind<N_DAC-N_PWR) {
		    
		    if (mV) {
		      
		      v=val;
		      
		      if (val>2500) 
			val=-1;
		      printf("%d mV is ",val);
		      if (val>0)
			val=val/2500*4095;
		      printf("%d DACu\n", val);
		    } else {
		      v=val*2500/4095;
		      if (val>4095) {
			val=-1;
		      }
		    }
		    
		    if (vLimitCompliant(v))
		      retval=setDac(ind,val); // in DACu


		  } else {
		    switch (ind) {
		    case ADC_VPP:
		      
		      printf("Setting ADC VPP to %d\n",val);
		      if (val>4 || val<0)
			printf("Cannot set ADC VPP to %d\n",val);
		      else {
			writeADC(0x18,val);
			adcvpp=val;
		      }
		      retval=adcvpp;;
		      break;

		    case HV_NEW:
		      retval=initHighVoltage(val,imod);//ByModule
		      break;
		      
		    case V_POWER_A:
		    case V_POWER_B:
		    case V_POWER_C:
		    case V_POWER_D:
		    case V_POWER_IO:
		    case V_POWER_CHIP:
		      if (mV) {
			if (vLimitCompliant(val))
			  retval=setPower(ind,val);
			else
			  printf("********power %d exceeds voltage limits", ind);
			  
		      } else
			printf("********power %d should be set in mV instead od DACu", ind);
		      break;
		      
		    case V_LIMIT:
		      if (mV) {
			retval=setPower(ind,val);
		      } else
			printf("********power %d should be set in mV instead od DACu", ind);
		      break;
		      
		    default:
		      printf("**********No dac with index %d\n",ind);
		      printf("**********%d %d\n",N_DAC,N_PWR);
		      ret=FAIL;
		    }
		    
		  }
		}
	}


	if(ret==OK){
	  if (ind<N_DAC-N_PWR) {	
	    if (mV) {
	      printf("%d DACu is ",retval);
	      retval1=2500*retval/4095;
	      printf("%d mV \n",retval1);
	    } else
	      retval1=retval;
	  } else 
	    retval1=retval;
	}

	//#ifdef VERBOSE
	printf("DAC set to %d mV\n",  retval);
	//#endif  
	
	if(ret==FAIL)
	  printf("Setting dac %d of module %d: wrote %d but read %d\n", ind, imod, val, retval);
	else{
	  if (differentClients)
	    ret=FORCE_UPDATE;

	}
	
	
	/* send answer */
	/* send OK/failed */
	n = sendDataOnly(file_des,&ret,sizeof(ret));
	if (ret!=FAIL) {
	  /* send return argument */
	  n += sendDataOnly(file_des,&retval,sizeof(retval));
	  n += sendDataOnly(file_des,&retval1,sizeof(retval1));
	} else {
	  n += sendDataOnly(file_des,mess,sizeof(mess));
	}


	/*return ok/fail*/
	return ret;


}



int get_adc(int file_des) {
	//default: mod 0
	int retval;
	int ret=OK;
	int arg[2];
	enum dacIndex ind;
	int imod;
	int n;
	int idac=0;
	int ipwr=-1;

	sprintf(mess,"Can't read ADC\n");


	n = receiveDataOnly(file_des,arg,sizeof(arg));
	if (n < 0) {
		sprintf(mess,"Error reading from socket\n");
		ret=FAIL;
	}
	ind=arg[0];
	imod=arg[1];

#ifdef VERBOSE
	printf("Getting ADC %d of module %d\n", ind, imod);
#endif

	if (imod>=getNModBoard() || imod<0)
		ret=FAIL;

	//#ifdef MCB_FUNCS
	switch (ind) {
	case TEMPERATURE_FPGA:
		idac=TEMP_FPGA;
		break;
	case TEMPERATURE_ADC:
		idac=TEMP_ADC;
		break;

	case V_POWER_D:
	  idac++;
	case V_POWER_C:
	  idac++;
	case V_POWER_B:
	  idac++;
	case V_POWER_A:
	  idac++;
	case V_POWER_IO:
	  idac+=100;
	  break;

	case I_POWER_D:
	  idac++;
	case I_POWER_C:
	  idac++;
	case I_POWER_B:
	  idac++;
	case I_POWER_A:
	  idac++;
	case I_POWER_IO:
	  idac+=200;
	  break;
	  
	default:
	  //  printf("Unknown DAC index %d\n",ind);
	  sprintf(mess,"Unknown DAC index %d\n",ind);
	  ret=FAIL;
	  break;
	}
	printf("DAC index %d (%d)\n",idac,ind);

	if (ret==OK) {


	  if (idac>=200)
	    retval=getCurrent(idac-200);
	  else if (idac>=100)
	    retval=getVoltage(idac-100);
	  else
	    retval=getTemperature(idac);


	}else if (ind>=1000) {
	  retval=readSlowADC(ind-1000);
	  if (retval>=0) {
	    ret=OK;
	  }
	  
	}
	  

	//#endif

#ifdef VERBOSE
	printf("ADC is %d V\n",  retval);
#endif  
	if (ret==FAIL) {
		printf("Getting adc %d of module %d failed\n", ind, imod);
	}

	if (differentClients)
		ret=FORCE_UPDATE;

	/* send answer */
	/* send OK/failed */
	n = sendDataOnly(file_des,&ret,sizeof(ret));
	if (ret!=FAIL) {
		/* send return argument */
		n += sendDataOnly(file_des,&retval,sizeof(retval));
	} else {
		n += sendDataOnly(file_des,mess,sizeof(mess));
	}

	/*return ok/fail*/
	return ret;

}

int set_channel(int file_des) {
  int ret=OK;
  sls_detector_channel myChan;
  int retval;
  int n;
  

  sprintf(mess,"Can't set channel\n");

#ifdef VERBOSE
  printf("Setting channel\n");
#endif
  ret=receiveChannel(file_des, &myChan);
  if (ret>=0)
    ret=OK;
  else
    ret=FAIL;
#ifdef VERBOSE
  printf("channel number is %d, chip number is %d, module number is %d, register is %lld\n", myChan.chan,myChan.chip, myChan.module, myChan.reg);
#endif

  if (ret==OK) {
    if (myChan.module>=getNModBoard()) 
      ret=FAIL;
    if (myChan.chip>=N_CHIP)
      ret=FAIL;
    if (myChan.chan>=N_CHAN)
      ret=FAIL;
    if (myChan.module<0)
      myChan.module=ALLMOD;
  }

 
  if (ret==OK) {
    if (differentClients==1 && lockStatus==1) {
      ret=FAIL;
      sprintf(mess,"Detector locked by %s\n",lastClientIP);  
    } else {

    }
  }
  /* Maybe this is done inside the initialization funcs */
  //copyChannel(detectorChans[myChan.module][myChan.chip]+(myChan.chan), &myChan);
  


  if (differentClients==1 && ret==OK)
    ret=FORCE_UPDATE;

  /* send answer */
  /* send OK/failed */
  n = sendDataOnly(file_des,&ret,sizeof(ret));
  if (ret!=FAIL) {
    /* send return argument */
    n += sendDataOnly(file_des,&retval,sizeof(retval));
  } else {
    n += sendDataOnly(file_des,mess,sizeof(mess));
  }


  /*return ok/fail*/
  return ret; 
  
}




int get_channel(int file_des) {

  int ret=OK;
  sls_detector_channel retval;

  int arg[3];
  int ichan, ichip, imod;
  int n;
  
  sprintf(mess,"Can't get channel\n");



  n = receiveDataOnly(file_des,arg,sizeof(arg));
  if (n < 0) {
    sprintf(mess,"Error reading from socket\n");
    ret=FAIL;
  }
  ichan=arg[0];
  ichip=arg[1];
  imod=arg[2];
 
  if (ret==OK) {
    ret=FAIL;
    if (imod>=0 && imod<getNModBoard()) {
      ret=OK;
    }
  }  
  if (ret==OK) {
    ret=FAIL;
    if (ichip>=0 && ichip<N_CHIP) {
      ret=OK;
    }
  }   
  if (ret==OK) {
    ret=FAIL;
    if (ichan>=0 && ichan<N_CHAN) {
      ret=OK;
    }
  }   


  if (ret==OK) {
    if (differentClients && ret==OK)
      ret=FORCE_UPDATE;
  } 

#ifdef VERBOSE
  printf("Returning channel %d %d %d, 0x%llx\n", ichan, ichip, imod, (retval.reg));
#endif 

  /* send answer */
  /* send OK/failed */
  n = sendDataOnly(file_des,&ret,sizeof(ret));
  if (ret!=FAIL) {
    /* send return argument */
    ret=sendChannel(file_des, &retval);
  } else {
    n += sendDataOnly(file_des,mess,sizeof(mess));
  }


  
  /*return ok/fail*/
  return ret; 


}


int set_chip(int file_des) {

  sls_detector_chip myChip;
  int ch[N_CHAN];
  int n, retval;
  int ret=OK;
  

  myChip.nchan=N_CHAN;
  myChip.chanregs=ch;





#ifdef VERBOSE
  printf("Setting chip\n");
#endif
  ret=receiveChip(file_des, &myChip);
#ifdef VERBOSE
  printf("Chip received\n");
#endif
  if (ret>=0)
    ret=OK;
  else
    ret=FAIL;
#ifdef VERBOSE
  printf("chip number is %d, module number is %d, register is %d, nchan %d\n",myChip.chip, myChip.module, myChip.reg, myChip.nchan);
#endif
  
  if (ret==OK) {
    if (myChip.module>=getNModBoard()) 
      ret=FAIL;
    if  (myChip.module<0) 
      myChip.module=ALLMOD;
    if (myChip.chip>=N_CHIP)
      ret=FAIL;
  }
    if (differentClients==1 && lockStatus==1) {
      ret=FAIL;
      sprintf(mess,"Detector locked by %s\n",lastClientIP);  
    } else {
      ;
    }
  /* Maybe this is done inside the initialization funcs */
  //copyChip(detectorChips[myChip.module]+(myChip.chip), &myChip);

    if (differentClients && ret==OK)
      ret=FORCE_UPDATE;
  /* send answer */
  /* send OK/failed */
  n = sendDataOnly(file_des,&ret,sizeof(ret));
  if (ret!=FAIL) {
    /* send return argument */
    n += sendDataOnly(file_des,&retval,sizeof(retval));
  } else {
    n += sendDataOnly(file_des,mess,sizeof(mess));
  }


  return ret;
}

int get_chip(int file_des) {

  
  int ret=OK;
  sls_detector_chip retval;
  int arg[2];
  int  ichip, imod;
  int n;
  


  n = receiveDataOnly(file_des,arg,sizeof(arg));
  if (n < 0) {
    sprintf(mess,"Error reading from socket\n");
    ret=FAIL;
  }
  ichip=arg[0];
  imod=arg[1];
 if (ret==OK) {
    ret=FAIL;
    if (imod>=0 && imod<getNModBoard()) {
      ret=OK;
    }
  }  
  if (ret==OK) {
    ret=FAIL;
    if (ichip>=0 && ichip<N_CHIP) {
      ret=OK;
    }
  }   
 

 
  if (ret==OK) {
    if (differentClients && ret==OK)
      ret=FORCE_UPDATE;
  } 

#ifdef VERBOSE
  printf("Returning chip %d %d\n",  ichip, imod);
#endif 


  /* send answer */
  /* send OK/failed */
  n = sendDataOnly(file_des,&ret,sizeof(ret));
  if (ret!=FAIL) {
    /* send return argument */  
    ret=sendChip(file_des, &retval);
  } else {
    n += sendDataOnly(file_des,mess,sizeof(mess));
  }


  
  /*return ok/fail*/
  return ret; 


}
int set_module(int file_des) {

  int retval, n;
  int ret=OK;
  int dr;
  sls_detector_module myModule;
  int *myDac=malloc(N_DAC*sizeof(int));
  int *myAdc=malloc(N_ADC*sizeof(int));
  int *myChip=NULL;
  int *myChan=NULL;
  /*not required for jungfrau. so save memory*/
  if(myDetectorType != JUNGFRAU){
	  myChip=malloc(N_CHIP*sizeof(int));
	  myChan=malloc(N_CHIP*N_CHAN*sizeof(int));
  }

  dr=setDynamicRange(-1); /* move this down to after initialization?*/

  //initialize myModule values
  if (myDac)
    myModule.dacs=myDac;
  else {
    sprintf(mess,"could not allocate dacs\n");
    ret=FAIL;
  }
  if (myAdc)
	  myModule.adcs=myAdc;
	else {
	  sprintf(mess,"could not allocate adcs\n");
	  ret=FAIL;
  }

  myModule.chipregs=NULL;
  myModule.chanregs=NULL;
  /*not required for jungfrau. so save memory*/
  if(myDetectorType != JUNGFRAU){
	if (myChip)
	  myModule.chipregs=myChip;
	else {
	  sprintf(mess,"could not allocate chips\n");
	  ret=FAIL;
	}
	if (myChan)
	  myModule.chanregs=myChan;
	else {
	  sprintf(mess,"could not allocate chans\n");
	  ret=FAIL;
	}
}

  myModule.ndac=N_DAC;
  myModule.nchip=N_CHIP;
  myModule.nchan=N_CHAN*N_CHIP;
  myModule.nadc=N_ADC;
  
#ifdef VERBOSE
  printf("Setting module\n");
#endif

  if(myDetectorType != JUNGFRAU)
	  ret=receiveModuleGeneral(file_des, &myModule, 1); //1 is to receive everything
  else
	  ret=receiveModuleGeneral(file_des, &myModule, 0); //0 is to receive partially (without trimbits etc.)

  if (ret>=0)
    ret=OK;
  else
    ret=FAIL;


#ifdef VERBOSE
  printf("module number is %d,register is %d, nchan %d, nchip %d, ndac %d, nadc %d, gain %f, offset %f\n",myModule.module, myModule.reg, myModule.nchan, myModule.nchip, myModule.ndac,  myModule.nadc, myModule.gain,myModule.offset);
#endif
  
  if (ret==OK) {
	  if (myModule.module>=getNModBoard()) {
      ret=FAIL;
      printf("Module number is too large %d\n",myModule.module);
    }
    if (myModule.module<0) 
      myModule.module=ALLMOD;
  }
    
  if (ret==OK) {
    if (differentClients==1 && lockStatus==1) {
      ret=FAIL;
      sprintf(mess,"Detector locked by %s\n",lastClientIP);  
    } else {
    }
  }

  if (differentClients==1 && ret==OK)
    ret=FORCE_UPDATE;

  /* Maybe this is done inside the initialization funcs */
  //copyChip(detectorChips[myChip.module]+(myChip.chip), &myChip);

  /* send answer */
  /* send OK/failed */
  n = sendDataOnly(file_des,&ret,sizeof(ret));
  if (ret!=FAIL) {
    /* send return argument */
    n += sendDataOnly(file_des,&retval,sizeof(retval));
  } else {
    n += sendDataOnly(file_des,mess,sizeof(mess));
  }

  free(myDac);
  if(myAdc != NULL) 	free(myAdc);
  if(myChip != NULL) 	free(myChip);
  if(myChan != NULL) 	free(myChan);


  //setDynamicRange(dr);  always 16 commented out

  return ret;
}




int get_module(int file_des) {

  int ret=OK;
  int arg;
  int   imod;
  int n;
  sls_detector_module myModule;
  int *myDac=malloc(N_DAC*sizeof(int));
  int *myChip=NULL;
  int *myChan=NULL;
  int *myAdc=NULL;
  
  /*not required for jungfrau. so save memory*/
  if(myDetectorType != JUNGFRAU){
	myChip=malloc(N_CHIP*sizeof(int));
	myChan=malloc(N_CHIP*N_CHAN*sizeof(int));
	myAdc=malloc(N_ADC*sizeof(int));
  }


  if (myDac)
    myModule.dacs=myDac;
  else {
    sprintf(mess,"could not allocate dacs\n");
    ret=FAIL;
  }


  myModule.adcs=NULL;
  myModule.chipregs=NULL;
  myModule.chanregs=NULL;
  /*not required for jungfrau. so save memory*/
  if(myDetectorType != JUNGFRAU){
    if (myAdc)
      myModule.adcs=myAdc;
    else {
      sprintf(mess,"could not allocate adcs\n");
     ret=FAIL;
    }
    if (myChip)
      myModule.chipregs=myChip;
    else {
      sprintf(mess,"could not allocate chips\n");
      ret=FAIL;
    }
    if (myChan)
      myModule.chanregs=myChan;
    else {
      sprintf(mess,"could not allocate chans\n");
      ret=FAIL;
    }
  }

  myModule.ndac=N_DAC;
  myModule.nchip=N_CHIP;
  myModule.nchan=N_CHAN*N_CHIP;
  myModule.nadc=N_ADC;
  


  n = receiveDataOnly(file_des,&arg,sizeof(arg));
  if (n < 0) {
    sprintf(mess,"Error reading from socket\n");
    ret=FAIL;
  }
  imod=arg;

  if (ret==OK) {
   ret=FAIL;
   if (imod>=0 && imod<getNModBoard()) {
     ret=OK;
     myModule.module=imod;

#ifdef VERBOSE
     printf("Returning module %d of register %x\n",  imod, myModule.reg);
#endif 
    }
 } 

  if (differentClients==1 && ret==OK)
    ret=FORCE_UPDATE;
 
  /* send answer */
  /* send OK/failed */
  n = sendDataOnly(file_des,&ret,sizeof(ret));
  if (ret!=FAIL) {
    /* send return argument */  
	if(myDetectorType != JUNGFRAU)
	  ret=sendModuleGeneral(file_des, &myModule,1);  //1 is to send everything
	else
      ret=sendModuleGeneral(file_des, &myModule,0);  //0 is to send partially (without trimbits etc.)
  } else {
    n += sendDataOnly(file_des,mess,sizeof(mess));
  }

  free(myDac);
  if(myChip != NULL) 	free(myChip);
  if(myChan != NULL) 	free(myChan);
  if(myAdc != NULL) 	free(myAdc);


  /*return ok/fail*/
  return ret; 

}


int get_threshold_energy(int file_des) { 
  int ret=FAIL;
  int n;
  int  imod;

  strcpy(mess,"cannot set threshold for moench");

  n = receiveDataOnly(file_des,&imod,sizeof(imod));
  if (n < 0)
    sprintf(mess,"Error reading from socket\n");


  /* send answer */
  n = sendDataOnly(file_des,&ret,sizeof(ret));
  n += sendDataOnly(file_des,mess,sizeof(mess));

  /*return ok/fail*/
  return OK;

}
 
int set_threshold_energy(int file_des) { 
  int ret=FAIL;
  int arg[3];
  int n;

  strcpy(mess,"cannot set threshold for moench");

  n = receiveDataOnly(file_des,&arg,sizeof(arg));
  if (n < 0)
    sprintf(mess,"Error reading from socket\n");


  /* send answer */
  n = sendDataOnly(file_des,&ret,sizeof(ret));
  n += sendDataOnly(file_des,mess,sizeof(mess));

  /*return ok/fail*/
  return OK;

}


 
int set_settings(int file_des) {

  int retval;
  int ret=OK;
  int arg[2];
  int n;
  int  imod;
  enum detectorSettings isett;


  n = receiveDataOnly(file_des,&arg,sizeof(arg));
  if (n < 0) {
    sprintf(mess,"Error reading from socket\n");
    ret=FAIL;
  }
  imod=arg[1];
  isett=arg[0];
  

#ifdef VERBOSE
  printf("Changing settings of module %d to %d\n", imod,  isett);
#endif 

  if (differentClients==1 && lockStatus==1 && arg[0]!=GET_SETTINGS) {
    ret=FAIL;
    sprintf(mess,"Detector locked by %s\n",lastClientIP);  
  } else {
/* #ifdef MCB_FUNCS */
/*     retval=setSettings(arg[0],imod); */
/* #endif */
#ifdef VERBOSE
    printf("Settings changed to %d\n",retval);
#endif  
    
    if (retval==isett || isett<0) {
      ret=OK;
    } else {
      ret=FAIL;
      printf("Changing settings of module %d: wrote %d but read %d\n", imod, isett, retval);
    }
    
  }
  if (ret==OK && differentClients==1)
    ret=FORCE_UPDATE;

  /* send answer */
  n = sendDataOnly(file_des,&ret,sizeof(ret));
  if (ret==FAIL) {
    n += sendDataOnly(file_des,mess,sizeof(mess));
  } else
    n += sendDataOnly(file_des,&retval,sizeof(retval));
    
 
  
  return ret; 


}

int start_acquisition(int file_des) {

  int ret=OK;
  int n;
  

  sprintf(mess,"can't start acquisition\n");

#ifdef VERBOSE
  printf("Starting acquisition\n");
#endif
  
  if (differentClients==1 && lockStatus==1) {
    ret=FAIL;
    sprintf(mess,"Detector locked by %s\n",lastClientIP);  
  } else {
    ret=startStateMachine();
  }
  if (ret==FAIL)
    sprintf(mess,"Start acquisition failed\n");
  else if (differentClients)
    ret=FORCE_UPDATE;

  n = sendDataOnly(file_des,&ret,sizeof(ret));
  if (ret==FAIL) {
    n += sendDataOnly(file_des,mess,sizeof(mess));
  }
  return ret; 

}

int stop_acquisition(int file_des) {

  int ret=OK;
  int n;
  

  sprintf(mess,"can't stop acquisition\n");

#ifdef VERBOSE
  printf("Stopping acquisition\n");
#endif 

    
  if (differentClients==1 && lockStatus==1) {
    ret=FAIL;
    sprintf(mess,"Detector locked by %s\n",lastClientIP);  
  } else {
    ret=stopStateMachine();
  }
  
  if (ret==FAIL)
    sprintf(mess,"Stop acquisition failed\n");
  else if (differentClients)
    ret=FORCE_UPDATE;

  n = sendDataOnly(file_des,&ret,sizeof(ret));
  if (ret==FAIL) {
    n += sendDataOnly(file_des,mess,sizeof(mess));
  }
  return ret; 


}

int start_readout(int file_des) {


  int ret=OK;
  int n;
  

  sprintf(mess,"can't start readout\n");

#ifdef VERBOSE
  printf("Starting readout\n");
#endif     
  if (differentClients==1 && lockStatus==1) {
    ret=FAIL;
    sprintf(mess,"Detector locked by %s\n",lastClientIP);  
  } else {
    ret=startReadOut();
  }
  if (ret==FAIL)
    sprintf(mess,"Start readout failed\n");
  else if (differentClients)
    ret=FORCE_UPDATE;

  n = sendDataOnly(file_des,&ret,sizeof(ret));
  if (ret==FAIL) {
    n += sendDataOnly(file_des,mess,sizeof(mess));
  }
  return ret; 



}

int get_run_status(int file_des) {  

  int ret=OK;
  int n;
  
  int retval;
  enum runStatus s;
  sprintf(mess,"getting run status\n");

#ifdef VERBOSE
  printf("Getting status\n");
#endif 

  retval= runState();
  printf("\n\nSTATUS=%08x\n",retval);
  
    //error
    if(retval&SOME_FIFO_FULL_BIT){
      printf("-----------------------------------ERROR--------------------------------------x%0x\n",retval);
      s=ERROR;
    } else if(!(retval&RUN_BUSY_BIT)){ // by Anna 24.10.2012
      if((retval&READMACHINE_BUSY_BIT)  ){ // ///and readbusy=1, its last frame read
	printf("-----------------------------------READ MACHINE BUSY--------------------------\n");
	s=TRANSMITTING;
      }else if((retval&STOPPED_BIT)  ){ // ///and readbusy=1, its last frame read
	printf("-----------------------------------STOPPED--------------------------\n");
	s=STOPPED;
      } else if (!(retval&ALL_FIFO_EMPTY_BIT)) {
	printf("-----------------------------------DATA IN FIFO--------------------------\n");
	s=TRANSMITTING;
      } else if(!(retval&0xeffff)){
	//if(!(retval&0x00000001)){
	printf("-----------------------------------IDLE--------------------------------------\n");
	s=IDLE;
      } else {
	printf("-----------------------------------Unknown status %08x--------------------------------------\n", retval);
	s=ERROR;
	ret=FAIL;
      }
    }    else {
      if (retval&WAITING_FOR_TRIGGER_BIT){
	printf("-----------------------------------WAITING-----------------------------------\n");
	s=WAITING;
      }
      else{
	printf("-----------------------------------RUNNING-----------------------------------\n");
	s=RUNNING;
      }
    }
  if (ret!=OK) {
    printf("get status failed %04x\n",retval);
    sprintf(mess, "get status failed %08x\n", retval);
    
  } else if (differentClients)
    ret=FORCE_UPDATE;

  n = sendDataOnly(file_des,&ret,sizeof(ret));
  if (ret==FAIL) {
    n += sendDataOnly(file_des,mess,sizeof(mess));
  } else {
    n += sendDataOnly(file_des,&s,sizeof(s));
  }
  return ret; 



}

int read_frame(int file_des) {
  int n;
  u_int16_t* p=NULL;

  if (differentClients==1 && lockStatus==1) {
    dataret=FAIL;
    sprintf(mess,"Detector locked by %s\n",lastClientIP);
    printf("Warning: %s\n",mess);
    sendDataOnly(file_des,&dataret,sizeof(dataret));
    sendDataOnly(file_des,mess,sizeof(mess));
#ifdef VERBOSE
    printf("dataret %d\n",dataret);
#endif
    return dataret;

  }

  p=fifo_read_frame();
  if (p) {
    nframes++;
    dataretval=(char*)ram_values;
    dataret=OK;
    #ifdef VERBOSE
      printf("sending data of %d frames\n",nframes);
     #endif
	sendDataOnly(file_des,&dataret,sizeof(dataret));
	#ifdef VERYVERBOSE
	  printf("sending pointer %x of size %d\n",(unsigned int)(dataretval),dataBytes);
	 #endif
	  n=sendDataOnly(file_des,dataretval,dataBytes);
	  printf("Sent %d bytes\n",n);
  } else  {
      if (getFrames()>-1) {
	dataret=FAIL;
	sprintf(mess,"no data and run stopped: %d frames left\n",(int)(getFrames()+2));
	printf("%s\n",mess);
      } else {
	dataret=FINISHED;
	sprintf(mess,"acquisition successfully finished\n");
	printf("%s\n",mess);
	if (differentClients)
	  dataret=FORCE_UPDATE;
      }
#ifdef VERBOSE
      printf("Frames left %d\n",(int)(getFrames()));
#endif
      sendDataOnly(file_des,&dataret,sizeof(dataret));
      sendDataOnly(file_des,mess,sizeof(mess));
  }




  return dataret;
      
}








int read_all(int file_des) {

while(read_frame(file_des)==OK) {

#ifdef VERBOSE
  printf("frame read\n");
#endif   
    ;
  }
#ifdef VERBOSE
  printf("Frames finished\n");
#endif   
  return OK; 


}

int start_and_read_all(int file_des) {
  //int dataret=OK;
#ifdef VERBOSE
  printf("Starting and reading all frames\n");
#endif 

  if (differentClients==1 && lockStatus==1) {
    dataret=FAIL;
    sprintf(mess,"Detector locked by %s\n",lastClientIP);  
    sendDataOnly(file_des,&dataret,sizeof(dataret));
    sendDataOnly(file_des,mess,sizeof(mess));
    return dataret;

  }

  startStateMachine();

  /*  ret=startStateMachine();  
      if (ret!=OK) {
      sprintf(mess,"could not start state machine\n");
    sendDataOnly(file_des,&ret,sizeof(ret));
    sendDataOnly(file_des,mess,sizeof(mess));
    
    #ifdef VERBOSE
    printf("could not start state machine\n");
#endif
} else {*/
  read_all(file_des);
#ifdef VERBOSE
  printf("Frames finished\n");
#endif   
  //}


  return OK; 


}

int set_timer(int file_des) {
  enum timerIndex ind;
  int64_t tns;
  int n;
  int64_t retval;
  int ret=OK;
  

  sprintf(mess,"can't set timer\n");
  
  n = receiveDataOnly(file_des,&ind,sizeof(ind));
  if (n < 0) {
    sprintf(mess,"Error reading from socket\n");
    ret=FAIL;
  }
  
  n = receiveDataOnly(file_des,&tns,sizeof(tns)); // total received data
  if (n < 0) {
    sprintf(mess,"Error reading from socket\n");
    ret=FAIL;
  }
  
  if (ret!=OK) {
    printf(mess);
  }

#ifdef VERBOSE
  printf("setting timer %d to %lld ns\n",ind,tns);
#endif 
  if (ret==OK) {

    if (differentClients==1 && lockStatus==1 && tns!=-1) { 
      ret=FAIL;
      sprintf(mess,"Detector locked by %s\n",lastClientIP);
    }  else {
      switch(ind) {
      case FRAME_NUMBER:
	retval=setFrames(tns);
	break;
      case ACQUISITION_TIME: 
	retval=setExposureTime(tns);
	break;
      case FRAME_PERIOD: 
	retval=setPeriod(tns);
	break;
      case DELAY_AFTER_TRIGGER: 
	retval=setDelay(tns);
	break;
      case GATES_NUMBER:
	retval=setGates(tns);
	break;
      case PROBES_NUMBER: 
	sprintf(mess,"can't set timer for moench\n");
	ret=FAIL;
	break;
      case CYCLES_NUMBER: 
	retval=setTrains(tns);
	break;
      case SAMPLES_JCTB:
	retval=setSamples(tns);
	break;
      default:
	ret=FAIL;
	sprintf(mess,"timer index unknown %d\n",ind);
    break;
      }
    }
  }
  if (ret!=OK) {
    printf(mess);
    if (differentClients)
      ret=FORCE_UPDATE;
  }

  if (ret!=OK) {
    printf(mess);
    printf("set timer failed\n");
  } else if (ind==FRAME_NUMBER) {
    //  ret=allocateRAM();
    // if (ret!=OK) 
    //   sprintf(mess, "could not allocate RAM for %lld frames\n", tns);
  }

  n = sendDataOnly(file_des,&ret,sizeof(ret));
  if (ret==FAIL) {
    n = sendDataOnly(file_des,mess,sizeof(mess));
  } else {
#ifdef VERBOSE
  printf("returning ok %d\n",(int)(sizeof(retval)));
#endif 

    n = sendDataOnly(file_des,&retval,sizeof(retval));
  }

  return ret; 

}








int get_time_left(int file_des) {

  enum timerIndex ind;
  int n;
  int64_t retval;
  int ret=OK;
  
  sprintf(mess,"can't get timer\n");
  n = receiveDataOnly(file_des,&ind,sizeof(ind));
  if (n < 0) {
    sprintf(mess,"Error reading from socket\n");
    ret=FAIL;
  }
  

  //#ifdef VERBOSE

  printf("getting time left on timer %d \n",ind);
  //#endif

  if (ret==OK) {
    switch(ind) {
    case FRAME_NUMBER:
       printf("getting frames \n");
      retval=getFrames(); 
      break;
    case ACQUISITION_TIME: 
      retval=getExposureTime();
      break;
    case FRAME_PERIOD: 
      retval=getPeriod();
      break;
    case DELAY_AFTER_TRIGGER: 
      retval=getDelay();
    break;
    case GATES_NUMBER:
      retval=getGates();
      break;
    case PROBES_NUMBER: 
      retval=getProbes();
      break;
    case CYCLES_NUMBER: 
      retval=getTrains();
      break;
    case PROGRESS: 
      retval=getProgress();
      break;
    case ACTUAL_TIME:
      retval=getActualTime();
      break;
    case MEASUREMENT_TIME: 
      retval=getMeasurementTime();
      break;
    case FRAMES_FROM_START:
    case FRAMES_FROM_START_PG:
      retval=getFramesFromStart();
      break;
    case SAMPLES_JCTB:
      retval=setSamples(-1);
      break;
    default:
      ret=FAIL;
      sprintf(mess,"timer index unknown %d\n",ind);
      break;
    }
  }


  if (ret!=OK) {
    printf("get time left failed\n");
  } else if (differentClients)
      ret=FORCE_UPDATE;

  //#ifdef VERBOSE

  printf("time left on timer %d is %lld\n",ind, retval);
  //#endif

  n = sendDataOnly(file_des,&ret,sizeof(ret));
  if (ret==FAIL) {
    n += sendDataOnly(file_des,mess,sizeof(mess));
  } else
    n = sendDataOnly(file_des,&retval,sizeof(retval));

#ifdef VERBOSE

  printf("data sent\n");
#endif 

  return ret; 


}

int set_dynamic_range(int file_des) {


 
  int dr;
  int n;
  int retval;
  int ret=OK;
  
  printf("Set dynamic range?\n");
  sprintf(mess,"can't set dynamic range\n");
  

  n = receiveDataOnly(file_des,&dr,sizeof(dr));
  if (n < 0) {
    sprintf(mess,"Error reading from socket\n");
    ret=FAIL;
  }
  
   
  if (differentClients==1 && lockStatus==1 && dr>=0) {
      ret=FAIL;
      sprintf(mess,"Detector locked by %s\n",lastClientIP);
  }  else {
    retval=setDynamicRange(dr);
  }

  //if (dr>=0 && retval!=dr)   ret=FAIL;
  if (ret!=OK) {
    sprintf(mess,"set dynamic range failed\n");
  } else {
  /*   ret=allocateRAM(); */
/*     if (ret!=OK) */
/*       sprintf(mess,"Could not allocate RAM for the dynamic range selected\n"); */
//    else 
 if (differentClients)
      ret=FORCE_UPDATE;
  }

  n = sendDataOnly(file_des,&ret,sizeof(ret));
  if (ret==FAIL) {
    n = sendDataOnly(file_des,mess,sizeof(mess));
  } else {
    n = sendDataOnly(file_des,&retval,sizeof(retval));
  }
  return ret; 
}

int set_roi(int file_des) {

	int i;
	int ret=OK;
	int nroi=-1;
	int n=0;
	int retvalsize=0;
	ROI arg[MAX_ROIS];
	int retval;
	strcpy(mess,"Could not set/get roi\n");
	//	u_int32_t disable_reg=0;

	n = receiveDataOnly(file_des,&nroi,sizeof(nroi));
	if (n < 0) {
		sprintf(mess,"Error reading from socket\n");
		ret=FAIL;
	}


	if(nroi>=0){
	  n = receiveDataOnly(file_des,arg,nroi*sizeof(ROI));
	  if (n != (nroi*sizeof(ROI))) {
	    sprintf(mess,"Received wrong number of bytes for ROI\n");
	    ret=FAIL;
	  }
	  
	  printf("Setting ROI to:");
	  for( i=0;i<nroi;i++)
	    printf("%d\t%d\t%d\t%d\n",arg[i].xmin,arg[i].xmax,arg[i].ymin,arg[i].ymax);
	  // printf("Error: Function 41 or Setting ROI is not yet implemented in Moench!\n");
	  
	}

	/* execute action if the arguments correctly arrived*/
	if (lockStatus==1 && differentClients==1 && nroi>=0){//necessary???
	  sprintf(mess,"Detector locked by %s\n", lastClientIP);
	  ret=FAIL;
	} else{
	  retval=setROI(nroi,arg,&retvalsize,&ret);
	  if (ret==FAIL){
	    printf("mess:%s\n",mess);
	    sprintf(mess,"Could not set all roi, should have set %d rois, but only set %d rois\n",nroi,retvalsize);
	  }
	}

	if(ret==OK && differentClients){
		printf("Force update\n");
		ret=FORCE_UPDATE;
	}

	/* send answer */
	n = sendDataOnly(file_des,&ret,sizeof(ret));
	if(ret==FAIL)
		n = sendDataOnly(file_des,mess,sizeof(mess));
	else{
		sendDataOnly(file_des,&retvalsize,sizeof(retvalsize));
		sendDataOnly(file_des,arg,retvalsize*sizeof(ROI));
	}
	/*return ok/fail*/
	return ret;
}

int get_roi(int file_des) {


  return FAIL;
}

int set_speed(int file_des) {

  enum speedVariable arg;
  int val,n;
  int ret=OK;
  int retval;
  
  n=receiveDataOnly(file_des,&arg,sizeof(arg));
  if (n < 0) {
    sprintf(mess,"Error reading from socket\n");
    ret=FAIL;
  }
  n=receiveDataOnly(file_des,&val,sizeof(val));
   if (n < 0) {
     sprintf(mess,"Error reading from socket\n");
     ret=FAIL;
   }
  
  
  
  if (ret==OK) {

    /* if (arg==PHASE_SHIFT || arg==ADC_PHASE) { */
      
      
    /*   retval=phaseStep(val); */

    /* } else if ( arg==DBIT_PHASE) { */
    /* 	  retval=dbitPhaseStep(val);  */
    /* } else { */


    /* if (val!=-1) { */


      if (differentClients==1 && lockStatus==1 && val>=0) {
	ret=FAIL;
	sprintf(mess,"Detector locked by %s\n",lastClientIP);
      }  else {
	switch (arg) {
	case PHASE_SHIFT:
	case ADC_PHASE:
	  if (val==-1)
	    retval=getPhase(run_clk_c);
	  else
	    retval=configurePhase(val,run_clk_c);
	  break;

	case DBIT_PHASE: 
	  if (val==-1)
	    retval=getPhase(dbit_clk_c);
	  else
	    retval=configurePhase(val,dbit_clk_c);
	  break;

	case CLOCK_DIVIDER:
	  retval=configureFrequency(val,run_clk_c);//setClockDivider(val,0);
	  if (configureFrequency(-1,sync_clk_c)>retval) {
	    configureFrequency(retval,sync_clk_c);
	    printf("--Configuring sync clk to %d MHz\n",val);
	  } else if (configureFrequency(-1,dbit_clk_c)>val && configureFrequency(-1,adc_clk_c)>retval) {
	    printf("++Configuring sync clk to %d MHz\n",val);
	    configureFrequency(retval,sync_clk_c);
	  }
	  break;

/* 	case PHASE_SHIFT: */
/* 	  retval=phaseStep(val,0); */
/* 	  break; */

	case OVERSAMPLING:
	  retval=setOversampling(val);
	  break;

	case ADC_CLOCK:
	  retval=configureFrequency(val,adc_clk_c);//setClockDivider(val,1);
	  if (configureFrequency(-1,sync_clk_c)>val) {
	    configureFrequency(retval,sync_clk_c);
	    printf("--Configuring sync clk to %d MHz\n",val);
	  } else if (configureFrequency(-1,dbit_clk_c)>val && configureFrequency(-1,run_clk_c)>retval) {
	    printf("++Configuring sync clk to %d MHz\n",val);
	    configureFrequency(retval,sync_clk_c);
	  }
	  break;

	case DBIT_CLOCK:
	  retval=configureFrequency(val,dbit_clk_c);//setClockDivider(val,2);
	  if (configureFrequency(-1,sync_clk_c)>retval){
	    configureFrequency(retval,sync_clk_c);
	    printf("--Configuring sync clk to %d MHz\n",val);
	  } else if (configureFrequency(-1,adc_clk_c)>retval && configureFrequency(-1,run_clk_c)>retval) {
	    printf("++Configuring sync clk to %d MHz\n",val);
	    configureFrequency(retval,sync_clk_c);
	  }
	    
	  break;



	case ADC_PIPELINE:
	  retval=adcPipeline(val);
	  break;


	case DBIT_PIPELINE:
	  retval=dbitPipeline(val);
	  break;

	default:
	  ret=FAIL;
	  sprintf(mess,"Unknown speed parameter %d",arg);
	}
      }
      // }


  }


  

  n = sendDataOnly(file_des,&ret,sizeof(ret));
  if (ret==FAIL) {
    n = sendDataOnly(file_des,mess,sizeof(mess));
  } else {
    n = sendDataOnly(file_des,&retval,sizeof(retval));
  }
  return ret; 
}



int set_readout_flags(int file_des) {

  enum readOutFlags arg;
  int ret=OK;
  enum readOutFlags v=-1;

  receiveDataOnly(file_des,&arg,sizeof(arg));

  switch (arg) {
  case NORMAL_READOUT:
  case DIGITAL_ONLY:
  case ANALOG_AND_DIGITAL:
  case GET_READOUT_FLAGS:
    break;
  default:
    sprintf(mess,"unknown readout flags for jctb\n");
    ret=FAIL;
  }
  if (ret==OK)
    v=setReadOutMode(arg);
  
  if (v<0) {
    ret=FAIL;
    sprintf(mess,"found non valid readout mode (neither analog nor digital)\n");
  }
  sendDataOnly(file_des,&ret,sizeof(ret));
  if (ret==OK) 
    sendDataOnly(file_des,&v,sizeof(v));
  else
    sendDataOnly(file_des,mess,sizeof(mess));
  // sendDataOnly(file_des,mess,sizeof(mess));

  return ret; 
}





int execute_trimming(int file_des) {
 
  int arg[3];
  int ret=FAIL;
  enum trimMode mode;
  
  sprintf(mess,"can't set execute trimming for moench\n");
  
  receiveDataOnly(file_des,&mode,sizeof(mode));
  receiveDataOnly(file_des,arg,sizeof(arg));


  sendDataOnly(file_des,&ret,sizeof(ret));
  sendDataOnly(file_des,mess,sizeof(mess));

  return ret; 
}


int lock_server(int file_des) {

  
  int n;
  int ret=OK;

  int lock;
  n = receiveDataOnly(file_des,&lock,sizeof(lock));
  if (n < 0) {
    sprintf(mess,"Error reading from socket\n");
    printf("Error reading from socket (lock)\n");
    ret=FAIL;
  }
  if (lock>=0) {
    if (lockStatus==0 || strcmp(lastClientIP,thisClientIP)==0 || strcmp(lastClientIP,"none")==0)
      lockStatus=lock;
    else {
      ret=FAIL;
      sprintf(mess,"Server already locked by %s\n", lastClientIP);
    }
  }
 if (differentClients && ret==OK)
   ret=FORCE_UPDATE;
  
  n = sendDataOnly(file_des,&ret,sizeof(ret));
  if (ret==FAIL) {
    n = sendDataOnly(file_des,mess,sizeof(mess));
  }  else
    n = sendDataOnly(file_des,&lockStatus,sizeof(lockStatus));
  
  return ret;

}

int set_port(int file_des) {
	int n;
	int ret=OK;
	int sd=-1;

	enum portType p_type; /** data? control? stop? Unused! */
	int p_number; /** new port number */

	n = receiveDataOnly(file_des,&p_type,sizeof(p_type));
	if (n < 0) {
		sprintf(mess,"Error reading from socket\n");
		printf("Error reading from socket (ptype)\n");
		ret=FAIL;
	}

	n = receiveDataOnly(file_des,&p_number,sizeof(p_number));
	if (n < 0) {
		sprintf(mess,"Error reading from socket\n");
		printf("Error reading from socket (pnum)\n");
		ret=FAIL;
	}
	if (differentClients==1 && lockStatus==1 ) {
		ret=FAIL;
		sprintf(mess,"Detector locked by %s\n",lastClientIP);
	}  else {
		if (p_number<1024) {
			sprintf(mess,"Too low port number %d\n", p_number);
			printf("\n");
			ret=FAIL;
		}

		printf("set port %d to %d\n",p_type, p_number);

		sd=bindSocket(p_number);
	}
	if (sd>=0) {
		ret=OK;
		if (differentClients )
			ret=FORCE_UPDATE;
	} else {
		ret=FAIL;
		sprintf(mess,"Could not bind port %d\n", p_number);
		printf("Could not bind port %d\n", p_number);
		if (sd==-10) {
			sprintf(mess,"Port %d already set\n", p_number);
			printf("Port %d already set\n", p_number);

		}
	}

	n = sendDataOnly(file_des,&ret,sizeof(ret));
	if (ret==FAIL) {
		n = sendDataOnly(file_des,mess,sizeof(mess));
	} else {
		n = sendDataOnly(file_des,&p_number,sizeof(p_number));
		closeConnection(file_des);
		exitServer(sockfd);
		sockfd=sd;

	}

	return ret;

}

int get_last_client_ip(int file_des) {
  int ret=OK;
  int n;
 if (differentClients )
   ret=FORCE_UPDATE;
  n = sendDataOnly(file_des,&ret,sizeof(ret));
  n = sendDataOnly(file_des,lastClientIP,sizeof(lastClientIP));
  
  return ret;

}


int send_update(int file_des) {

  int ret=OK;
  enum detectorSettings t;
  int n;//int thr, n;
  //int it;
  int64_t retval, tns=-1;
  enum readOutFlags v=-1;
  n = sendDataOnly(file_des,lastClientIP,sizeof(lastClientIP));
  n = sendDataOnly(file_des,&nModX,sizeof(nModX));
  n = sendDataOnly(file_des,&nModY,sizeof(nModY));
  n = sendDataOnly(file_des,&dynamicRange,sizeof(dynamicRange));
  n = sendDataOnly(file_des,&dataBytes,sizeof(dataBytes));
  t=GET_SETTINGS;//setSettings(GET_SETTINGS,-1);
  n = sendDataOnly(file_des,&t,sizeof(t));
/*  thr=getThresholdEnergy();
  n = sendDataOnly(file_des,&thr,sizeof(thr));*/
  retval=setFrames(tns);
  n = sendDataOnly(file_des,&retval,sizeof(int64_t));
  retval=setExposureTime(tns);
  n = sendDataOnly(file_des,&retval,sizeof(int64_t));
  retval=setPeriod(tns);
  n = sendDataOnly(file_des,&retval,sizeof(int64_t));
  retval=setDelay(tns);
  n = sendDataOnly(file_des,&retval,sizeof(int64_t));
  retval=setGates(tns);
  n = sendDataOnly(file_des,&retval,sizeof(int64_t));
/*  retval=setProbes(tns);
  n = sendDataOnly(file_des,&retval,sizeof(int64_t));*/
  retval=setTrains(tns);
  n = sendDataOnly(file_des,&retval,sizeof(int64_t));
  retval=setSamples(tns);
  n = sendDataOnly(file_des,&retval,sizeof(int64_t));

  v=setReadOutMode(-1);
  sendDataOnly(file_des,&v,sizeof(v));
  
  if (lockStatus==0) {
    strcpy(lastClientIP,thisClientIP);
  }

  return ret;
  

}
int update_client(int file_des) {

  int ret=OK;

  sendDataOnly(file_des,&ret,sizeof(ret));
  return send_update(file_des);
  
  

}


int configure_mac(int file_des) {

	int ret=OK;
	char arg[5][50];
	int n;

	int imod=0;//should be in future sent from client as -1, arg[2]
	int ipad;
	long long int imacadd;
	long long int idetectormacadd;
	int udpport;
	int detipad;
	int retval=-100;

	sprintf(mess,"Can't configure MAC\n");


	n = receiveDataOnly(file_des,arg,sizeof(arg));
	if (n < 0) {
		sprintf(mess,"Error reading from socket\n");
		ret=FAIL;
	}

	sscanf(arg[0], "%x", 		&ipad);
	sscanf(arg[1], "%llx", 	&imacadd);
	sscanf(arg[2], "%x", 		&udpport);
	sscanf(arg[3], "%llx",	&idetectormacadd);
	sscanf(arg[4], "%x",		&detipad);

	//#ifdef VERBOSE
	int i;
	printf("\ndigital_test_bit in server %d\t",digitalTestBit);
	printf("\nipadd %x\t",ipad);
	printf("destination ip is %d.%d.%d.%d = 0x%x \n",(ipad>>24)&0xff,(ipad>>16)&0xff,(ipad>>8)&0xff,(ipad)&0xff,ipad);
	printf("macad:%llx\n",imacadd);
	for (i=0;i<6;i++)
		printf("mac adress %d is 0x%x \n",6-i,(unsigned int)(((imacadd>>(8*i))&0xFF)));
	printf("udp port:0x%x\n",udpport);
	printf("detector macad:%llx\n",idetectormacadd);
	for (i=0;i<6;i++)
		printf("detector mac adress %d is 0x%x \n",6-i,(unsigned int)(((idetectormacadd>>(8*i))&0xFF)));
	printf("detipad %x\n",detipad);
	printf("\n");
	//#endif



	if (imod>=getNModBoard())
		ret=FAIL;
	if (imod<0)
		imod=ALLMOD;

	//#ifdef VERBOSE
	printf("Configuring MAC of module %d at port %x\n", imod, udpport);
	//#endif
	//#ifdef MCB_FUNCS
	if (ret==OK){
		if(runBusy()){
			ret=stopStateMachine();
			if(ret==FAIL)
				strcpy(mess,"could not stop detector acquisition to configure mac");
		}

		if(ret==OK)
			configureMAC(ipad,imacadd,idetectormacadd,detipad,digitalTestBit,udpport);
		retval=getAdcConfigured();
	}
	//#endif
	if (ret==FAIL)
		printf("configuring MAC of mod %d failed\n", imod);
	else
		printf("Configuremac successful of mod %d and adc %d\n",imod,retval);

	if (differentClients)
		ret=FORCE_UPDATE;

	/* send answer */
	/* send OK/failed */
	n = sendDataOnly(file_des,&ret,sizeof(ret));
	if (ret==FAIL)
		n += sendDataOnly(file_des,mess,sizeof(mess));
	else
		n += sendDataOnly(file_des,&retval,sizeof(retval));
	/*return ok/fail*/
	return ret;

}



int load_image(int file_des) {
	int retval;
	int ret=OK;
	int n;
	enum imageType index;
	short int ImageVals[N_CHAN*N_CHIP];

	sprintf(mess,"Loading image failed\n");

	n = receiveDataOnly(file_des,&index,sizeof(index));
	if (n < 0) {
		sprintf(mess,"Error reading from socket\n");
		ret=FAIL;
	}

	n = receiveDataOnly(file_des,ImageVals,dataBytes);
	if (n < 0) {
		sprintf(mess,"Error reading from socket\n");
		ret=FAIL;
	}

	switch (index) {
	case DARK_IMAGE :
#ifdef VERBOSE
		printf("Loading Dark image\n");
#endif
		break;
	case GAIN_IMAGE :
#ifdef VERBOSE
		printf("Loading Gain image\n");
#endif
		break;
	default:
		printf("Unknown index %d\n",index);
		sprintf(mess,"Unknown index %d\n",index);
		ret=FAIL;
	    break;
	}

	if (ret==OK) {
		if (differentClients==1 && lockStatus==1) {
			ret=FAIL;
			sprintf(mess,"Detector locked by %s\n",lastClientIP);
		} else{
			retval=loadImage(index,ImageVals);
			if (retval==-1)
				ret = FAIL;
		}
	}

	if(ret==OK){
		if (differentClients)
			ret=FORCE_UPDATE;
	}

	/* send answer */
	/* send OK/failed */
	n = sendDataOnly(file_des,&ret,sizeof(ret));
	if (ret!=FAIL) {
		/* send return argument */
		n += sendDataOnly(file_des,&retval,sizeof(retval));
	} else {
		n += sendDataOnly(file_des,mess,sizeof(mess));
	}

	/*return ok/fail*/
	return ret;
}



int set_master(int file_des) {

  enum masterFlags retval=GET_MASTER;
  enum masterFlags arg;
  int n;
  int ret=OK;
  // int regret=OK;


  sprintf(mess,"can't set master flags\n");


  n = receiveDataOnly(file_des,&arg,sizeof(arg));
  if (n < 0) {
    sprintf(mess,"Error reading from socket\n");
    ret=FAIL;
  }


#ifdef VERBOSE
  printf("setting master flags  to %d\n",arg);
#endif

  if (differentClients==1 && lockStatus==1 && arg!=GET_READOUT_FLAGS) {
    ret=FAIL;
    sprintf(mess,"Detector locked by %s\n",lastClientIP);
  }  else {
    retval=setMaster(arg);

  }
  if (retval==GET_MASTER) {
    ret=FAIL;
  }
  n = sendDataOnly(file_des,&ret,sizeof(ret));
  if (ret==FAIL) {
    n = sendDataOnly(file_des,mess,sizeof(mess));
  } else {
    n = sendDataOnly(file_des,&retval,sizeof(retval));
  }
  return ret;
}






int set_synchronization(int file_des) {

  enum synchronizationMode retval=GET_MASTER;
  enum synchronizationMode arg;
  int n;
  int ret=OK;
  //int regret=OK;


  sprintf(mess,"can't set synchronization mode\n");


  n = receiveDataOnly(file_des,&arg,sizeof(arg));
  if (n < 0) {
    sprintf(mess,"Error reading from socket\n");
    ret=FAIL;
  }
#ifdef VERBOSE
  printf("setting master flags  to %d\n",arg);
#endif

  if (differentClients==1 && lockStatus==1 && arg!=GET_READOUT_FLAGS) {
    ret=FAIL;
    sprintf(mess,"Detector locked by %s\n",lastClientIP);
  }  else {
    //ret=setStoreInRAM(0);
    // initChipWithProbes(0,0,0, ALLMOD);
    retval=setSynchronization(arg);
  }
  if (retval==GET_SYNCHRONIZATION_MODE) {
    ret=FAIL;
  }
  n = sendDataOnly(file_des,&ret,sizeof(ret));
  if (ret==FAIL) {
    n = sendDataOnly(file_des,mess,sizeof(mess));
  } else {
    n = sendDataOnly(file_des,&retval,sizeof(retval));
  }
  return ret;
}






int read_counter_block(int file_des) {

	int ret=OK;
	int n;
	int startACQ;
	//char *retval=NULL;
	short int CounterVals[N_CHAN*N_CHIP];

	sprintf(mess,"Read counter block failed\n");

	n = receiveDataOnly(file_des,&startACQ,sizeof(startACQ));
	if (n < 0) {
		sprintf(mess,"Error reading from socket\n");
		ret=FAIL;
	}

	if (ret==OK) {
		if (differentClients==1 && lockStatus==1) {
			ret=FAIL;
			sprintf(mess,"Detector locked by %s\n",lastClientIP);
		} else{
			ret=readCounterBlock(startACQ,CounterVals);
#ifdef VERBOSE
			int i;
			for(i=0;i<6;i++)
				printf("%d:%d\t",i,CounterVals[i]);
#endif
		}
	}

	if(ret!=FAIL){
		if (differentClients)
			ret=FORCE_UPDATE;
	}

	/* send answer */
	/* send OK/failed */
	n = sendDataOnly(file_des,&ret,sizeof(ret));
	if (ret!=FAIL) {
		/* send return argument */
		n += sendDataOnly(file_des,CounterVals,dataBytes);//1280*2
	} else {
		n += sendDataOnly(file_des,mess,sizeof(mess));
	}

	/*return ok/fail*/
	return ret;
}





int reset_counter_block(int file_des) {

	int ret=OK;
	int n;
	int startACQ;

	sprintf(mess,"Reset counter block failed\n");

	n = receiveDataOnly(file_des,&startACQ,sizeof(startACQ));
	if (n < 0) {
		sprintf(mess,"Error reading from socket\n");
		ret=FAIL;
	}

	if (ret==OK) {
		if (differentClients==1 && lockStatus==1) {
			ret=FAIL;
			sprintf(mess,"Detector locked by %s\n",lastClientIP);
		} else
			ret=resetCounterBlock(startACQ);
	}

	if(ret==OK){
		if (differentClients)
			ret=FORCE_UPDATE;
	}

	/* send answer */
	/* send OK/failed */
	n = sendDataOnly(file_des,&ret,sizeof(ret));
	if (ret==FAIL)
		n += sendDataOnly(file_des,mess,sizeof(mess));

	/*return ok/fail*/
	return ret;
}



int calibrate_pedestal(int file_des){

	int ret=OK;
	int retval=-1;
	int n;
	int frames;

	sprintf(mess,"Could not calibrate pedestal\n");

	n = receiveDataOnly(file_des,&frames,sizeof(frames));
	if (n < 0) {
		sprintf(mess,"Error reading from socket\n");
		ret=FAIL;
	}

	if (ret==OK) {
		if (differentClients==1 && lockStatus==1) {
			ret=FAIL;
			sprintf(mess,"Detector locked by %s\n",lastClientIP);
		} else
			ret=calibratePedestal(frames);
	}

	if(ret==OK){
		if (differentClients)
			ret=FORCE_UPDATE;
	}

	/* send answer */
	/* send OK/failed */
	n = sendDataOnly(file_des,&ret,sizeof(ret));
	if (ret==FAIL)
		n += sendDataOnly(file_des,mess,sizeof(mess));
	else
		n += sendDataOnly(file_des,&retval,sizeof(retval));

	/*return ok/fail*/
	return ret;
}


int set_ctb_pattern(int file_des){

  int ret=OK;//FAIL;
	int retval=-1;
	int n;
	int mode;
	uint64_t word, retval64, t;
	int addr;
	int level, start, stop, nl;
	uint64_t pat[1024];

	sprintf(mess,"Could not set pattern\n");

 	n = receiveDataOnly(file_des,&mode,sizeof(mode)); 
	printf("pattern mode is %d\n",mode);
	switch (mode) {

	case 0: //sets word
	  n = receiveDataOnly(file_des,&addr,sizeof(addr)); 
	  n = receiveDataOnly(file_des,&word,sizeof(word)); 
	  ret=OK;

	  printf("pattern addr is %d %x\n",addr, word);
	  switch (addr) {
	  case -1:
	    retval64=writePatternIOControl(word);
	    break;
	  case -2:
	    retval64=writePatternClkControl(word);
	    break;
	  default:
	    retval64=writePatternWord(addr,word);
	  };


	  //write word;
	  //@param addr address of the word, -1 is I/O control register,  -2 is clk control register
	  //@param word 64bit word to be written, -1 gets
	  
	  n = sendDataOnly(file_des,&ret,sizeof(ret));
	  if (ret==FAIL)
	    n += sendDataOnly(file_des,mess,sizeof(mess));
	  else
	    n += sendDataOnly(file_des,&retval64,sizeof(retval64));
	  break;

	case 1: //pattern loop
	  // printf("loop\n");
	  n = receiveDataOnly(file_des,&level,sizeof(level)); 
	  n = receiveDataOnly(file_des,&start,sizeof(start)); 
	  n = receiveDataOnly(file_des,&stop,sizeof(stop)); 
	  n = receiveDataOnly(file_des,&nl,sizeof(nl)); 
	  


	  //       printf("level %d start %x stop %x nl %d\n",level, start, stop, nl);
  /** Sets the pattern or loop limits in the CTB
      @param level -1 complete pattern, 0,1,2, loop level
      @param start start address if >=0
      @param stop stop address if >=0
      @param n number of loops (if level >=0)
      @returns OK/FAIL
  */
	  ret=setPatternLoop(level, &start, &stop, &nl);
	  
	  n = sendDataOnly(file_des,&ret,sizeof(ret));
	  if (ret==FAIL)
	    n += sendDataOnly(file_des,mess,sizeof(mess));
	  else {
	    n += sendDataOnly(file_des,&start,sizeof(start));
	    n += sendDataOnly(file_des,&stop,sizeof(stop));
	    n += sendDataOnly(file_des,&nl,sizeof(nl));
	  }
	  break;



	case 2: //wait address
	  printf("wait\n");
	  n = receiveDataOnly(file_des,&level,sizeof(level)); 
	  n = receiveDataOnly(file_des,&addr,sizeof(addr)); 

	  

  /** Sets the wait address in the CTB
      @param level  0,1,2, wait level
      @param addr wait address, -1 gets
      @returns actual value
  */
	  printf("wait addr %d %x\n",level, addr);
	  retval=setPatternWaitAddress(level,addr);
	  printf("ret: wait addr %d %x\n",level, retval);
	  ret=OK;
	  n = sendDataOnly(file_des,&ret,sizeof(ret));
	  if (ret==FAIL)
	    n += sendDataOnly(file_des,mess,sizeof(mess));
	  else {
	    n += sendDataOnly(file_des,&retval,sizeof(retval));

	  }
	  

	  break;


	case 3: //wait time
	  printf("wait time\n");
	  n = receiveDataOnly(file_des,&level,sizeof(level)); 
	  n = receiveDataOnly(file_des,&t,sizeof(t)); 


   /** Sets the wait time in the CTB
      @param level  0,1,2, wait level
      @param t wait time, -1 gets
      @returns actual value
  */

	  ret=OK;

	  retval64=setPatternWaitTime(level,t);

	  n = sendDataOnly(file_des,&ret,sizeof(ret));
	  if (ret==FAIL)
	    n += sendDataOnly(file_des,mess,sizeof(mess));
	  else
	    n += sendDataOnly(file_des,&retval64,sizeof(retval64));

	  break;



	case 4:
	  n = receiveDataOnly(file_des,pat,sizeof(pat)); 
	  for (addr=0; addr<1024; addr++)
	    writePatternWord(addr,word);
	  ret=OK;
	  retval=0;
	  n = sendDataOnly(file_des,&ret,sizeof(ret));
	  if (ret==FAIL)
	    n += sendDataOnly(file_des,mess,sizeof(mess));
	  else
	    n += sendDataOnly(file_des,&retval64,sizeof(retval64));

	  break;

	  



	default:
	  ret=FAIL;
	  printf(mess);
	  sprintf(mess,"%s - wrong mode %d\n",mess, mode);
	  n = sendDataOnly(file_des,&ret,sizeof(ret));
	  n += sendDataOnly(file_des,mess,sizeof(mess));
	  
	  

	}


	/*return ok/fail*/
	return ret;
}


int write_adc_register(int file_des) {

  int retval;
  int ret=OK;
  int arg[2]; 
  int addr, val;
  int n;

  sprintf(mess,"Can't write to register\n");

  n = receiveDataOnly(file_des,arg,sizeof(arg));
  if (n < 0) {
    sprintf(mess,"Error reading from socket\n");
    ret=FAIL;
  }
  addr=arg[0];
  val=arg[1];

#ifdef VERBOSE
  printf("writing to register 0x%x data 0x%x\n", addr, val);
#endif  

  if (differentClients==1 && lockStatus==1) {
    ret=FAIL;
    sprintf(mess,"Detector locked by %s\n",lastClientIP);    
  }


  if(ret!=FAIL){
    ret=writeADC(addr,val);
    if (ret==OK)
      retval=val;
  }
  

#ifdef VERBOSE
  printf("Data set to 0x%x\n",  retval);
#endif  
  if (retval==val) {
    ret=OK;
    if (differentClients)
      ret=FORCE_UPDATE;
  } else {
    ret=FAIL;
    sprintf(mess,"Writing to register 0x%x failed: wrote 0x%x but read 0x%x\n", addr, val, retval);
  }

  /* send answer */
  /* send OK/failed */
  n = sendDataOnly(file_des,&ret,sizeof(ret));
  if (ret!=FAIL) {
    /* send return argument */
    n += sendDataOnly(file_des,&retval,sizeof(retval));
  } else {
    n += sendDataOnly(file_des,mess,sizeof(mess));
  }

  /*return ok/fail*/
  return ret; 

}

int power_chip(int file_des) {

	int retval=-1;
	int ret=OK;
	int arg=-1;
	int n;

	n = receiveDataOnly(file_des,&arg,sizeof(arg));
	if (n < 0) {
		sprintf(mess,"Error reading from socket\n");
		ret=FAIL;
	}


#ifdef VERBOSE
	printf("Power chip to %d\n", arg);
#endif

	if (differentClients==1 && lockStatus==1 && arg!=-1) {
		ret=FAIL;
		sprintf(mess,"Detector locked by %s\n",lastClientIP);
	} else {
		retval=powerChip(arg);
#ifdef VERBOSE
		printf("Chip powered: %d\n",retval);
#endif

		if (retval==arg || arg<0) {
			ret=OK;
		} else {
			ret=FAIL;
			printf("Powering chip failed, wrote %d but read %d\n", arg, retval);
		}

	}
	if (ret==OK && differentClients==1)
		ret=FORCE_UPDATE;

	/* send answer */
	n = sendDataOnly(file_des,&ret,sizeof(ret));
	if (ret==FAIL) {
		n += sendDataOnly(file_des,mess,sizeof(mess));
	} else
		n += sendDataOnly(file_des,&retval,sizeof(retval));

	return ret;
}


int reset_fpga(int file_des) {
	int ret=OK;
	int n;
	sprintf(mess,"Reset FPGA unsuccessful\n");

	resetFPGA();
	initializeDetector();

	ret = FORCE_UPDATE;
	n = sendDataOnly(file_des,&ret,sizeof(ret));
	if (ret==FAIL)
		n += sendDataOnly(file_des,mess,sizeof(mess));

	/*return ok/fail*/
	return ret;
}


int program_fpga(int file_des) {
	int ret=OK;
	int n;
	sprintf(mess,"Program FPGA unsuccessful\n");
	char* fpgasrc = NULL;
	FILE* fp = NULL;
	size_t filesize = 0;
	size_t unitprogramsize = 0;
	size_t totalsize = 0;


	//filesize
	n = receiveDataOnly(file_des,&filesize,sizeof(filesize));
	if (n < 0) {
		sprintf(mess,"Error reading from socket\n");
		ret=FAIL;
	}
	totalsize = filesize;
#ifdef VERY_VERBOSE
	printf("\n\n Total size is:%d\n",totalsize);
#endif

	//lock
	if (ret==OK && differentClients==1 && lockStatus==1) {
		ret=FAIL;
		sprintf(mess,"Detector locked by %s\n",lastClientIP);
		filesize = 0;
	}

	//opening file pointer to flash and telling FPGA to not touch flash
	if(ret == OK && startWritingFPGAprogram(&fp) != OK){
		sprintf(mess,"Could not write to flash. Error at startup.\n");
		cprintf(RED,"%s",mess);
		ret=FAIL;
		filesize = 0;
	}

	//---------------- first ret ----------------
	n = sendDataOnly(file_des,&ret,sizeof(ret));
	if (ret==FAIL)
		n += sendDataOnly(file_des,mess,sizeof(mess));
	//---------------- first ret ----------------


	//erasing flash
	if(ret != FAIL){
		eraseFlash();
		fpgasrc = (char*)malloc(MAX_FPGAPROGRAMSIZE);
	}



	//writing to flash part by part
	while(ret != FAIL && filesize){

		unitprogramsize = MAX_FPGAPROGRAMSIZE;  //2mb
		if(unitprogramsize > filesize) //less than 2mb
			unitprogramsize = filesize;
#ifdef VERY_VERBOSE
		printf("unit size to receive is:%d\n",unitprogramsize);
		printf("filesize:%d currentpointer:%d\n",filesize,currentPointer);
#endif


		//receive
		n = receiveDataOnly(file_des,fpgasrc,unitprogramsize);
		if (n < 0) {
			sprintf(mess,"Error reading from socket\n");
			ret=FAIL;
		}


		if (ret==OK) {
			if(!(unitprogramsize - filesize)){
				fpgasrc[unitprogramsize]='\0';
				filesize-=unitprogramsize;
				unitprogramsize++;
			}else
				filesize-=unitprogramsize;

			ret = writeFPGAProgram(fpgasrc,unitprogramsize,fp);
		}


		//---------------- middle rets ----------------
		n = sendDataOnly(file_des,&ret,sizeof(ret));
		if (ret==FAIL) {
			n += sendDataOnly(file_des,mess,sizeof(mess));
			cprintf(RED,"Failure: Breaking out of program receiving\n");
		}
		//---------------- middle rets ----------------


		if(ret != FAIL){
			//print progress
			printf("Writing to Flash:%d%%\r",(int) (((double)(totalsize-filesize)/totalsize)*100) );
			fflush(stdout);
		}

	}



	printf("\n");

	//closing file pointer to flash and informing FPGA
	if(stopWritingFPGAprogram(fp) == FAIL){
		sprintf(mess,"Could not write to flash. Error at end.\n");
		cprintf(RED,"%s",mess);
		ret=FAIL;
	}

	if(ret!=FAIL){
		ret=FORCE_UPDATE;
	}


	//---------------- last ret ----------------
	n = sendDataOnly(file_des,&ret,sizeof(ret));
	if (ret==FAIL)
		n += sendDataOnly(file_des,mess,sizeof(mess));
	//---------------- last ret ----------------


	//free resources
	if(fpgasrc != NULL)
		free(fpgasrc);
	if(fp!=NULL)
		fclose(fp);
#ifdef VERY_VERBOSE
	printf("Done with program receiving command\n");
#endif
	/*return ok/fail*/
	return ret;
}


int activate(int file_des) {

	int retval=-1;
	int ret=OK;
	int arg=-1;
	int n;
	
	sprintf(mess,"Can't activate detector\n");
	n = receiveDataOnly(file_des,&arg,sizeof(arg));
	if (n < 0) {
		sprintf(mess,"Error reading from socket\n");
		ret=FAIL;
	}

	if (ret==OK && differentClients==1)
		ret=FORCE_UPDATE;
	retval=arg;
	/* send answer */
	n = sendDataOnly(file_des,&ret,sizeof(ret));
	if (ret==FAIL) {
		n += sendDataOnly(file_des,mess,sizeof(mess));
	} else
		n += sendDataOnly(file_des,&retval,sizeof(retval));

	return ret;
}

int prepare_acquisition(int file_des) {
	int ret = OK;
	int n=0;
	strcpy(mess,"prepare acquisition failed\n");


	ret = FAIL;
	strcpy(mess,"Not implemented for this detector\n");
	cprintf(RED, "Warning: %s", mess);
	/*
	//lock
	if (ret==OK && differentClients && lockStatus) {
		ret=FAIL;
		sprintf(mess,"Detector locked by %s\n",lastClientIP);
		cprintf(RED, "Warning: %s", mess);
	}
	else {
		ret = startReceiver(1);
		if (ret == FAIL)
			cprintf(RED, "Warning: %s", mess);
	}
*/
	
	if(ret==OK && differentClients)
		ret=FORCE_UPDATE;

	/* send answer */
	n = sendDataOnly(file_des,&ret,sizeof(ret));
	if (ret==FAIL) {
		n += sendDataOnly(file_des,mess,sizeof(mess));
	} 
	return ret;
}

int cleanup_acquisition(int file_des) {
	int ret = OK;
	int n=0;


	//to receive any arguments
	while (n > 0)
		n = receiveDataOnly(file_des,mess,MAX_STR_LENGTH);

	ret = FAIL;
	strcpy(mess,"Not implemented for this detector\n");
	cprintf(RED, "Warning: %s", mess);
	/*
	if (lockStatus && differentClients){//necessary???
		sprintf(mess,"Detector locked by %s\n", lastClientIP);
		cprintf(RED, "Warning: %s", mess);
		ret=FAIL;
	}
	else {
		ret=startReceiver(0);
		if (ret == FAIL)
			cprintf(RED, "Warning: %s", mess);
	}
	*/
	if(ret==OK && differentClients)
		ret=FORCE_UPDATE;

	/* send answer */
	n = sendDataOnly(file_des,&ret,sizeof(ret));
	if (ret==FAIL) {
		n += sendDataOnly(file_des,mess,sizeof(mess));
	} 
	return ret;
}
