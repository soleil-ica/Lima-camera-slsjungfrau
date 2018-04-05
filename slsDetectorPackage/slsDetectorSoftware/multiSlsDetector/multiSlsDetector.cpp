/*******************************************************************

Date:       $Date$
Revision:   $Rev$
Author:     $Author$
URL:        $URL$
ID:         $Id$

********************************************************************/



#include "multiSlsDetector.h"
#include "slsDetector.h"
#include "multiSlsDetectorCommand.h"
#include "multiSlsDetectorClient.h"
#include "postProcessingFuncs.h"
#include "usersFunctions.h"
#include "ThreadPool.h"
#include "ZmqSocket.h"

#include  <sys/types.h>
#include  <sys/ipc.h>
#include  <sys/shm.h>
#include  <iostream>
#include  <string>
using namespace std;


char ans[MAX_STR_LENGTH];

int multiSlsDetector::freeSharedMemory() {
  // Detach Memory address
  for (int id=0; id<thisMultiDetector->numberOfDetectors; ++id) {
    if (detectors[id])
      detectors[id]->freeSharedMemory();
  }


  if (shmdt(thisMultiDetector) == -1) {
    perror("shmdt failed\n");
    return FAIL;
  }
#ifdef VERBOSE
  printf("Shared memory %d detached\n", shmId);
#endif
  // remove shared memory
  if (shmctl(shmId, IPC_RMID, 0) == -1) {
    perror("shmctl(IPC_RMID) failed\n");
    return FAIL;
  }
  printf("Shared memory %d deleted\n", shmId);
  return OK;

}



int multiSlsDetector::initSharedMemory(int id=0) {
  
  key_t     mem_key=DEFAULT_SHM_KEY+MAXDET+id;
  int       shm_id;
  int sz;



  sz=sizeof(sharedMultiSlsDetector);


#ifdef VERBOSE
  std::cout<<"multiSlsDetector: Size of shared memory is "<< sz << " - id " << mem_key << std::endl;
#endif
  shm_id = shmget(mem_key,sz,IPC_CREAT  | 0666); // allocate shared memory

  if (shm_id < 0) {
    std::cout<<"*** shmget error (server) ***"<< shm_id << std::endl;
    return shm_id;
  }
  
  /**
     thisMultiDetector pointer is set to the memory address of the shared memory
  */

  thisMultiDetector = (sharedMultiSlsDetector*) shmat(shm_id, NULL, 0);  /* attach */
  
  if (thisMultiDetector == (void*)-1) {
    std::cout<<"*** shmat error (server) ***" << std::endl;
    return shm_id;
  }
  /**
     shm_id returns -1 is shared memory initialization fails
  */ 

  return shm_id;

}




multiSlsDetector::multiSlsDetector(int id) :  slsDetectorUtils(), shmId(-1)
{

  while (shmId<0) {
    shmId=initSharedMemory(id);
    ++id;
  }
  --id;

  for (int id=0; id<MAXDET; ++id) {
    detectors[id]=NULL;
  }
  if (thisMultiDetector->alreadyExisting==0) {


    thisMultiDetector->onlineFlag = ONLINE_FLAG;
    thisMultiDetector->receiverOnlineFlag = OFFLINE_FLAG;
    thisMultiDetector->numberOfDetectors=0;
    for (int id=0; id<MAXDET; ++id) {
      thisMultiDetector->detectorIds[id]=-1;
      thisMultiDetector->offsetX[id]=0;
      thisMultiDetector->offsetY[id]=0;
    }
    thisMultiDetector->masterPosition=-1;
    thisMultiDetector->dataBytes=0;
    thisMultiDetector->numberOfChannels=0;
    thisMultiDetector->numberOfChannel[X]=0;
    thisMultiDetector->numberOfChannel[Y]=0;

    thisMultiDetector->maxNumberOfChannels=0;
    thisMultiDetector->maxNumberOfChannel[X]=0;
    thisMultiDetector->maxNumberOfChannel[Y]=0;

    thisMultiDetector->maxNumberOfChannelsPerDetector[X]=-1;
    thisMultiDetector->maxNumberOfChannelsPerDetector[Y]=-1;

    /** set trimDsdir, calDir and filePath to default to root directory*/
    strcpy(thisMultiDetector->filePath,"/");
    /** set fileName to default to run*/
    strcpy(thisMultiDetector->fileName,"run");
    /** set fileIndex to default to 0*/
    thisMultiDetector->fileIndex=0;
    /** set frames per file to default to 1*/
    thisMultiDetector->framesPerFile=1;
    /** set fileFormat to default to ascii*/
    thisMultiDetector->fileFormatType=ASCII;

    /** set progress Index to default to 0*/
    thisMultiDetector->progressIndex=0;
    /** set total number of frames to be acquired to default to 1*/
    thisMultiDetector->totalProgress=1;




    /** set correction mask to 0*/
    thisMultiDetector->correctionMask=1<<WRITE_FILE;
    thisMultiDetector->correctionMask|=(1<<OVERWRITE_FILE);
    /** set deat time*/
    thisMultiDetector->tDead=0;
    /** sets bad channel list file to none */
    strcpy(thisMultiDetector->badChanFile,"none");
    /** sets flat field correction directory */
    strcpy(thisMultiDetector->flatFieldDir,getenv("HOME"));
    /** sets flat field correction file */
    strcpy(thisMultiDetector->flatFieldFile,"none");
    /** set angular direction to 1*/
    thisMultiDetector->angDirection=1;
    /** set fine offset to 0*/
    thisMultiDetector->fineOffset=0;
    /** set global offset to 0*/
    thisMultiDetector->globalOffset=0;



    /** set threshold to -1*/
    thisMultiDetector->currentThresholdEV=-1;
    // /** set clockdivider to 1*/
    // thisMultiDetector->clkDiv=1;
    /** set number of positions to 0*/
    thisMultiDetector->numberOfPositions=0;
    /** sets angular conversion file to none */
    strcpy(thisMultiDetector->angConvFile,"none");
    /** set binsize*/
    thisMultiDetector->binSize=0.001;
    thisMultiDetector->stoppedFlag=0;
     
    thisMultiDetector->threadedProcessing=1;

    thisMultiDetector->actionMask=0;


    for (int ia=0; ia<MAX_ACTIONS; ++ia) {
      //thisMultiDetector->actionMode[ia]=0;
      strcpy(thisMultiDetector->actionScript[ia],"none");
      strcpy(thisMultiDetector->actionParameter[ia],"none");
    }


    for (int iscan=0; iscan<MAX_SCAN_LEVELS; ++iscan) {
       
      thisMultiDetector->scanMode[iscan]=0;
      strcpy(thisMultiDetector->scanScript[iscan],"none");
      strcpy(thisMultiDetector->scanParameter[iscan],"none");
      thisMultiDetector->nScanSteps[iscan]=0;
      thisMultiDetector->scanPrecision[iscan]=0;
    }

    thisMultiDetector->acquiringFlag = false;
    thisMultiDetector->receiver_upstream = false;
    thisMultiDetector->alreadyExisting=1;
  }


  //assigned before creating detector
  stoppedFlag=&thisMultiDetector->stoppedFlag;
  threadedProcessing=&thisMultiDetector->threadedProcessing;
  actionMask=&thisMultiDetector->actionMask;
  actionScript=thisMultiDetector->actionScript;		
  actionParameter=thisMultiDetector->actionParameter;      
  nScanSteps=thisMultiDetector->nScanSteps;
  scanMode=thisMultiDetector->scanMode;
  scanScript=thisMultiDetector->scanScript;
  scanParameter=thisMultiDetector->scanParameter;
  scanSteps=thisMultiDetector->scanSteps;
  scanPrecision=thisMultiDetector->scanPrecision;
  numberOfPositions=&thisMultiDetector->numberOfPositions;
  detPositions=thisMultiDetector->detPositions;
  angConvFile=thisMultiDetector->angConvFile;
  correctionMask=&thisMultiDetector->correctionMask;
  binSize=&thisMultiDetector->binSize;
  fineOffset=&thisMultiDetector->fineOffset;
  globalOffset=&thisMultiDetector->globalOffset;
  angDirection=&thisMultiDetector->angDirection;
  flatFieldDir=thisMultiDetector->flatFieldDir;
  flatFieldFile=thisMultiDetector->flatFieldFile;
  badChanFile=thisMultiDetector->badChanFile;
  timerValue=thisMultiDetector->timerValue;

  expTime=&timerValue[ACQUISITION_TIME];

  currentSettings=&thisMultiDetector->currentSettings;
  currentThresholdEV=&thisMultiDetector->currentThresholdEV;
  moveFlag=NULL;

  sampleDisplacement=thisMultiDetector->sampleDisplacement;

  filePath=thisMultiDetector->filePath;
  fileName=thisMultiDetector->fileName;
  fileIndex=&thisMultiDetector->fileIndex;
  framesPerFile=&thisMultiDetector->framesPerFile;
  fileFormatType=&thisMultiDetector->fileFormatType;


  for (int i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
#ifdef VERBOSE
    cout << thisMultiDetector->detectorIds[i] << endl;
#endif
    detectors[i]=new slsDetector(i, thisMultiDetector->detectorIds[i], this);


    //  setAngularConversionPointer(detectors[i]->getAngularConversionPointer(),detectors[i]->getNModsPointer(),detectors[i]->getNChans()*detectors[i]->getNChips(), i);

  }
 // for (int i=thisMultiDetector->numberOfDetectors; i<MAXDET; ++i)
   // detectors[i]=NULL;



  /** modifies the last PID accessing the detector system*/
  thisMultiDetector->lastPID=getpid();


  getNMods();
  getMaxMods();
  client_downstream = false;
  for(int i=0;i<MAXDET;++i)
	  zmqSocket[i] = 0;
  threadpool = 0;
	if(createThreadPool() == FAIL)
		exit(-1);

}

multiSlsDetector::~multiSlsDetector() {
	//removeSlsDetector();
	for(int i=0;i<MAXDET;++i) {
		if (zmqSocket[i]) {
			delete zmqSocket[i];
		}
	}
	destroyThreadPool();
}


int multiSlsDetector::createThreadPool(){
	if(threadpool)
		destroyThreadPool();
	int numthreads = thisMultiDetector->numberOfDetectors;
	if(numthreads < 1){
		numthreads = 1; //create threadpool anyway, threads initialized only when >1 detector added
	}
	threadpool = new ThreadPool(numthreads);
	switch(threadpool->initialize_threadpool()){
	case 0:
		cerr << "Failed to initialize thread pool!" << endl;
		return FAIL;
	case 1:
#ifdef VERBOSE
		cout << "Not initializing threads, not multi detector" << endl;
#endif
		break;
	default:
#ifdef VERBOSE
		cout << "Initialized Threadpool " << threadpool << endl;
#endif
		break;
	}
	return OK;
}

void multiSlsDetector::destroyThreadPool(){
	if(threadpool){
		delete threadpool;
		threadpool=0;
#ifdef VERBOSE
		cout<<"Destroyed Threadpool "<< threadpool << endl;
#endif
	}
}


int multiSlsDetector::addSlsDetector(int id, int pos) {
  int j=thisMultiDetector->numberOfDetectors;


  if (slsDetector::exists(id)==0) {
    cout << "Detector " << id << " does not exist - You should first create it to determine type etc." << endl;
  }
  
#ifdef VERBOSE
  cout << "Adding detector " << id << " in position " << pos << endl;
#endif

  if (pos<0)
    pos=j;

  if (pos>j)
    pos=thisMultiDetector->numberOfDetectors;
  


  //check that it is not already in the list
  
  for (int i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
    //check that it is not already in the list, in that case move to new position
    if (detectors[i]) {
      if (detectors[i]->getDetectorId()==id) { 
	cout << "Detector " << id << "already part of the multiDetector in position " << i << "!" << endl << "Remove it before adding it back in a new position!"<< endl;
	return -1;
      }
    }
  }
  


  if (pos!=thisMultiDetector->numberOfDetectors) {
    for (int ip=thisMultiDetector->numberOfDetectors-1; ip>=pos; --ip) {
#ifdef VERBOSE
      cout << "Moving detector " << thisMultiDetector->detectorIds[ip] << " from position " << ip << " to " << ip+1 << endl;
#endif
      thisMultiDetector->detectorIds[ip+1]=thisMultiDetector->detectorIds[ip];
      detectors[ip+1]=detectors[ip];
    }
  }
#ifdef VERBOSE
  cout << "Creating new detector " << pos << endl;
#endif

  detectors[pos]=new slsDetector(pos, id, this);
  thisMultiDetector->detectorIds[pos]=detectors[pos]->getDetectorId();
  ++thisMultiDetector->numberOfDetectors;
  
  thisMultiDetector->dataBytes+=detectors[pos]->getDataBytes();
 
  thisMultiDetector->numberOfChannels+=detectors[pos]->getTotalNumberOfChannels();
  thisMultiDetector->maxNumberOfChannels+=detectors[pos]->getMaxNumberOfChannels();



  getMaxMods();
  getNMods();
  getMaxMod(X);
  getNMod(X);
  getMaxMod(Y);
  getNMod(Y);

#ifdef VERBOSE
  cout << "Detector added " << thisMultiDetector->numberOfDetectors<< endl;

  for (int ip=0; ip<thisMultiDetector->numberOfDetectors; ++ip) {
    cout << "Detector " << thisMultiDetector->detectorIds[ip] << " position " << ip << " "  << detectors[ip]->getHostname() << endl;
  }
#endif

  //set offsets
  updateOffsets();
  if(createThreadPool() == FAIL)
	exit(-1);


  return thisMultiDetector->numberOfDetectors;

}


void multiSlsDetector::updateOffsets(){//cannot paralllize due to slsdetector calling this via parentdet->
#ifdef VERBOSE
	cout << endl << "Updating Multi-Detector Offsets" << endl;
#endif
	int offsetX=0, offsetY=0, numX=0, numY=0, maxX=0, maxY=0;
	int maxChanX = thisMultiDetector->maxNumberOfChannelsPerDetector[X];
	int maxChanY = thisMultiDetector->maxNumberOfChannelsPerDetector[Y];
	int prevChanX=0;
	int prevChanY=0;
	bool firstTime = true;

	thisMultiDetector->numberOfChannel[X] = 0;
	thisMultiDetector->numberOfChannel[Y] = 0;
	thisMultiDetector->maxNumberOfChannel[X] = 0;
	thisMultiDetector->maxNumberOfChannel[Y] = 0;


	for (int i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
		if (detectors[i]) {
#ifdef VERBOSE
			cout<<"offsetX:"<<offsetX<<" prevChanX:"<<prevChanX<<" offsetY:"<<offsetY<<" prevChanY:"<<prevChanY<<endl;
#endif
			//cout<<" totalchan:"<< detectors[i]->getTotalNumberOfChannels(Y) <<" maxChanY:"<<maxChanY<<endl;
			//incrementing in both direction
			if(firstTime){
				//incrementing in both directions
				firstTime = false;
				if((maxChanX > 0) && ((offsetX + detectors[i]->getTotalNumberOfChannels(X)) > maxChanX))
					cout<<"\nDetector[" << i << "] exceeds maximum channels allowed for complete detector set in X dimension!" << endl;
				if ((maxChanY > 0) && ((offsetY + detectors[i]->getTotalNumberOfChannels(Y)) > maxChanY))
					cout<<"\nDetector[" << i << "] exceeds maximum channels allowed for complete detector set in Y dimension!" << endl;
				prevChanX = detectors[i]->getTotalNumberOfChannels(X);
				prevChanY = detectors[i]->getTotalNumberOfChannels(Y);
				numX += detectors[i]->getTotalNumberOfChannels(X);
				numY += detectors[i]->getTotalNumberOfChannels(Y);
				maxX += detectors[i]->getMaxNumberOfChannels(X);
				maxY += detectors[i]->getMaxNumberOfChannels(Y);
#ifdef VERBOSE
				cout<<"incrementing in both direction"<<endl;
#endif
			}

			//incrementing in y direction
			else if ((maxChanY == -1) || ((maxChanY > 0) && ((offsetY + prevChanY + detectors[i]->getTotalNumberOfChannels(Y)) <= maxChanY))){
				offsetY += prevChanY;
				prevChanY = detectors[i]->getTotalNumberOfChannels(Y);
				numY += detectors[i]->getTotalNumberOfChannels(Y);
				maxY += detectors[i]->getMaxNumberOfChannels(Y);
#ifdef VERBOSE
				cout<<"incrementing in y direction"<<endl;
#endif
			}

			//incrementing in x direction
			else{
				if((maxChanX > 0) && ((offsetX + prevChanX + detectors[i]->getTotalNumberOfChannels(X)) > maxChanX))
					cout<<"\nDetector[" << i << "] exceeds maximum channels allowed for complete detector set in X dimension!" << endl;
				offsetY = 0;
				prevChanY = detectors[i]->getTotalNumberOfChannels(Y);;
				numY = 0; //assuming symmetry with this statement. whats on 1st column should be on 2nd column
				maxY = 0;
				offsetX += prevChanX;
				prevChanX = detectors[i]->getTotalNumberOfChannels(X);
				numX += detectors[i]->getTotalNumberOfChannels(X);
				maxX += detectors[i]->getMaxNumberOfChannels(X);
#ifdef VERBOSE
				cout<<"incrementing in x direction"<<endl;
#endif
			}

			thisMultiDetector->offsetX[i] = offsetX;
			thisMultiDetector->offsetY[i] = offsetY;
#ifdef VERBOSE
			cout << "Detector[" << i << "] has offsets (" << thisMultiDetector->offsetX[i] << ", " << thisMultiDetector->offsetY[i] << ")" << endl;
#endif
			//offsetY has been reset sometimes and offsetX the first time, but remember the highest values
			if(numX > thisMultiDetector->numberOfChannel[X])
				thisMultiDetector->numberOfChannel[X] = numX;
			if(numY > thisMultiDetector->numberOfChannel[Y])
				thisMultiDetector->numberOfChannel[Y] = numY;
			if(maxX > thisMultiDetector->maxNumberOfChannel[X])
				thisMultiDetector->maxNumberOfChannel[X] = maxX;
			if(maxY > thisMultiDetector->maxNumberOfChannel[Y])
				thisMultiDetector->maxNumberOfChannel[Y] = maxY;
		}
	}
#ifdef VERBOSE
	cout << "Number of Channels in X direction:" << thisMultiDetector->numberOfChannel[X] << endl;
	cout << "Number of Channels in Y direction:" << thisMultiDetector->numberOfChannel[Y] << endl << endl;
#endif
}

string multiSlsDetector::setHostname(const char* name, int pos){

  // int id=0;
  string s;
  if (pos>=0) {
    addSlsDetector(name, pos);
    if (detectors[pos])
      return detectors[pos]->getHostname();
  } else {
    size_t p1=0;
    s=string(name);
    size_t p2=s.find('+',p1);
    char hn[1000];
    if (p2==string::npos) {
      strcpy(hn,s.c_str());
      addSlsDetector(hn, pos);
    } else {
      while (p2!=string::npos) {
	strcpy(hn,s.substr(p1,p2-p1).c_str());
	addSlsDetector(hn, pos);
	s=s.substr(p2+1);
	p2=s.find('+');
      }
    }
  }
#ifdef VERBOSE
  cout << "-----------------------------set online!" << endl;
#endif
  setOnline(ONLINE_FLAG);

  return getHostname(pos);
}

string multiSlsDetector::ssetDetectorsType(string name, int pos) {


  // int id=0;
  string s;
  if (pos>=0) {
    if (getDetectorType(name)!=GET_DETECTOR_TYPE)
      addSlsDetector(name.c_str(), pos);
  } else {
    removeSlsDetector(); //reset detector list!
    size_t p1=0;
    s=string(name);
    size_t p2=s.find('+',p1);
    char hn[1000];
    if (p2==string::npos) {
      strcpy(hn,s.c_str());
      addSlsDetector(hn, pos);
    } else {
      while (p2!=string::npos) {
	strcpy(hn,s.substr(p1,p2-p1).c_str());
	if (getDetectorType(hn)!=GET_DETECTOR_TYPE)
	  addSlsDetector(hn, pos);
	s=s.substr(p2+1);
	p2=s.find('+');
      }
    }
  }
  return sgetDetectorsType(pos);

}

string multiSlsDetector::getHostname(int pos) {
  string s=string("");
#ifdef VERBOSE
  cout << "returning hostname" << pos << endl;
#endif
  if (pos>=0) {
    if (detectors[pos])
      return detectors[pos]->getHostname();
  } else {
    for (int ip=0; ip<thisMultiDetector->numberOfDetectors; ++ip) {
#ifdef VERBOSE
      cout << "detector " << ip << endl;
#endif
      if (detectors[ip]) {
	s+=detectors[ip]->getHostname();
	s+=string("+");
      }
#ifdef VERBOSE
      cout << s <<endl;
      cout << "hostname " << s << endl;
#endif
    }
  }
  return s;
  

}


slsDetectorDefs::detectorType multiSlsDetector::getDetectorsType(int pos) {

  detectorType s =GENERIC;
#ifdef VERBOSE
  cout << "returning type of detector with ID " << pos << endl;
#endif
  if (pos>=0) {
    if (detectors[pos])
      return detectors[pos]->getDetectorsType();
  } else if (detectors[0])
    return detectors[0]->getDetectorsType();
  return s;
}


string multiSlsDetector::sgetDetectorsType(int pos) {
  
  string s=string("");
#ifdef VERBOSE
  cout << "returning type" << pos << endl;
#endif
  if (pos>=0) {
    if (detectors[pos])
      return detectors[pos]->sgetDetectorsType();
  } else {
    for (int ip=0; ip<thisMultiDetector->numberOfDetectors; ++ip) {
#ifdef VERBOSE
      cout << "detector " << ip << endl;
#endif
      if (detectors[ip]) {
	s+=detectors[ip]->sgetDetectorsType();
	s+=string("+");
      }
#ifdef VERBOSE
      cout << "type " << s << endl;
#endif
    }
  }
  return s;
  
}




int multiSlsDetector::getDetectorId(int pos) {
  
#ifdef VERBOSE
  cout << "Getting detector ID " << pos << endl;
#endif

  if (pos>=0) {
    if (detectors[pos])
      return detectors[pos]->getDetectorId();
  } 
  return -1;
}



int multiSlsDetector::setDetectorId(int ival, int pos){

  if (pos>=0) {
    addSlsDetector(ival, pos);
    if (detectors[pos])
      return detectors[pos]->getDetectorId();
  } else {
    return -1;
  }
  return -1;
 
}


int multiSlsDetector::addSlsDetector(const char *name, int pos) {
  
  detectorType t=getDetectorType(string(name));
  int online=0;
  slsDetector *s=NULL;
  int id;
#ifdef VERBOSE
  cout << "Adding detector "<<name << " in position " << pos << endl;
#endif


  if (t==GENERIC) {
    for (int i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
      if (detectors[i]) {
	if (detectors[i]->getHostname()==string(name)) {
	  cout << "Detector " << name << "already part of the multiDetector in position " << i << "!" << endl<<  "Remove it before adding it back in a new position!"<< endl;
	  return -1;	
	}
      }
    }
   
    //checking that the detector doesn't already exists
    
    for (id=0; id<MAXDET; ++id) {
      if (slsDetector::exists(id)>0) {
#ifdef VERBOSE
	cout << "Detector " << id << " already exists" << endl;
#endif
	s=new slsDetector(pos, id, this);
	if (s->getHostname()==string(name))
	  break;
	delete s;
	s=NULL;
	//++id;
      }
    }

    if (s==NULL) {
      t=slsDetector::getDetectorType(name, DEFAULT_PORTNO);
      if (t==GENERIC) {
	cout << "Detector " << name << "does not exist in shared memory and could not connect to it to determine the type (which is not specified)!" << endl;
	setErrorMask(getErrorMask()|MULTI_DETECTORS_NOT_ADDED);
	appendNotAddedList(name);
	return -1;
      }
#ifdef VERBOSE
      else
	cout << "Detector type is " << t << endl;
#endif
      online=1;
    }
  } 
#ifdef VERBOSE
  else
    cout << "Adding detector by type " << getDetectorType(t) << endl;
#endif



  if (s==NULL) {
    for (id=0; id<MAXDET; ++id) {
      if (slsDetector::exists(id)==0) {
	break;
      }
    }
    
#ifdef VERBOSE
    cout << "Creating detector " << id << " of type "  << getDetectorType(t) << endl;
#endif
    s=new slsDetector(pos, t, id, this);
    if (online) {
      s->setTCPSocket(name);
      setOnline(ONLINE_FLAG);
    }
    delete s;
  }
#ifdef VERBOSE
  cout << "Adding it to the multi detector structure" << endl;
#endif
  return addSlsDetector(id, pos);


}


int multiSlsDetector::addSlsDetector(detectorType t, int pos) {

  int id;

  if (t==GENERIC) {
    return -1;
  }

  for (id=0; id<MAXDET; ++id) {
    if (slsDetector::exists(id)==0) {
      break;
    }
  }
    
#ifdef VERBOSE
  cout << "Creating detector " << id << " of type "  << getDetectorType(t) << endl;
#endif
  slsDetector *s=new slsDetector(pos, t, id, this);
  s=NULL;
#ifdef VERBOSE
  cout << "Adding it to the multi detector structure" << endl;
#endif

  return addSlsDetector(id, pos);

}



void multiSlsDetector::getNumberOfDetectors(int& nx, int& ny) {
	nx = 0; ny = 0;

	int offsetx = -1, offsety = -1;
	for (int i = 0; i < thisMultiDetector->numberOfDetectors; ++i) {
		if (thisMultiDetector->offsetX[i] > offsetx) {
			++nx;
			offsetx = thisMultiDetector->offsetX[i];
		}
		if (thisMultiDetector->offsetY[i] > offsety) {
			++ny;
			offsety = thisMultiDetector->offsetY[i];
		}
	}
}




int multiSlsDetector::getDetectorOffset(int pos, int &ox, int &oy) {
  ox=-1;
  oy=-1;
  int ret=FAIL;
  if (pos>=0 && pos<thisMultiDetector->numberOfDetectors) {
    if (detectors[pos]) {
      ox=thisMultiDetector->offsetX[pos];
      oy=thisMultiDetector->offsetY[pos];
      ret=OK;
    }
  }
  return ret;
}

int multiSlsDetector::setDetectorOffset(int pos, int ox, int oy) {
 
 
  int ret=FAIL;
 
  if (pos>=0 && pos<thisMultiDetector->numberOfDetectors) {
    if (detectors[pos]) {
      if (ox!=-1)
	thisMultiDetector->offsetX[pos]=ox;
      if (oy!=-1) 
	thisMultiDetector->offsetY[pos]=oy;
      ret=OK;
    }
  }
  return ret;
}



int multiSlsDetector::removeSlsDetector(char *name){
  for (int id=0; id<thisMultiDetector->numberOfDetectors; ++id) {
    if (detectors[id]) {						
      if (detectors[id]->getHostname()==string(name)) {			
	removeSlsDetector(id);						
      }									
    }									
  }									
  return thisMultiDetector->numberOfDetectors;
};




int multiSlsDetector::removeSlsDetector(int pos) {
  int j;
  
#ifdef VERBOSE
  cout << "Removing detector in position " << pos << endl;
#endif

  int mi=0, ma=thisMultiDetector->numberOfDetectors, single=0;

  if (pos>=0) {
    mi=pos;
    ma=pos+1;
    single=1;
  }

  //   if (pos<0 )
  //     pos=thisMultiDetector->numberOfDetectors-1;

  if (pos>=thisMultiDetector->numberOfDetectors)
    return thisMultiDetector->numberOfDetectors;

  //j=pos;
  for (j=mi; j<ma; ++j) {
    
    if (detectors[j]) {

      thisMultiDetector->dataBytes-=detectors[j]->getDataBytes();
      thisMultiDetector->numberOfChannels-=detectors[j]->getTotalNumberOfChannels();
      thisMultiDetector->maxNumberOfChannels-=detectors[j]->getMaxNumberOfChannels();

      delete detectors[j];
      detectors[j]=0;
      --thisMultiDetector->numberOfDetectors;


      if (single) {
	for (int i=j+1; i<thisMultiDetector->numberOfDetectors+1; ++i) {
	  detectors[i-1]=detectors[i];
	  thisMultiDetector->detectorIds[i-1]=thisMultiDetector->detectorIds[i];
	}
	detectors[thisMultiDetector->numberOfDetectors]=NULL;
	thisMultiDetector->detectorIds[thisMultiDetector->numberOfDetectors]=-1;
      }
    }
  }

  updateOffsets();
  if(createThreadPool() == FAIL)
	exit(-1);

  return thisMultiDetector->numberOfDetectors;
}







 
int multiSlsDetector::setMaster(int i) {

  int ret=-1, slave=0;
  masterFlags f;
#ifdef VERBOSE
  cout << "settin master in position " << i << endl;
#endif
  if (i>=0 && i<thisMultiDetector->numberOfDetectors) {
    if (detectors[i]) {
#ifdef VERBOSE
      cout << "detector position " << i << " ";
#endif
      thisMultiDetector->masterPosition=i;
      detectors[i]->setMaster(IS_MASTER);
      if(detectors[i]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<i));
    }
    for (int id=0; id<thisMultiDetector->numberOfDetectors; ++id) {
      if (i!=id) {
	if (detectors[id]) {
#ifdef VERBOSE
	  cout << "detector position " << id << " ";
#endif
	  detectors[id]->setMaster(IS_SLAVE);
	  if(detectors[id]->getErrorMask())
	    setErrorMask(getErrorMask()|(1<<id));

	}
      }
    }

  } else if (i==-2) {
    for (int id=0; id<thisMultiDetector->numberOfDetectors; ++id) {
      if (detectors[id]) {
#ifdef VERBOSE
	cout << "detector position " << id << " ";
#endif
	detectors[id]->setMaster(NO_MASTER);
	if(detectors[id]->getErrorMask())
	  setErrorMask(getErrorMask()|(1<<id));

      }
    }
    
  }

  // check return value

  for (int id=0; id<thisMultiDetector->numberOfDetectors; ++id) {
    if (detectors[id]) {
#ifdef VERBOSE
      cout << "detector position " << id << " ";
#endif
      f=detectors[id]->setMaster(GET_MASTER);
      if(detectors[id]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<id));

      switch (f) {
      case NO_MASTER:
	if (ret!=-1)
	  ret=-2;
	break;
      case IS_MASTER:
	if (ret==-1)
	  ret=id;
	else
	  ret=-2;
	break;
      case IS_SLAVE:
	slave=1;
	break;
      default:
	ret=-2;
      }
    }
  }
  if (slave>0 && ret<0)
    ret=-2;
  
  if (ret<0)
    ret=-1;
  
  thisMultiDetector->masterPosition=ret;

  return thisMultiDetector->masterPosition;
}

//   enum synchronyzationMode {
//     GET_SYNCHRONIZATION_MODE=-1, /**< the multidetector will return its synchronization mode */
//     NONE, /**< all detectors are independent (no cabling) */
//     MASTER_GATES, /**< the master gates the other detectors */
//     MASTER_TRIGGERS, /**< the master triggers the other detectors */
//     SLAVE_STARTS_WHEN_MASTER_STOPS /**< the slave acquires when the master finishes, to avoid deadtime */
//   }
  
/** 
    Sets/gets the synchronization mode of the various detectors
    \param sync syncronization mode
    \returns current syncronization mode
*/
slsDetectorDefs::synchronizationMode multiSlsDetector::setSynchronization(synchronizationMode sync) {

  
  synchronizationMode ret=GET_SYNCHRONIZATION_MODE, ret1=GET_SYNCHRONIZATION_MODE;
  
  for (int id=0; id<thisMultiDetector->numberOfDetectors; ++id) {
    if (detectors[id]) {
      ret1=detectors[id]->setSynchronization(sync);
      if(detectors[id]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<id));

      if (id==0)
	ret=ret1;
      else if (ret!=ret1)
	ret=GET_SYNCHRONIZATION_MODE;

    }
  }
  
  thisMultiDetector->syncMode=ret;

  return thisMultiDetector->syncMode;
  
}



























int multiSlsDetector::setOnline(int off) {

  if (off!=GET_ONLINE_FLAG) {
    thisMultiDetector->onlineFlag=off;

    int ret=-100;
	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return -1;
	}else{
		//return storage values
		int* iret[thisMultiDetector->numberOfDetectors];
		for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
			if(detectors[idet]){
				iret[idet]= new int(-1);
				Task* task = new Task(new func1_t<int,int>(&slsDetector::setOnline,
						detectors[idet],off,iret[idet]));
				threadpool->add_task(task);
			}
		}
		threadpool->startExecuting();
		threadpool->wait_for_tasks_to_complete();
		for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
			if(detectors[idet]){
				if(iret[idet] != NULL){
					if (ret==-100)
						ret=*iret[idet];
					else if (ret!=*iret[idet])
						ret=-1;
					delete iret[idet];
				}else ret=-1;
				if(detectors[idet]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<idet));
			}
		}
	}

	thisMultiDetector->onlineFlag=ret;
  }
  return thisMultiDetector->onlineFlag;
};



string multiSlsDetector::checkOnline() {
  string retval1 = "",retval;
  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      retval=detectors[idet]->checkOnline();
      if(!retval.empty()){
        retval1.append(retval);
    	retval1.append("+");
      }
    }
  }
  return retval1;
};



int multiSlsDetector::activate(int const enable){
	int i;
	int ret1=-100, ret;

	for (i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
		if (detectors[i]) {
			ret=detectors[i]->activate(enable);
			if(detectors[i]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<i));
			if (ret1==-100)
				ret1=ret;
			else if (ret!=ret1)
				ret1=-1;
		}
	}

	return ret1;
}



int multiSlsDetector::exists() {
  return thisMultiDetector->alreadyExisting;
}




// Initialization functions



  

int multiSlsDetector::getThresholdEnergy(int pos) {
 

  int i, posmin, posmax;
  int ret1=-100, ret;

  if (pos<0) {
    posmin=0;
    posmax=thisMultiDetector->numberOfDetectors;
  } else {
    posmin=pos;
    posmax=pos+1;
  }

  for (i=posmin; i<posmax; ++i) {
    if (detectors[i]) {
      ret=detectors[i]->getThresholdEnergy();
      if(detectors[i]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<i));
      if (ret1==-100)
	ret1=ret;
      else if (ret<(ret1-200) || ret>(ret1+200))
	ret1=-1;
      
    }
   
  }
  thisMultiDetector->currentThresholdEV=ret1;
  return ret1;


}  


int multiSlsDetector::setThresholdEnergy(int e_eV, int pos, detectorSettings isettings, int tb) {

	int posmin, posmax;
	int ret=-100;
	if (pos<0) {
		posmin=0;
		posmax=thisMultiDetector->numberOfDetectors;
	} else {
		posmin=pos;
		posmax=pos+1;
	}

	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return -1;
	}else{
		//return storage values
		int* iret[posmax-posmin];
		for(int idet=posmin; idet<posmax; ++idet){
			if(detectors[idet]){
				iret[idet]= new int(-1);
				Task* task = new Task(new func4_t<int,int,int,detectorSettings,int>(&slsDetector::setThresholdEnergy,
						detectors[idet],e_eV,-1,isettings,tb,iret[idet]));
				threadpool->add_task(task);
			}
		}
		threadpool->startExecuting();
		threadpool->wait_for_tasks_to_complete();
		for(int idet=posmin; idet<posmax; ++idet){
			if(detectors[idet]){
				if(iret[idet] != NULL){
					if (ret==-100)
						ret=*iret[idet];
					else if (*iret[idet]<(ret-200) || *iret[idet]>(ret+200))
							ret=-1;
					delete iret[idet];
				}else ret=-1;
				if(detectors[idet]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<idet));
			}
		}
	}

	thisMultiDetector->currentThresholdEV=ret;
	return ret;
}
 



slsDetectorDefs::detectorSettings multiSlsDetector::getSettings(int pos) {

  int posmin, posmax;
  int ret=-100;

  if (pos<0) {
    posmin=0;
    posmax=thisMultiDetector->numberOfDetectors;
  } else {
    posmin=pos;
    posmax=pos+1;
  }

	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return GET_SETTINGS;
	}else{
		//return storage values
		detectorSettings* iret[posmax-posmin];
		for(int idet=posmin; idet<posmax; ++idet){
			if(detectors[idet]){
				iret[idet]= new detectorSettings(GET_SETTINGS);
				Task* task = new Task(new func1_t<detectorSettings,int>(&slsDetector::getSettings,
						detectors[idet],-1,iret[idet]));
				threadpool->add_task(task);
			}
		}
		threadpool->startExecuting();
		threadpool->wait_for_tasks_to_complete();
		for(int idet=posmin; idet<posmax; ++idet){
			if(detectors[idet]){
				if(iret[idet] != NULL){
					if (ret==-100)
						ret=*iret[idet];
					else if (ret!=*iret[idet])
						ret=GET_SETTINGS;
					delete iret[idet];
				}else ret=GET_SETTINGS;
				if(detectors[idet]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<idet));
			}
		}
	}

  thisMultiDetector->currentSettings=(detectorSettings)ret;
  return (detectorSettings)ret;
}

slsDetectorDefs::detectorSettings multiSlsDetector::setSettings(detectorSettings isettings, int pos) {

	int posmin, posmax;
	int ret=-100;

	if (pos<0) {
		posmin=0;
		posmax=thisMultiDetector->numberOfDetectors;
	} else {
		posmin=pos;
		posmax=pos+1;
	}

	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return GET_SETTINGS;
	}else{
		//return storage values
		detectorSettings* iret[posmax-posmin];
		for(int idet=posmin; idet<posmax; ++idet){
			if(detectors[idet]){
				iret[idet]= new detectorSettings(GET_SETTINGS);
				Task* task = new Task(new func2_t<detectorSettings,detectorSettings,int>(&slsDetector::setSettings,
						detectors[idet],isettings,-1,iret[idet]));
				threadpool->add_task(task);
			}
		}
		threadpool->startExecuting();
		threadpool->wait_for_tasks_to_complete();
		for(int idet=posmin; idet<posmax; ++idet){
			if(detectors[idet]){
				if(iret[idet] != NULL){
					if (ret==-100)
						ret=*iret[idet];
					else if (ret!=*iret[idet])
						ret=GET_SETTINGS;
					delete iret[idet];
				}else ret=GET_SETTINGS;
				if(detectors[idet]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<idet));
			}
		}
	}

	thisMultiDetector->currentSettings=(detectorSettings)ret;
	return (detectorSettings)ret;

}



int multiSlsDetector::getChanRegs(double* retval,bool fromDetector){
  //nChansDet and currentNumChans is because of varying channel size per detector
  int n = thisMultiDetector->numberOfChannels,nChansDet,currentNumChans=0;
  double retval1[n];

  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      //  cout << "det " << idet << endl;
      nChansDet = detectors[idet]->getChanRegs(retval1,fromDetector);
      //   cout << "returned" << endl;
      if(detectors[idet]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<idet));
      //  cout << "memcopy "<< currentNumChans << " " << nChansDet << "(" << n << ")" << endl;

      memcpy(retval + (currentNumChans), retval1 , nChansDet*sizeof(double));
      currentNumChans += nChansDet;
      //  cout << "Done" << endl;
    }
  }
  return n;
}



















/* Communication to server */

int multiSlsDetector::prepareAcquisition(){

	int i=0;
	int ret=OK;
	int posmin=0, posmax=thisMultiDetector->numberOfDetectors;

	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return FAIL;
	}else{
		int* iret[posmax-posmin];
		for(int idet=posmin; idet<posmax; ++idet){
			if((idet!=thisMultiDetector->masterPosition) && (detectors[idet])){
				iret[idet]= new int(OK);
				Task* task = new Task(new func0_t<int>(&slsDetector::prepareAcquisition,
						detectors[idet],iret[idet]));
				threadpool->add_task(task);
			}
		}
		threadpool->startExecuting();
		threadpool->wait_for_tasks_to_complete();
		for(int idet=posmin; idet<posmax; ++idet){
			if((idet!=thisMultiDetector->masterPosition) && (detectors[idet])){
				if(iret[idet] != NULL){
					if(*iret[idet] != OK)
						ret = FAIL;
					delete iret[idet];
				}else  ret = FAIL;
				if(detectors[idet]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<idet));
			}
		}
	}

	//master
	int ret1=OK;
	i=thisMultiDetector->masterPosition;
	if (thisMultiDetector->masterPosition>=0) {
		if (detectors[i]) {
			ret1=detectors[i]->prepareAcquisition();
			if(detectors[i]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<i));
			if (ret1!=OK)
				ret=FAIL;
		}
	}

	return ret;

}



int multiSlsDetector::cleanupAcquisition(){
	int i=0;
	int ret=OK,ret1=OK;
	int posmin=0, posmax=thisMultiDetector->numberOfDetectors;

	i=thisMultiDetector->masterPosition;
	if (thisMultiDetector->masterPosition>=0) {
		if (detectors[i]) {
			ret1=detectors[i]->cleanupAcquisition();
			if(detectors[i]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<i));
			if (ret1!=OK)
				ret=FAIL;
		}
	}

	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return FAIL;
	}else{
		int* iret[posmax-posmin];
		for(int idet=posmin; idet<posmax; ++idet){
			if((idet!=thisMultiDetector->masterPosition) && (detectors[idet])){
				iret[idet]= new int(OK);
				Task* task = new Task(new func0_t<int>(&slsDetector::cleanupAcquisition,
						detectors[idet],iret[idet]));
				threadpool->add_task(task);
			}
		}
		threadpool->startExecuting();
		threadpool->wait_for_tasks_to_complete();
		for(int idet=posmin; idet<posmax; ++idet){
			if((idet!=thisMultiDetector->masterPosition) && (detectors[idet])){
				if(iret[idet] != NULL){
					if(*iret[idet] != OK)
						ret = FAIL;
					delete iret[idet];
				}else  ret = FAIL;
				if(detectors[idet]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<idet));
			}
		}
	}

	return ret;
}


// Acquisition functions
/* change these funcs accepting also ok/fail */

int multiSlsDetector::startAcquisition(){

	if (getDetectorsType() == EIGER) {
		if (prepareAcquisition() == FAIL)
			return FAIL;
	}

	int i=0;
	int ret=OK;
	int posmin=0, posmax=thisMultiDetector->numberOfDetectors;

	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return FAIL;
	}else{
		int* iret[posmax-posmin];
		for(int idet=posmin; idet<posmax; ++idet){
			if((idet!=thisMultiDetector->masterPosition) && (detectors[idet])){
				iret[idet]= new int(OK);
				Task* task = new Task(new func0_t<int>(&slsDetector::startAcquisition,
						detectors[idet],iret[idet]));
				threadpool->add_task(task);
			}
		}
		threadpool->startExecuting();
		threadpool->wait_for_tasks_to_complete();
		for(int idet=posmin; idet<posmax; ++idet){
			if((idet!=thisMultiDetector->masterPosition) && (detectors[idet])){
				if(iret[idet] != NULL){
					if(*iret[idet] != OK)
						ret = FAIL;
					delete iret[idet];
				}else  ret = FAIL;
				if(detectors[idet]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<idet));
			}
		}
	}

	//master
	int ret1=OK;
	i=thisMultiDetector->masterPosition;
	if (thisMultiDetector->masterPosition>=0) {
		if (detectors[i]) {
			ret1=detectors[i]->startAcquisition();
			if(detectors[i]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<i));
			if (ret1!=OK)
				ret=FAIL;
		}
	}
	return ret;
};




int multiSlsDetector::stopAcquisition(){
	pthread_mutex_lock(&mg); // locks due to processing thread using threadpool when in use
	int i=0;
	int ret=OK,ret1=OK;
	int posmin=0, posmax=thisMultiDetector->numberOfDetectors;

	i=thisMultiDetector->masterPosition;
	if (thisMultiDetector->masterPosition>=0) {
		if (detectors[i]) {
			ret1=detectors[i]->stopAcquisition();
			if(detectors[i]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<i));
			if (ret1!=OK)
				ret=FAIL;
		}
	}

	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return FAIL;
	}else{
		int* iret[posmax-posmin];
		for(int idet=posmin; idet<posmax; ++idet){
			if((idet!=thisMultiDetector->masterPosition) && (detectors[idet])){
				iret[idet]= new int(OK);
				Task* task = new Task(new func0_t<int>(&slsDetector::stopAcquisition,
						detectors[idet],iret[idet]));
				threadpool->add_task(task);
			}
		}
		threadpool->startExecuting();
		threadpool->wait_for_tasks_to_complete();
		for(int idet=posmin; idet<posmax; ++idet){
			if((idet!=thisMultiDetector->masterPosition) && (detectors[idet])){
				if(iret[idet] != NULL){
					if(*iret[idet] != OK)
						ret = FAIL;
					delete iret[idet];
				}else  ret = FAIL;
				if(detectors[idet]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<idet));
			}
		}
	}

	*stoppedFlag=1;
	pthread_mutex_unlock(&mg);
	return ret;
};




int multiSlsDetector::startReadOut(){

  int i=0;
  int ret=OK, ret1=OK;
  i=thisMultiDetector->masterPosition;
  if (i>=0) {
    if (detectors[i]) {
      ret=detectors[i]->startReadOut();
      if(detectors[i]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<i));
      if (ret!=OK)
	ret1=FAIL;
    }
  }
  for (i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
    if (detectors[i]) {
      ret=detectors[i]->startReadOut();
      if(detectors[i]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<i));
      if (ret!=OK)
	ret1=FAIL;
    }
  }

  return ret1;
  

};



int* multiSlsDetector::getDataFromDetector() {

	int nel=thisMultiDetector->dataBytes/sizeof(int);
	int n = 0;
	int* retval= NULL;
	int *retdet, *p=retval;
	int nodatadet=-1;
	int nodatadetectortype = false;
	detectorType types = getDetectorsType();
	if(types == EIGER || types == JUNGFRAU){
		nodatadetectortype = true;
	}

	if(!nodatadetectortype)
		retval=new int[nel];
	p=retval;
	//	cout << "multi: " << thisMultiDetector->dataBytes << endl;

	for (int id=0; id<thisMultiDetector->numberOfDetectors; ++id) {
		if (detectors[id]) {
			retdet=detectors[id]->getDataFromDetector(p);
			if(detectors[id]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<id));
			if(!nodatadetectortype){
				n=detectors[id]->getDataBytes();
				if (retdet) {;
#ifdef VERBOSE
					cout << "Detector " << id << " returned " << n << " bytes " << endl;
#endif
				} else {
					nodatadet=id;
#ifdef VERBOSE
					cout << "Detector " << id << " does not have data left " << endl;
#endif
				}
				p+=n/sizeof(int);
			}
		}
	}

	//eiger returns only null
	if(nodatadetectortype){
		return NULL;
	}

	if (nodatadet>=0) {
		for (int id=0; id<thisMultiDetector->numberOfDetectors; ++id) {
			if (id!=nodatadet) {
				if (detectors[id]) {
//#ifdef VERBOSE
					cout << "Stopping detector "<< id << endl;
//#endif
					detectors[id]->stopAcquisition();
					if(detectors[id]->getErrorMask())
						setErrorMask(getErrorMask()|(1<<id));

					while ((retdet=detectors[id]->getDataFromDetector())) {
						if(detectors[id]->getErrorMask())
							setErrorMask(getErrorMask()|(1<<id));

#ifdef VERBOSE
						cout << "Detector "<< id << " still sent data " << endl;
#endif
						delete [] retdet;
					}
					if(detectors[id]->getErrorMask())
						setErrorMask(getErrorMask()|(1<<id));

				}
			}
		}
		delete [] retval;
		return NULL;
	}

	return retval;
};



int* multiSlsDetector::readFrame(){
  int nel=thisMultiDetector->dataBytes/sizeof(int);
  int n;
  int* retval=new int[nel];
  int *retdet, *p=retval;
  
  /** probably it's always better to have one integer per channel in any case! */

  for (int id=0; id<thisMultiDetector->numberOfDetectors; ++id) {
    if (detectors[id]) {
      retdet=detectors[id]->readFrame();
      if(detectors[id]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<id));
      if (retdet) {
	n=detectors[id]->getDataBytes();
	if(detectors[id]->getErrorMask())
	  setErrorMask(getErrorMask()|(1<<id));
	memcpy(p,retdet,n);
	delete [] retdet;
	p+=n/sizeof(int);
      } else {
#ifdef VERBOSE
	cout << "Detector " << id << " does not have data left " << endl;
#endif
	delete [] retval;
	return NULL;
      }
    }
  }
  dataQueue.push(retval);
  return retval;

};



int* multiSlsDetector::readAll(){
  
  /** Thread for each detector?!?!?! */

  // int fnum=F_READ_ALL;
  int* retval; // check what we return!
  // int ret=OK, ret1=OK;

  int i=0;
#ifdef VERBOSE
  std::cout<< "Reading all frames "<< std::endl;
#endif
  if (thisMultiDetector->onlineFlag==ONLINE_FLAG) {
    
    for (int id=0; id<thisMultiDetector->numberOfDetectors; ++id) {
      if (detectors[id]) {
	detectors[id]->readAllNoWait();
	if(detectors[id]->getErrorMask())
	  setErrorMask(getErrorMask()|(1<<id));
      }
    }
    while ((retval=getDataFromDetector())){
      ++i;
#ifdef VERBOSE
      std::cout<< i << std::endl;
      //#else
      //std::cout << "-" << flush;
#endif
      dataQueue.push(retval);
    }
    for (int id=0; id<thisMultiDetector->numberOfDetectors; ++id) {
      if (detectors[id]) {
	detectors[id]->disconnectControl();
      }
    }  
 
  }

#ifdef VERBOSE
  std::cout<< "received "<< i<< " frames" << std::endl;
  //#else
  // std::cout << std::endl; 
#endif
  return dataQueueFront(); // check what we return!

};

int* multiSlsDetector::startAndReadAll(){

  /** Thread for each detector?!?!?! */
#ifdef VERBOSE
  cout << "Start and read all " << endl;
#endif


  int* retval;
  int i=0;
  if (thisMultiDetector->onlineFlag==ONLINE_FLAG) {

	  if(getDetectorsType() == EIGER) {
		  if (prepareAcquisition() == FAIL)
			  return NULL;
	  }
	  startAndReadAllNoWait();
   
    while ((retval=getDataFromDetector())){
      ++i;
#ifdef VERBOSE
      std::cout<< i << std::endl;
      //#else
      //std::cout << "-" << flush;
#endif
      dataQueue.push(retval);
    }

    for (int id=0; id<thisMultiDetector->numberOfDetectors; ++id) {
      if (detectors[id]) {
	detectors[id]->disconnectControl();
      }
    }
 
  }

#ifdef VERBOSE
  std::cout<< "MMMM recieved "<< i<< " frames" << std::endl;
  //#else
  // std::cout << std::endl; 
#endif
  return dataQueueFront(); // check what we return!

  
};


int multiSlsDetector::startAndReadAllNoWait(){
	pthread_mutex_lock(&mg); // locks due to processing thread using threadpool when in use
	int i=0;
	int ret=OK;
	int posmin=0, posmax=thisMultiDetector->numberOfDetectors;

	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return FAIL;
	}else{
		int* iret[posmax-posmin];
		for(int idet=posmin; idet<posmax; ++idet){
			if((idet!=thisMultiDetector->masterPosition) && (detectors[idet])){
				iret[idet]= new int(OK);
				Task* task = new Task(new func0_t<int>(&slsDetector::startAndReadAllNoWait,
						detectors[idet],iret[idet]));
				threadpool->add_task(task);
			}
		}
		threadpool->startExecuting();
		threadpool->wait_for_tasks_to_complete();
		for(int idet=posmin; idet<posmax; ++idet){
			if((idet!=thisMultiDetector->masterPosition) && (detectors[idet])){
				if(iret[idet] != NULL){
					if(*iret[idet] != OK)
						ret = FAIL;
					delete iret[idet];
				}else  ret = FAIL;
				if(detectors[idet]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<idet));
			}
		}
	}

	//master
	int ret1=OK;
	i=thisMultiDetector->masterPosition;
	if (thisMultiDetector->masterPosition>=0) {
		if (detectors[i]) {
			ret1=detectors[i]->startAndReadAllNoWait();
			if(detectors[i]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<i));
			if (ret1!=OK)
				ret=FAIL;
		}
	}
	pthread_mutex_unlock(&mg);
	return ret;

}


/**
   get run status
   \returns status mask
*/
slsDetectorDefs::runStatus  multiSlsDetector::getRunStatus() {

  runStatus s = IDLE,s1 = IDLE;
  if (thisMultiDetector->masterPosition>=0)
    if (detectors[thisMultiDetector->masterPosition]){
      s = detectors[thisMultiDetector->masterPosition]->getRunStatus();
      if(detectors[thisMultiDetector->masterPosition]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<thisMultiDetector->masterPosition));
      return s;
    }

  for (int i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
    s1=detectors[i]->getRunStatus(); 
    if(detectors[i]->getErrorMask())
      setErrorMask(getErrorMask()|(1<<i));
    if (s1==ERROR) {
      return ERROR;
    }
    if (s1!=IDLE)
    	s = s1;
   // if (s1==IDLE && s!=IDLE)
   //   s=ERROR;
    
  }
  return s;
}


int* multiSlsDetector::popDataQueue() {
  int *retval=NULL;
  if( !dataQueue.empty() ) {
    retval=dataQueue.front();
    dataQueue.pop();
  }
  return retval;
}

detectorData* multiSlsDetector::popFinalDataQueue() {
  detectorData *retval=NULL;
  if( !finalDataQueue.empty() ) {
    retval=finalDataQueue.front();
    finalDataQueue.pop();
  }
  return retval;
}

void multiSlsDetector::resetDataQueue() {
  int *retval=NULL;
  while( !dataQueue.empty() ) {
    retval=dataQueue.front();
    dataQueue.pop();
    delete [] retval;
  }
}

void multiSlsDetector::resetFinalDataQueue() {
  detectorData *retval=NULL;
  while( !finalDataQueue.empty() ) {
    retval=finalDataQueue.front();
    finalDataQueue.pop();
    delete retval;
  }

}



/* 
   set or read the acquisition timers 
   enum timerIndex {
   FRAME_NUMBER,
   ACQUISITION_TIME,
   FRAME_PERIOD,
   DELAY_AFTER_TRIGGER,
   GATES_NUMBER,
   PROBES_NUMBER
   CYCLES_NUMBER,
   GATE_INTEGRATED_TIME
   }
 */
int64_t multiSlsDetector::setTimer(timerIndex index, int64_t t){

	int64_t ret=-100;

	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return -1;
	}else{
		//return storage values
		int64_t* iret[thisMultiDetector->numberOfDetectors];
		for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
			if(detectors[idet]){
				iret[idet]= new int64_t(-1);
				Task* task = new Task(new func2_t<int64_t,timerIndex,int64_t>(&slsDetector::setTimer,
						detectors[idet],index,t,iret[idet]));
				threadpool->add_task(task);
			}
		}
		threadpool->startExecuting();
		threadpool->wait_for_tasks_to_complete();
		for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
			if(detectors[idet]){
				if(iret[idet] != NULL){
					if (ret==-100)
						ret=*iret[idet];
					else if (ret!=*iret[idet])
						ret=-1;
					delete iret[idet];
				}else ret=-1;
				if(detectors[idet]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<idet));
			}
		}
	}

	if (index==SAMPLES_JCTB)
		setDynamicRange();

	thisMultiDetector->timerValue[index]=ret;
	return ret;
}



 
int64_t multiSlsDetector::getTimeLeft(timerIndex index){
  int i;
  int64_t ret1=-100, ret;
  

  if (thisMultiDetector->masterPosition>=0)
    if (detectors[thisMultiDetector->masterPosition]){
      ret1 = detectors[thisMultiDetector->masterPosition]->getTimeLeft(index); 
      if(detectors[thisMultiDetector->masterPosition]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<thisMultiDetector->masterPosition));
      return ret1;
    }
  
  for (i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
    if (detectors[i]) {
      ret=detectors[i]->getTimeLeft(index);
      if(detectors[i]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<i));
      if (ret1==-100)
	ret1=ret;
      else if (ret!=ret1)
	ret1=-1;
    }
  }
  
  return ret1;
  
}



int multiSlsDetector::setSpeed(speedVariable index, int value){
  int i;
  int ret1=-100, ret;
  

  
  for (i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
    if (detectors[i]) {
      ret=detectors[i]->setSpeed(index,value);
      if(detectors[i]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<i));
      if (ret1==-100)
	ret1=ret;
      else if (ret!=ret1)
	ret1=FAIL;
    }
  }
  
  return ret1;
  
}













int  multiSlsDetector::getDataBytes(){
  int ret=0;   
  for (int ip=0; ip<thisMultiDetector->numberOfDetectors; ++ip) {
    if (detectors[ip]) 
      ret+=detectors[ip]->getDataBytes();
  }
  return ret;
}






// Flags
int multiSlsDetector::setDynamicRange(int n, int pos){

  //  cout << "multi "  << endl;
  int imi, ima, i;
  int ret, ret1=-100;

  if (pos<0) {
    imi=0;
    ima=thisMultiDetector->numberOfDetectors;
  } else {
    imi=pos;
    ima=pos+1;
  }
 
  for (i=imi; i<ima; ++i) {
    //  cout << "multi ************ detector " << i << endl;

    if (detectors[i]) {
      thisMultiDetector->dataBytes-=detectors[i]->getDataBytes();
      ret=detectors[i]->setDynamicRange(n);
      if(detectors[i]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<i));
      if (ret1==-100)
	ret1=ret;
      else if (ret!=ret1)
	ret1=FAIL;
      thisMultiDetector->dataBytes+=detectors[i]->getDataBytes();
    }
  }

  //for usability for the user
  if (getDetectorsType() == EIGER){
	  if(n == 32){
		  std::cout << "Setting Clock to Quarter Speed to cope with Dynamic Range of 32" << std::endl;
		  setSpeed(CLOCK_DIVIDER,2);
	  }
	  else if(n == 16){
		  std::cout << "Setting Clock to Half Speed for Dynamic Range of 16" << std::endl;
		  setSpeed(CLOCK_DIVIDER,1);
	  }
  }

  return thisMultiDetector->dataBytes;
};




void multiSlsDetector::verifyMinMaxROI(int n, ROI r[]){
	int temp;
	for(int i=0;i<n;++i){
		if ((r[i].xmax) < (r[i].xmin)){
			temp=r[i].xmax;
			r[i].xmax=r[i].xmin;
			r[i].xmin=temp;

		}
		if ((r[i].ymax) < (r[i].ymin)){
			temp=r[i].ymax;
			r[i].ymax=r[i].ymin;
			r[i].ymin=temp;
		}
	}
}



int multiSlsDetector::decodeNChannel(int offsetX, int offsetY, int &channelX, int &channelY){
	channelX=-1;
	channelY=-1;
	//loop over
	for(int i=0;i<thisMultiDetector->numberOfDetectors;++i){
		if (detectors[i]) {
			//check x offset range
			if ((offsetX >= thisMultiDetector->offsetX[i]) && (offsetX < (thisMultiDetector->offsetX[i]+detectors[i]->getMaxNumberOfChannels(X)))){
				if(offsetY==-1){
					channelX = offsetX - thisMultiDetector->offsetX[i];
					return i;
				}else{
					//check y offset range
					if((offsetY >= thisMultiDetector->offsetY[i]) && (offsetY< (thisMultiDetector->offsetY[i]+detectors[i]->getMaxNumberOfChannels(Y)))){
						channelX = offsetX - thisMultiDetector->offsetX[i];
						channelY = offsetY - thisMultiDetector->offsetY[i];
						return i;
					}
				}
			}
		}
	}
	return -1;
}


int multiSlsDetector::setROI(int n,ROI roiLimits[]){
	int ret1=-100,ret;
	int i,xmin,xmax,ymin,ymax,channelX,channelY,idet,lastChannelX,lastChannelY,index,offsetX,offsetY;
	bool invalidroi=false;
	int ndet = thisMultiDetector->numberOfDetectors;
	ROI allroi[ndet][n];
	int nroi[ndet];
	for(i=0;i<ndet;++i) nroi[i]=0;


	if ((n < 0) || (roiLimits == NULL))
		return FAIL;

	//ensures min < max
	verifyMinMaxROI(n,roiLimits);
#ifdef VERBOSE
	cout<<"Setting ROI for "<< n << "rois:"<<endl;
	for(i=0;i<n;++i)
		cout<<i<<":"<<roiLimits[i].xmin<<"\t"<<roiLimits[i].xmax<<"\t"<<roiLimits[i].ymin<<"\t"<<roiLimits[i].ymax<<endl;
#endif
	//for each roi
	for(i=0;i<n;++i){
		xmin = roiLimits[i].xmin;
		xmax = roiLimits[i].xmax;
		ymin = roiLimits[i].ymin;
		ymax = roiLimits[i].ymax;

		//check roi max values
		idet = decodeNChannel(xmax,ymax,channelX,channelY);
#ifdef VERBOSE
		cout<<"Decoded Channel max vals: "<<endl;
		cout<<"det:"<<idet<<"\t"<<xmax<<"\t"<<ymax<<"\t"<<channelX<<"\t"<<channelY<<endl;
#endif
		if (idet == -1){
			cout << "invalid roi" << endl;
			continue;
		}

		//split in x dir
		while (xmin <= xmax){
			invalidroi=false;
			ymin = roiLimits[i].ymin;
			//split in y dir
			while (ymin <= ymax){
				//get offset for each detector
				idet = decodeNChannel(xmin,ymin,channelX,channelY);
#ifdef VERBOSE
				cout<<"Decoded Channel min vals: "<<endl;
				cout<<"det:"<<idet<<"\t"<<xmin<<"\t"<<ymin<<"\t"<<channelX<<"\t"<<channelY<<endl;
#endif
				if (idet == -1){
					cout << "invalid roi" << endl;
					invalidroi = true;
					break;
				}
				if(detectors[idet]){
					//get last channel for each det in x and y dir
					lastChannelX = (detectors[idet]->getMaxNumberOfChannels(X))-1;
					lastChannelY = (detectors[idet]->getMaxNumberOfChannels(Y))-1;

					offsetX = thisMultiDetector->offsetX[idet];
					offsetY = thisMultiDetector->offsetY[idet];
					//at the end in x dir
					if ((offsetX + lastChannelX) >= xmax)
						lastChannelX = xmax - offsetX;
					//at the end in y dir
					if ((offsetY + lastChannelY) >= ymax)
						lastChannelY = ymax - offsetY;

#ifdef VERBOSE
					cout<<"lastChannelX:"<<lastChannelX<<"\t"<<"lastChannelY:"<<lastChannelY<<endl;
#endif

					//creating the list of roi for corresponding detector
					index = nroi[idet];
					allroi[idet][index].xmin = channelX;
					allroi[idet][index].xmax = lastChannelX;
					allroi[idet][index].ymin = channelY;
					allroi[idet][index].ymax = lastChannelY;
					nroi[idet] = nroi[idet]+1;

					ymin = lastChannelY + offsetY + 1;
					if ((lastChannelY + offsetY) == ymax)
						ymin = ymax + 1;

#ifdef VERBOSE
					cout<<"nroi[idet]:"<<nroi[idet]<<"\tymin:"<<ymin<<endl;
#endif

				}
			}
			if(invalidroi) break;

			xmin = lastChannelX + offsetX + 1;
			if ((lastChannelX + offsetX) == xmax)
				xmin = xmax + 1;
		}
	}

#ifdef VERBOSE
		cout<<"Setting ROI :"<<endl;
		for(i=0;i<thisMultiDetector->numberOfDetectors;++i){
			cout<<"detector "<<i<<endl;
			for(int j=0;j<nroi[i];++j){
				cout<<allroi[i][j].xmin<<"\t"<<allroi[i][j].xmax<<"\t"<<allroi[i][j].ymin<<"\t"<<allroi[i][j].ymax<<endl;
			}
		}
#endif

	//settings the rois for each detector
	for (i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
		if (detectors[i]){
#ifdef VERBOSE
			cout << "detector " << i << ":" << endl;
#endif
			ret = detectors[i]->setROI(nroi[i],allroi[i]);
		    if(detectors[i]->getErrorMask())
			  setErrorMask(getErrorMask()|(1<<i));
			if(ret1==-100)
				ret1=ret;
			else
				ret1=FAIL;
		}
	}

	return ret1;
}



slsDetectorDefs::ROI* multiSlsDetector::getROI(int &n){

	n = 0;
	int num = 0,i,j;
	int ndet = thisMultiDetector->numberOfDetectors;
	int maxroi = ndet*MAX_ROIS;
	ROI temproi;
	ROI roiLimits[maxroi];
	ROI* retval = new ROI[maxroi];
	ROI* temp=0;
	int index=0;

	//get each detector's roi array
	for (i=0; i<thisMultiDetector->numberOfDetectors; ++i){
		if (detectors[i]){
			temp = detectors[i]->getROI(index);
			if(detectors[i]->getErrorMask())
			  setErrorMask(getErrorMask()|(1<<i));

			if(temp){
//#ifdef VERBOSE
				if(index)
					cout << "detector " << i << ":" << endl;
//#endif
				for(j=0;j<index;++j){
//#ifdef VERBOSE
					cout<<temp[j].xmin<<"\t"<<temp[j].xmax<<"\t"<<temp[j].ymin<<"\t"<<temp[j].ymax<<endl;
//#endif
					roiLimits[n].xmin = temp[j].xmin + thisMultiDetector->offsetX[i];
					roiLimits[n].xmax = temp[j].xmax + thisMultiDetector->offsetX[i];
					roiLimits[n].ymin = temp[j].ymin + thisMultiDetector->offsetY[i];
					roiLimits[n].ymax = temp[j].ymin + thisMultiDetector->offsetY[i];
					++n;
				}
			}
		}
	}



	//empty roi
	if (!n)	return NULL;



#ifdef VERBOSE
	cout<<"ROI :"<<endl;
		for(int j=0;j<n;++j){
			cout<<roiLimits[j].xmin<<"\t"<<roiLimits[j].xmax<<"\t"<<roiLimits[j].ymin<<"\t"<<roiLimits[j].ymax<<endl;
		}
#endif




	//combine all the adjacent rois in x direction
	for(i=0;i<n;++i){
		//since the ones combined are replaced by -1
		if ((roiLimits[i].xmin) == -1)
			continue;
		for(j=i+1;j<n;++j){
			//since the ones combined are replaced by -1
			if ((roiLimits[j].xmin) == -1)
				continue;
			//if y values are same
			if (((roiLimits[i].ymin) == (roiLimits[j].ymin)) && ((roiLimits[i].ymax) == (roiLimits[j].ymax))){
				//if adjacent, increase [i] range and replace all [j] with -1
				if ((roiLimits[i].xmax)+1 == roiLimits[j].xmin){
					roiLimits[i].xmax = roiLimits[j].xmax;
					roiLimits[j].xmin = -1;
					roiLimits[j].xmax = -1;
					roiLimits[j].ymin = -1;
					roiLimits[j].ymax = -1;
				}
				//if adjacent, increase [i] range and replace all [j] with -1
				else if ((roiLimits[i].xmin)-1 == roiLimits[j].xmax){
					roiLimits[i].xmin = roiLimits[j].xmin;
					roiLimits[j].xmin = -1;
					roiLimits[j].xmax = -1;
					roiLimits[j].ymin = -1;
					roiLimits[j].ymax = -1;
				}
			}
		}
	}

#ifdef VERBOSE
	cout<<"Combined along x axis Getting ROI :"<<endl;
		cout<<"detector "<<i<<endl;
		for(int j=0;j<n;++j){
			cout<<roiLimits[j].xmin<<"\t"<<roiLimits[j].xmax<<"\t"<<roiLimits[j].ymin<<"\t"<<roiLimits[j].ymax<<endl;
		}
#endif

	//combine all the adjacent rois in y direction
	for(i=0;i<n;++i){
		//since the ones combined are replaced by -1
		if ((roiLimits[i].ymin) == -1)
			continue;
		for(j=i+1;j<n;++j){
			//since the ones combined are replaced by -1
			if ((roiLimits[j].ymin) == -1)
				continue;
			//if x values are same
			if (((roiLimits[i].xmin) == (roiLimits[j].xmin)) && ((roiLimits[i].xmax) == (roiLimits[j].xmax))){
				//if adjacent, increase [i] range and replace all [j] with -1
				if ((roiLimits[i].ymax)+1 == roiLimits[j].ymin){
					roiLimits[i].ymax = roiLimits[j].ymax;
					roiLimits[j].xmin = -1;
					roiLimits[j].xmax = -1;
					roiLimits[j].ymin = -1;
					roiLimits[j].ymax = -1;
				}
				//if adjacent, increase [i] range and replace all [j] with -1
				else if ((roiLimits[i].ymin)-1 == roiLimits[j].ymax){
					roiLimits[i].ymin = roiLimits[j].ymin;
					roiLimits[j].xmin = -1;
					roiLimits[j].xmax = -1;
					roiLimits[j].ymin = -1;
					roiLimits[j].ymax = -1;
				}
			}
		}
	}

	// get rid of -1s
	for(i=0;i<n;++i){
		if((roiLimits[i].xmin)!=-1){
			retval[num] = roiLimits[i];
			++num;
		}
	}
	//sort final roi
	for(i=0;i<num;++i){
		for(j=i+1;j<num;++j){
			if(retval[j].xmin<retval[i].xmin){
				temproi = retval[i];
				retval[i] = retval[j];
				retval[j] = temproi;
			}
		}
	}
	n = num;

	cout<<"\nxmin\txmax\tymin\tymax"<<endl;
	for(i=0;i<n;++i)
		cout<<retval[i].xmin<<"\t"<<retval[i].xmax<<"\t"<<retval[i].ymin<<"\t"<<retval[i].ymax<<endl;
	return retval;
}



double* multiSlsDetector::decodeData(int *datain, int &nn, double *fdata) {
  double *dataout;

  if (fdata)
    dataout=fdata;
  else {
    if (detectors[0]->getDetectorsType()==JUNGFRAUCTB) {
      nn=thisMultiDetector->dataBytes/2;
      dataout=new double[nn];
    } else {
      nn=thisMultiDetector->numberOfChannels;
      dataout=new double[nn];
    }
  }

  // int ich=0;
  int n;
  double *detp=dataout;
  int  *datap=datain;

  for (int i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
    if (detectors[i]) {
      detectors[i]->decodeData(datap, n, detp);
      if(detectors[i]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<i));
#ifdef VERBOSE
      cout << "increment pointers " << endl;
#endif   
      datap+=detectors[i]->getDataBytes()/sizeof(int);
      detp+=n;
      // if (detectors[0]->getDetectorsType()==JUNGFRAUCTB) {
      // 	detp+=detectors[i]->getDataBytes()/2;
      // } else {
      // 	detp+=detectors[i]->getTotalNumberOfChannels();
      // }
#ifdef VERBOSE
      cout << "done " << endl;
#endif
      //       for (int j=0; j<detectors[i]->getTotalNumberOfChannels(); ++j) {
      // 	dataout[ich]=detp[j];
      // 	++ich;
      //       }
      //delete [] detp;
    }
  }


  return dataout;
}
 
//Correction
/*
  enum correctionFlags {
  DISCARD_BAD_CHANNELS,
  AVERAGE_NEIGHBOURS_FOR_BAD_CHANNELS,
  FLAT_FIELD_CORRECTION,
  RATE_CORRECTION,
  ANGULAR_CONVERSION
  }
*/


///////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////














int multiSlsDetector::setFlatFieldCorrection(string fname){
  double* data = new double[thisMultiDetector->numberOfChannels];//  xmed[thisMultiDetector->numberOfChannels];
  double* ffcoefficients = new double[thisMultiDetector->numberOfChannels];
  double*fferrors = new double[thisMultiDetector->numberOfChannels];
  // int nmed=0;
  // int idet=0, ichdet=-1;
  char ffffname[MAX_STR_LENGTH*2];
  int  nch;//nbad=0,
  //int badlist[MAX_BADCHANS];
  //int im=0;

  if (fname=="default") {
    fname=string(thisMultiDetector->flatFieldFile);
  }	
  

  thisMultiDetector->correctionMask&=~(1<<FLAT_FIELD_CORRECTION);
  
  
  
  if (fname=="") {
#ifdef VERBOSE
    std::cout<< "disabling flat field correction" << std::endl;
#endif
    thisMultiDetector->correctionMask&=~(1<<FLAT_FIELD_CORRECTION);
    //  strcpy(thisMultiDetector->flatFieldFile,"none");
    for (int i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
      if (detectors[i]){
	detectors[i]->setFlatFieldCorrection(NULL, NULL);
	if(detectors[i]->getErrorMask())
	  setErrorMask(getErrorMask()|(1<<i));
      }
    }
  } else { 
#ifdef VERBOSE
    std::cout<< "Setting flat field correction from file " << fname << std::endl;
#endif
    sprintf(ffffname,"%s/%s",thisMultiDetector->flatFieldDir,fname.c_str());
    nch=readDataFile(string(ffffname),data);
    
    if (nch>thisMultiDetector->numberOfChannels)
      nch=thisMultiDetector->numberOfChannels;
    
    if (nch>0) {
      
      //???? bad ff chans?
      int nm=getNMods();
      int chpm[nm];
      int mMask[nm];
      for (int i=0; i<nm; ++i) {
	chpm[i]=getChansPerMod(i);
	mMask[i]=i;
	//	cout << "multi chpm " << im << " " << chpm[im] << endl;
      }
      fillModuleMask(mMask);
      // cout << "multi chpm0 " << chpm[0] << endl;
      fillBadChannelMask();
      if ((postProcessingFuncs::calculateFlatField(&nm, chpm, mMask, badChannelMask, data, ffcoefficients, fferrors))>=0) {
	strcpy(thisMultiDetector->flatFieldFile,fname.c_str());
	
	
	thisMultiDetector->correctionMask|=(1<<FLAT_FIELD_CORRECTION);
	
	setFlatFieldCorrection(ffcoefficients, fferrors);
	
      } else
	std::cout<< "Calculated flat field from file " << fname << " is not valid " << nch << std::endl;
    } else {
      std::cout<< "Flat field from file " << fname << " is not valid " << nch << std::endl;
    }
    
  }
  return thisMultiDetector->correctionMask&(1<<FLAT_FIELD_CORRECTION);
}

int multiSlsDetector::fillModuleMask(int *mM) {
  int imod=0, off=0;
  if (mM) {
    for (int i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
      if (detectors[i]) {
	for (int im=0; im<detectors[i]->getNMods(); ++im) {
	  mM[imod]=im+off;
	  ++imod;
	}
	off+=detectors[i]->getMaxMods();
      }
      
      
    }
  }
  return getNMods();
}



int multiSlsDetector::setFlatFieldCorrection(double *corr, double *ecorr) {
  int ichdet=0;
  double *p, *ep;
  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      if (corr!=NULL)
	p=corr+ichdet;
      else
	p=NULL;
      if (ecorr!=NULL)
	ep=ecorr+ichdet;
      else
	ep=NULL;
      detectors[idet]->setFlatFieldCorrection(p, ep);
      if(detectors[idet]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<idet));
      ichdet+=detectors[idet]->getTotalNumberOfChannels();
    }
  }
  return 0;
}









int multiSlsDetector::getFlatFieldCorrection(double *corr, double *ecorr) {
  int ichdet=0;
  double *p, *ep;
  int ff=1, dff;
  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      if (corr!=NULL)
	p=corr+ichdet;
      else
	p=NULL;
      if (ecorr!=NULL)
	ep=ecorr+ichdet;
      else
	ep=NULL;
      dff=detectors[idet]->getFlatFieldCorrection(p, ep);
      if(detectors[idet]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<idet));
      if (dff==0)
	ff=0;
      ichdet+=detectors[idet]->getTotalNumberOfChannels();
    }
  }
  return ff;
}















int multiSlsDetector::getNMods(){
  int nm=0;
  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      nm+=detectors[idet]->getNMods();
    }
  }
#ifdef VERBOSE
  cout << "total number of modules is " << nm << endl;
#endif

  return nm;
}


int multiSlsDetector::getNMod(dimension d){
  int nm=0;
  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      nm+=detectors[idet]->getNMod(d);
    }
  }
#ifdef VERBOSE
  cout << "total number of modules in dimension " << d << " is " << nm << endl;
#endif

  return nm;
}



int multiSlsDetector::getChansPerMod(int imod){
  int id=-1, im=-1;
#ifdef VERBOSE
  cout << "get chans per mod " << imod << endl;
#endif
  decodeNMod(imod, id, im);
  if (id >=0) {
    if (detectors[id]) {
      return detectors[id]->getChansPerMod(im);
    }
  }
  return -1;

}

int  multiSlsDetector::getMoveFlag(int imod){
  int id=-1, im=-1;
  decodeNMod(imod, id, im);
  if (id>=0) {
    if (detectors[id]) {
      return detectors[id]->getMoveFlag(im);
    }
  }
  //default!!!
  return 1;
}




angleConversionConstant * multiSlsDetector::getAngularConversionPointer(int imod){
  int id=-1, im=-1;
#ifdef VERBOSE
  cout << "get angular conversion pointer " << endl;
#endif
  if (decodeNMod(imod, id, im)>=0) {
    if (detectors[id]) {
      return detectors[id]->getAngularConversionPointer(im);
    }
  }
  return NULL;

}



int multiSlsDetector::flatFieldCorrect(double* datain, double *errin, double* dataout, double *errout){

  int ichdet=0;
  double *perr=errin;//*pdata,
  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
#ifdef VERBOSE
      cout << " detector " << idet << " offset " << ichdet << endl;
#endif
      if (errin)
	perr+=ichdet;
      detectors[idet]->flatFieldCorrect(datain+ichdet, perr, dataout+ichdet, errout+ichdet);
      if(detectors[idet]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<idet));
      ichdet+=detectors[idet]->getTotalNumberOfChannels();//detectors[idet]->getNChans()*detectors[idet]->getNChips()*detectors[idet]->getNMods();
    }
  }
  return 0;
};








int multiSlsDetector::setRateCorrection(double t){

#ifdef VERBOSE
	std::cout<< "Setting rate correction with dead time "<< thisMultiDetector->tDead << std::endl;
#endif
	int ret=OK;
	int posmax=thisMultiDetector->numberOfDetectors;

	// eiger return value is ok/fail
	if (getDetectorsType() == EIGER){
		if(!threadpool){
			cout << "Error in creating threadpool. Exiting" << endl;
			return FAIL;
		}else{
			int* iret[posmax];
			for(int idet=0; idet<posmax; ++idet){
				if(detectors[idet]){
					iret[idet]= new int(OK);
					Task* task = new Task(new func1_t<int,double>(&slsDetector::setRateCorrection,
							detectors[idet],t,iret[idet]));
					threadpool->add_task(task);
				}
			}
			threadpool->startExecuting();
			threadpool->wait_for_tasks_to_complete();
			for(int idet=0; idet<posmax; ++idet){
				if(detectors[idet]){
					if(iret[idet] != NULL){
						if(*iret[idet] != OK)
							ret = FAIL;
						delete iret[idet];
					}else  ret = FAIL;
					if(detectors[idet]->getErrorMask())
						setErrorMask(getErrorMask()|(1<<idet));
				}
			}
		}
		return ret;
	}


	// mythen, others
	if (t==0) {
		thisMultiDetector->correctionMask&=~(1<<RATE_CORRECTION);
		return thisMultiDetector->correctionMask&(1<<RATE_CORRECTION);
	} else
		thisMultiDetector->correctionMask|=(1<<RATE_CORRECTION);

	ret = -100;
	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return -1;
	}else{
		int* iret[posmax];
		for(int idet=0; idet<posmax; ++idet){
			if(detectors[idet]){
				iret[idet]= new int(-1);
				Task* task = new Task(new func1_t<int,double>(&slsDetector::setRateCorrection,
						detectors[idet],t,iret[idet]));
				threadpool->add_task(task);
			}
		}
		threadpool->startExecuting();
		threadpool->wait_for_tasks_to_complete();
		for(int idet=0; idet<posmax; ++idet){
			if(detectors[idet]){
				if(iret[idet] != NULL)
					delete iret[idet];
				if(detectors[idet]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<idet));
			}
		}
	}
	return thisMultiDetector->correctionMask&(1<<RATE_CORRECTION);
}


int multiSlsDetector::getRateCorrection(double &t){

	if (getDetectorsType() == EIGER){
		t = getRateCorrectionTau();
		return t;
	}

	if (thisMultiDetector->correctionMask&(1<<RATE_CORRECTION)) {
#ifdef VERBOSE
		std::cout<< "Rate correction is enabled with dead time "<< thisMultiDetector->tDead << std::endl;
#endif
		return 1;
	} else
		t=0;
#ifdef VERBOSE
	std::cout<< "Rate correction is disabled " << std::endl;
#endif
	return 0;
};

double multiSlsDetector::getRateCorrectionTau(){

	double ret=-100.0;
	int posmax = thisMultiDetector->numberOfDetectors;

	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return -1;
	}else{
		double* iret[posmax];
		for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
			if(detectors[idet]){
				iret[idet]= new double(-1);
				Task* task = new Task(new func0_t<double>(&slsDetector::getRateCorrectionTau,
						detectors[idet],iret[idet]));
				threadpool->add_task(task);
			}
		}
		threadpool->startExecuting();
		threadpool->wait_for_tasks_to_complete();

		for(int idet=0; idet<posmax; ++idet){
			if(detectors[idet]){
				if(iret[idet] != NULL){
					if(ret == -100.0)
						ret = *iret[idet];
					else if ((ret - *iret[idet]) > 0.000000001) {
						std::cout<< "Rate correction is different for different readouts " << std::endl;
						ret=-1;
					}
					delete iret[idet];
				}else  ret = -1;
				if(detectors[idet]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<idet));
			}
		}
	}

	if (getDetectorsType() == EIGER)
		return ret;


	//only mythen
	if (thisMultiDetector->correctionMask&(1<<RATE_CORRECTION)) {
#ifdef VERBOSE
		std::cout<< "Rate correction is enabled with dead time "<< thisMultiDetector->tDead << std::endl;
#endif
	} else {
#ifdef VERBOSE
		std::cout<< "Rate correction is disabled " << std::endl;
#endif
		ret=0;
	}
	return ret;

};



int multiSlsDetector::getRateCorrection(){

	if (getDetectorsType() == EIGER){
		return getRateCorrectionTau();
	}

  if (thisMultiDetector->correctionMask&(1<<RATE_CORRECTION)) {
    return 1;
  } else
    return 0;
};




int multiSlsDetector::rateCorrect(double* datain, double *errin, double* dataout, double *errout){

  int ichdet=0;
  double *perr=errin;
  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      if (errin)
	perr+=ichdet;
      detectors[idet]->rateCorrect(datain+ichdet, perr, dataout+ichdet, errout+ichdet);
      if(detectors[idet]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<idet));
      ichdet+=detectors[idet]->getTotalNumberOfChannels();
    }
  }
  return 0;
};


int multiSlsDetector::setBadChannelCorrection(string fname){

  int badlist[MAX_BADCHANS];// badlistdet[MAX_BADCHANS];
  int nbad=0;//, nbaddet=0, choff=0, idet=0;
  int ret=0;

  cout << thisMultiDetector->badChanFile << endl;

  if (fname=="default")
    fname=string(thisMultiDetector->badChanFile);
  
  
  


  ret=setBadChannelCorrection(fname, nbad, badlist);
  //#ifdef VERBOSE
  cout << "multi: file contained " << ret << " badchans" << endl; 
  //#endif
  if (ret==0) {
    thisMultiDetector->correctionMask&=~(1<<DISCARD_BAD_CHANNELS);
    nbad=0;
  } else {
    thisMultiDetector->correctionMask|=(1<<DISCARD_BAD_CHANNELS);
    strcpy(thisMultiDetector->badChanFile,fname.c_str());
  }

  return setBadChannelCorrection(nbad,badlist,0);

}



int multiSlsDetector::setBadChannelCorrection(int nbad, int *badlist, int ff) {

  //#define VERBOSE
  
  int  badlistdet[MAX_BADCHANS];
  int nbaddet=0, choff=0, idet=0;
  if (nbad<1)
    badlistdet[0]=0;
  else
    badlistdet[0]=badlist[0];

  if (nbad>0) {
    thisMultiDetector->correctionMask|=(1<<DISCARD_BAD_CHANNELS);
    
    for (int ich=0; ich<nbad; ++ich) {
      if (detectors[idet]) {
	if ((badlist[ich]-choff)>=detectors[idet]->getMaxNumberOfChannels()) {
	  //#ifdef VERBOSE
	  cout << "setting " << nbaddet << " badchans to detector " << idet << endl;
	  //#endif
	  detectors[idet]->setBadChannelCorrection(nbaddet,badlistdet,0);
	  if(detectors[idet]->getErrorMask())
	    setErrorMask(getErrorMask()|(1<<idet));
	  choff+=detectors[idet]->getMaxNumberOfChannels();
	  nbaddet=0;
	  ++idet;
	  if (detectors[idet]==NULL)
	    break;
	}
	badlistdet[nbaddet]=(badlist[ich]-choff);
	++nbaddet;
#ifdef VERBOSE
	cout << nbaddet << " " << badlist[ich] << " " << badlistdet[nbaddet-1] << endl;
#endif
      }
    }
    if (nbaddet>0) {
      
      if (detectors[idet]) {
#ifdef VERBOSE
	cout << "setting " << nbaddet << " badchans to detector " << idet << endl;
#endif
	detectors[idet]->setBadChannelCorrection(nbaddet,badlistdet,0);
	if(detectors[idet]->getErrorMask())
	  setErrorMask(getErrorMask()|(1<<idet));
	choff+=detectors[idet]->getMaxNumberOfChannels();
	nbaddet=0;
	++idet;
      }
    }
    nbaddet=0;
    for (int i=idet; i<thisMultiDetector->numberOfDetectors; ++i) {
 #ifdef VERBOSE
      cout << "setting " << 0 << " badchans to detector " << i << endl;
 #endif
      if (detectors[i]) {
	detectors[i]->setBadChannelCorrection(nbaddet,badlistdet,0);
	if(detectors[i]->getErrorMask())
	  setErrorMask(getErrorMask()|(1<<i));
       }
    }
    
  } else {
    nbaddet=0;
    for (int i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
      if (detectors[idet]) {
#ifdef VERBOSE
	cout << "setting " << 0 << " badchans to detector " << idet << endl;
#endif
	detectors[idet]->setBadChannelCorrection(nbaddet,badlistdet,0);
	if(detectors[idet]->getErrorMask())
	  setErrorMask(getErrorMask()|(1<<idet));
       }
    }
    thisMultiDetector->correctionMask&=~(1<<DISCARD_BAD_CHANNELS);
  }  
#ifdef VERBOSE 
  cout << (thisMultiDetector->correctionMask&(1<<DISCARD_BAD_CHANNELS)) << endl;
#endif
  return thisMultiDetector->correctionMask&(1<<DISCARD_BAD_CHANNELS);

}






int multiSlsDetector::readAngularConversionFile(string fname) {

  
  ifstream infile;
  //int nm=0;
  infile.open(fname.c_str(), ios_base::in);
  if (infile.is_open()) {

    for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
      if (detectors[idet]) {
#ifdef VERBOSE
	cout << " detector " << idet << endl;
#endif
	detectors[idet]->readAngularConversion(infile);
	if(detectors[idet]->getErrorMask())
	  setErrorMask(getErrorMask()|(1<<idet));
       }
    }
    infile.close();
  } else {
    std::cout<< "Could not open calibration file "<< fname << std::endl;
    return -1;
  }
  return 0;

}


int multiSlsDetector::writeAngularConversion(string fname) {

  
  ofstream outfile;
  //  int nm=0;
  outfile.open(fname.c_str(), ios_base::out);
  if (outfile.is_open()) {

    for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
      if (detectors[idet]) {
	detectors[idet]->writeAngularConversion(outfile);
	if(detectors[idet]->getErrorMask())
	  setErrorMask(getErrorMask()|(1<<idet));
       }
    }
    outfile.close();
  } else {
    std::cout<< "Could not open calibration file "<< fname << std::endl;
    return -1;
  }
  return 0;

}

int  multiSlsDetector::getAngularConversion(int &direction,  angleConversionConstant *angconv) {

  int dir=-100, dir1;
  angleConversionConstant *a1=angconv;

  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      detectors[idet]->getAngularConversion(dir1,a1);
      if(detectors[idet]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<idet));
       if (dir==-100)
	dir = dir1;
      if (dir!=dir1)
	dir=0;
      if (angconv) {
	a1+=detectors[idet]->getNMods();
      }
    }
  }
  direction=dir;
  
  if (thisMultiDetector->correctionMask&(1<< ANGULAR_CONVERSION)) {
    return 1;
  } 
  return 0;
 

}



dacs_t multiSlsDetector::setDAC(dacs_t val, dacIndex idac, int mV, int imod) {
	dacs_t ret = -100;

	// single
	{
		int id=-1, im=-1;
		if (decodeNMod(imod, id, im)>=0) {
			if(detectors[id]){
				ret = detectors[id]->setDAC(val, idac, mV, im);
				if(detectors[id]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<id));
				return ret;
			}
			return -1;
		}
	}


	// multi
	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return -1;
	}

	int posmin=0, posmax=thisMultiDetector->numberOfDetectors;
	dacs_t* iret[posmax-posmin];

	for(int idet=posmin; idet<posmax; ++idet){
		if(detectors[idet]){
			iret[idet]= new dacs_t(-1);
			Task* task = new Task(new func4_t<dacs_t,dacs_t,dacIndex,int,int>(&slsDetector::setDAC,
					detectors[idet],val, idac, mV, imod, iret[idet]));
			threadpool->add_task(task);
		}
	}
	threadpool->startExecuting();
	threadpool->wait_for_tasks_to_complete();
	for(int idet=posmin; idet<posmax; ++idet){
		if(detectors[idet]){
			if(iret[idet] != NULL){

				// highvoltage of slave, ignore value
				if ((idac == HV_NEW) && (*iret[idet] == -999))
					;
				else {
					if (ret==-100)
						ret=*iret[idet];
					else if (ret!=*iret[idet])
						ret=-1;
				}
				delete iret[idet];
			}else ret=-1;
			if(detectors[idet]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<idet));
		}
	}
	if (ret==-100)
		ret = -1;

	return ret;
}

dacs_t multiSlsDetector::getADC(dacIndex idac, int imod) {
	dacs_t ret = -100;

	// single
	{
		int id=-1, im=-1;
		if (decodeNMod(imod, id, im)>=0) {
			if(detectors[id]){
				ret = detectors[id]->getADC(idac, im);
				if(detectors[id]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<id));
				return ret;
			}
			return -1;
		}
	}


	// multi
	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return -1;
	}

	int posmin=0, posmax=thisMultiDetector->numberOfDetectors;
	dacs_t* iret[posmax-posmin];

	for(int idet=posmin; idet<posmax; ++idet){
		if(detectors[idet]){
			iret[idet]= new dacs_t(-1);
			Task* task = new Task(new func2_t<dacs_t,dacIndex,int>(&slsDetector::getADC,
					detectors[idet],idac, imod, iret[idet]));
			threadpool->add_task(task);
		}
	}
	threadpool->startExecuting();
	threadpool->wait_for_tasks_to_complete();
	for(int idet=posmin; idet<posmax; ++idet){
		if(detectors[idet]){
			if(iret[idet] != NULL){
				if (ret==-100)
					ret=*iret[idet];
				else if (ret!=*iret[idet])
					ret=-1;
				delete iret[idet];
			}else ret=-1;
			if(detectors[idet]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<idet));
		}
	}

	return ret;
}



int multiSlsDetector::setThresholdTemperature(int val, int imod) {
    int ret = -100;

    // single
    {
        int id=-1, im=-1;
        if (decodeNMod(imod, id, im)>=0) {
            if(detectors[id]){
                ret = detectors[id]->setThresholdTemperature(val, im);
                if(detectors[id]->getErrorMask())
                    setErrorMask(getErrorMask()|(1<<id));
                return ret;
            }
            return -1;
        }
    }

    // multi
    if(!threadpool){
        cout << "Error in creating threadpool. Exiting" << endl;
        return -1;
    }

    int posmin=0, posmax=thisMultiDetector->numberOfDetectors;
    int* iret[posmax-posmin];

    for(int idet=posmin; idet<posmax; ++idet){
        if(detectors[idet]){
            iret[idet]= new dacs_t(-1);
            Task* task = new Task(new func2_t<int,int,int>(&slsDetector::setThresholdTemperature,
                    detectors[idet], val, imod, iret[idet]));
            threadpool->add_task(task);
        }
    }
    threadpool->startExecuting();
    threadpool->wait_for_tasks_to_complete();
    for(int idet=posmin; idet<posmax; ++idet){
        if(detectors[idet]){
            if(iret[idet] != NULL){
                if (ret==-100)
                    ret=*iret[idet];
                else if (ret!=*iret[idet])
                    ret=-1;
                delete iret[idet];
            }else ret=-1;
            if(detectors[idet]->getErrorMask())
                setErrorMask(getErrorMask()|(1<<idet));
        }
    }

    return ret;
}


int multiSlsDetector::setTemperatureControl(int val, int imod) {
    int ret = -100;

    // single
    {
        int id=-1, im=-1;
        if (decodeNMod(imod, id, im)>=0) {
            if(detectors[id]){
                ret = detectors[id]->setTemperatureControl(val, im);
                if(detectors[id]->getErrorMask())
                    setErrorMask(getErrorMask()|(1<<id));
                return ret;
            }
            return -1;
        }
    }

    // multi
    if(!threadpool){
        cout << "Error in creating threadpool. Exiting" << endl;
        return -1;
    }

    int posmin=0, posmax=thisMultiDetector->numberOfDetectors;
    int* iret[posmax-posmin];

    for(int idet=posmin; idet<posmax; ++idet){
        if(detectors[idet]){
            iret[idet]= new dacs_t(-1);
            Task* task = new Task(new func2_t<int,int,int>(&slsDetector::setTemperatureControl,
                    detectors[idet], val, imod, iret[idet]));
            threadpool->add_task(task);
        }
    }
    threadpool->startExecuting();
    threadpool->wait_for_tasks_to_complete();
    for(int idet=posmin; idet<posmax; ++idet){
        if(detectors[idet]){
            if(iret[idet] != NULL){
                if (ret==-100)
                    ret=*iret[idet];
                else if (ret!=*iret[idet])
                    ret=-1;
                delete iret[idet];
            }else ret=-1;
            if(detectors[idet]->getErrorMask())
                setErrorMask(getErrorMask()|(1<<idet));
        }
    }

    return ret;
}



int multiSlsDetector::setTemperatureEvent(int val, int imod) {
    int ret = -100;

    // single
    {
        int id=-1, im=-1;
        if (decodeNMod(imod, id, im)>=0) {
            if(detectors[id]){
                ret = detectors[id]->setTemperatureEvent(val, im);
                if(detectors[id]->getErrorMask())
                    setErrorMask(getErrorMask()|(1<<id));
                return ret;
            }
            return -1;
        }
    }

    // multi
    if(!threadpool){
        cout << "Error in creating threadpool. Exiting" << endl;
        return -1;
    }

    int posmin=0, posmax=thisMultiDetector->numberOfDetectors;
    int* iret[posmax-posmin];

    for(int idet=posmin; idet<posmax; ++idet){
        if(detectors[idet]){
            iret[idet]= new dacs_t(-1);
            Task* task = new Task(new func2_t<int,int,int>(&slsDetector::setTemperatureEvent,
                    detectors[idet], val, imod, iret[idet]));
            threadpool->add_task(task);
        }
    }
    threadpool->startExecuting();
    threadpool->wait_for_tasks_to_complete();
    for(int idet=posmin; idet<posmax; ++idet){
        if(detectors[idet]){
            if(iret[idet] != NULL){
                if (ret==-100)
                    ret=*iret[idet];
                else if (ret!=*iret[idet])
                    ret=-1;
                delete iret[idet];
            }else ret=-1;
            if(detectors[idet]->getErrorMask())
                setErrorMask(getErrorMask()|(1<<idet));
        }
    }

    return ret;
}




int multiSlsDetector::setChannel(int64_t reg, int ichan, int ichip, int imod) {
  int ret, ret1=-100;
  int id=-1, im=-1;
  int dmi=0, dma=thisMultiDetector->numberOfDetectors;
  
  if (decodeNMod(imod, id, im)>=0) {
    dmi=id;
    dma=id+1;
  }
  for (int idet=dmi; idet<dma; ++idet) {
    if (detectors[idet]) {
      ret=detectors[idet]->setChannel(reg, ichan, ichip, im);
      if(detectors[idet]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<idet));
      if (ret1==-100)
	ret1=ret;
      else if (ret!=ret1)
	ret1=-1;
    }
  }
  return ret1;
   
}



/**
   sets the value of s angular conversion parameter
   \param c can be ANGULAR_DIRECTION, GLOBAL_OFFSET, FINE_OFFSET, BIN_SIZE
   \param v the value to be set
   \returns the actual value
*/

double multiSlsDetector::setAngularConversionParameter(angleConversionParameter c, double v) {
  double ret=slsDetectorUtils::setAngularConversionParameter(c,v);
  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    
    if (detectors[idet]) {
	
      detectors[idet]->setAngularConversionParameter(c,v);
      if(detectors[idet]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<idet));

    }
  }
  return ret;
}



// double* multiSlsDetector::convertAngles(double pos) {
//   double *ang=new double[thisMultiDetector->numberOfChannels];

//   double *p=ang;
//   int choff=0;

//   for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    
//     if (detectors[idet]) {
// #ifdef EPICS
//       //  cout << "convert angle det " << idet << endl;
//       if (idet<2)
// #endif
// 	p=detectors[idet]->convertAngles(pos);
// #ifdef EPICS
//       else //////////// GOOD ONLY AT THE BEAMLINE!!!!!!!!!!!!!
// 	p=detectors[idet]->convertAngles(0);
// #endif
//       for (int ich=0; ich<detectors[idet]->getTotalNumberOfChannels(); ich++) {
// 	ang[choff+ich]=p[ich];
//       }
//       choff+=detectors[idet]->getTotalNumberOfChannels();
//       delete [] p;
//     }
//   }
//   return ang;
// }


int multiSlsDetector::getBadChannelCorrection(int *bad) {
  //int ichan;
  int *bd, nd, ntot=0, choff=0;;
  
  if (((thisMultiDetector->correctionMask)&(1<< DISCARD_BAD_CHANNELS))==0)
    return 0;
  //else
  // cout << "bad chans corr enabled "<< thisMultiDetector->correctionMask << endl;

  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      nd=detectors[idet]->getBadChannelCorrection();
      //  cout << "det " << idet << " nbad " << nd << endl;
      if (nd>0) {
	bd = new int[nd];
	nd=detectors[idet]->getBadChannelCorrection(bd);
	if(detectors[idet]->getErrorMask())
	  setErrorMask(getErrorMask()|(1<<idet));

	for (int id=0; id<nd; ++id) {
	  if (bd[id]<detectors[idet]->getTotalNumberOfChannels()) {
	    if (bad) bad[ntot]=choff+bd[id];
	    ++ntot;
	  }
	}
	choff+=detectors[idet]->getTotalNumberOfChannels();
	delete [] bd;
      } else
	ntot+=nd;
      
    }
  }
  return ntot;

}


int multiSlsDetector::exitServer() {

  int ival=FAIL, iv;
  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      iv=detectors[idet]->exitServer();
      if (iv==OK)
	ival=iv;
    }
  }
  return ival;
}


/** returns the detector trimbit/settings directory  */
char* multiSlsDetector::getSettingsDir() {
  string s0="", s1="", s;
  

  //char ans[1000];
  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      s=detectors[idet]->getSettingsDir();
      if(detectors[idet]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<idet));

      if (s0=="")
	s0=s;
      else
	s0+=string("+")+s;
      if (s1=="")
	s1=s;
      else if (s1!=s)
	s1="bad";
    }
  }
  if (s1=="bad")
    strcpy(ans,s0.c_str());
  else
    strcpy(ans,s1.c_str());
  return ans;
}



/** sets the detector trimbit/settings directory  \sa sharedSlsDetector */
char* multiSlsDetector::setSettingsDir(string s){

  if (s.find('+')==string::npos) {
    for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
      if (detectors[idet]) {
	detectors[idet]->setSettingsDir(s);
      if(detectors[idet]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<idet));
      }
    }
  } else {
    size_t p1=0;
    size_t p2=s.find('+',p1);
    int id=0;
    while (p2!=string::npos) {

      if (detectors[id]) {
	detectors[id]->setSettingsDir(s.substr(p1,p2-p1));
       if(detectors[id]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<id));
     }
      ++id;
      s=s.substr(p2+1);
      p2=s.find('+');
      if (id>=thisMultiDetector->numberOfDetectors)
	break;
    }

  }
  return getSettingsDir();


}




int multiSlsDetector::setTrimEn(int ne, int *ene) {

  int ret=-100, ret1;

  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      ret1=detectors[idet]->setTrimEn(ne,ene);
      if(detectors[idet]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<idet));
     if (ret==-100)
	ret=ret1;
      else if (ret!=ret1)
	ret=-1;
    }
  }
  return ret;

}



int multiSlsDetector::getTrimEn(int *ene) {

  int ret=-100, ret1;

  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      ret1=detectors[idet]->getTrimEn(ene);
      if(detectors[idet]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<idet));
     if (ret==-100)
	ret=ret1;
      else if (ret!=ret1)
	ret=-1;
    }
  }
  return ret;

}









/**
   returns the location of the calibration files
   \sa  sharedSlsDetector
*/
char* multiSlsDetector::getCalDir() {
  string s0="", s1="", s;
  //char ans[1000];
  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      s=detectors[idet]->getCalDir();
      if(detectors[idet]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<idet));

      if (s0=="")
	s0=s;
      else
	s0+=string("+")+s;
      if (s1=="")
	s1=s;
      else if (s1!=s)
	s1="bad";
    }
  }
  if (s1=="bad")
    strcpy(ans,s0.c_str());
  else
    strcpy(ans,s1.c_str());
  return ans;
}


/**
   sets the location of the calibration files
   \sa  sharedSlsDetector
*/
char* multiSlsDetector::setCalDir(string s){

  if (s.find('+')==string::npos) {
    for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
      if (detectors[idet]) {
	detectors[idet]->setCalDir(s);
        if(detectors[idet]->getErrorMask())
	  setErrorMask(getErrorMask()|(1<<idet));
      }
    }
  } else {
    size_t p1=0;
    size_t p2=s.find('+',p1);
    int id=0;
    while (p2!=string::npos) {

      if (detectors[id]) {
	detectors[id]->setCalDir(s.substr(p1,p2-p1));
	if(detectors[id]->getErrorMask())
	  setErrorMask(getErrorMask()|(1<<id));
      }
      ++id;
      s=s.substr(p2+1);
      p2=s.find('+');
      if (id>=thisMultiDetector->numberOfDetectors)
	break;
    }

  }
  return getCalDir();

} 

/**
   returns the location of the calibration files
   \sa  sharedSlsDetector
*/
string multiSlsDetector::getNetworkParameter(networkParameter p) {
  string s0="", s1="",s ;
  string ans="";
  //char ans[1000];
  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      s=detectors[idet]->getNetworkParameter(p);
      if(detectors[idet]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<idet));

      if (s0=="")
	s0=s+string("+");
      else
	s0+=s+string("+");

      if (s1=="")
	s1=s;
      else if (s1!=s)
	s1="bad";

    }
  }
  if (s1=="bad")
	  ans=s0;
   // strcpy(ans,s0.c_str());
  else
	  ans=s1;
   // strcpy(ans,s1.c_str());
  return ans;
}


/**
   sets the location of the calibration files
   \sa  sharedSlsDetector
*/
string multiSlsDetector::setNetworkParameter(networkParameter p, string s){

	if (s.find('+')==string::npos) {

		if(!threadpool){
			cout << "Error in creating threadpool. Exiting" << endl;
			return getNetworkParameter(p);
		}else{
			string* sret[thisMultiDetector->numberOfDetectors];
			for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
				if(detectors[idet]){
					if (p == RECEIVER_STREAMING_PORT || p == CLIENT_STREAMING_PORT)
						s.append("multi\0");
					sret[idet]=new string("error");
					Task* task = new Task(new func2_t<string,networkParameter,string>(&slsDetector::setNetworkParameter,
							detectors[idet],p,s,sret[idet]));
					threadpool->add_task(task);
				}
			}
			threadpool->startExecuting();
			threadpool->wait_for_tasks_to_complete();
			for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
				if(detectors[idet]){
					if(sret[idet]!= NULL)
						delete sret[idet];
					//doing nothing with the return values
					if(detectors[idet]->getErrorMask())
						setErrorMask(getErrorMask()|(1<<idet));
				}
			}
		}



	} else {
		size_t p1=0;
		size_t p2=s.find('+',p1);
		int id=0;
		while (p2!=string::npos) {

			if (detectors[id]) {
				detectors[id]->setNetworkParameter(p,s.substr(p1,p2-p1));
				if(detectors[id]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<id));
			}
			++id;
			s=s.substr(p2+1);
			p2=s.find('+');
			if (id>=thisMultiDetector->numberOfDetectors)
				break;
		}

	}

	return getNetworkParameter(p);

} 

int multiSlsDetector::setPort(portType t, int p) {

  int ret=-100, ret1;

  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      ret1=detectors[idet]->setPort(t,p);
      if(detectors[idet]->getErrorMask())
    	  setErrorMask(getErrorMask()|(1<<idet));
      if (ret==-100)
	ret=ret1;
      else if (ret!=ret1)
	ret=-1;
    }
  }
  return ret;

}

int multiSlsDetector::lockServer(int p) {

  int ret=-100, ret1;

  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      ret1=detectors[idet]->lockServer(p);
      if(detectors[idet]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<idet));
      if (ret==-100)
	ret=ret1;
      else if (ret!=ret1)
	ret=-1;
    }
  }

  return ret;

}

string multiSlsDetector::getLastClientIP() {
  string s0="", s1="",s ;
  
  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      s=detectors[idet]->getLastClientIP();
      if(detectors[idet]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<idet));

      if (s0=="")
	s0=s;
      else
	s0+=string("+")+s;
      if (s1=="")
	s1=s;
      else if (s1!=s)
	s1="bad";
    }
  }
  if (s1=="bad")
    return s0;
  else
    return s1;
}






int multiSlsDetector::setReadOutFlags(readOutFlags flag) {

  int ret=-100, ret1;

  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      ret1=detectors[idet]->setReadOutFlags(flag);
      if(detectors[idet]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<idet));
      if (ret==-100)
	ret=ret1;
      else if (ret!=ret1)
	ret=-1;
    }
  }

  return ret;


}


slsDetectorDefs::externalCommunicationMode multiSlsDetector::setExternalCommunicationMode(externalCommunicationMode pol) {

  externalCommunicationMode  ret, ret1;

  if (detectors[0])
    ret=detectors[0]->setExternalCommunicationMode(pol);
    if(detectors[0]->getErrorMask())
      setErrorMask(getErrorMask()|(1<<0));

  
  for (int idet=1; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      ret1=detectors[idet]->setExternalCommunicationMode(pol);
      if(detectors[idet]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<idet));
      if (ret!=ret1)
	ret=GET_EXTERNAL_COMMUNICATION_MODE;
    }
  }

  setMaster();
  setSynchronization();
  return ret;

}



slsDetectorDefs::externalSignalFlag multiSlsDetector::setExternalSignalFlags(externalSignalFlag pol, int signalindex) {

  externalSignalFlag  ret, ret1;

  if (detectors[0])
    ret=detectors[0]->setExternalSignalFlags(pol,signalindex);
    if(detectors[0]->getErrorMask())
      setErrorMask(getErrorMask()|(1<<0));

  for (int idet=1; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      ret1=detectors[idet]->setExternalSignalFlags(pol,signalindex);
      if(detectors[idet]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<idet));
      if (ret!=ret1)
	ret=GET_EXTERNAL_SIGNAL_FLAG;
    }
  }

  setMaster();
  setSynchronization();
  return ret;


}












string multiSlsDetector::getSettingsFile() {

  string s0="", s1="",s ;
  
  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      s=detectors[idet]->getSettingsFile();
      if(detectors[idet]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<idet));

      if (s0=="")
	s0=s;
      else
	s0+=string("+")+s;
      if (s1=="")
	s1=s;
      else if (s1!=s)
	s1="bad";
    }
  }
  if (s1=="bad")
    return s0;
  else
    return s1;

}


int multiSlsDetector::configureMAC() {

  int ret=-100, ret1;

  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      ret1=detectors[idet]->configureMAC();
	  if(detectors[idet]->getErrorMask())
		setErrorMask(getErrorMask()|(1<<idet));
      if (ret==-100)
	ret=ret1;
      else if (ret!=ret1)
	ret=-1;
    }
  }

  return ret;

}


int multiSlsDetector::loadImageToDetector(imageType index,string const fname){

  int ret=-100, ret1;
  short int imageVals[thisMultiDetector->numberOfChannels];

  ifstream infile;
  infile.open(fname.c_str(), ios_base::in);
  if (infile.is_open()) {
#ifdef VERBOSE
    std::cout<< std::endl<< "Loading ";
    if(!index)
      std::cout<<"Dark";
    else
      std::cout<<"Gain";
    std::cout<<" image from file " << fname << std::endl;
#endif
    for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
      if (detectors[idet]) {
  	if(detectors[idet]->readDataFile(infile,imageVals)>=0){
	  ret1=detectors[idet]->sendImageToDetector(index,imageVals);
	  if (ret==-100)
	    ret=ret1;
	  else if (ret!=ret1)
	    ret=-1;
	}
      if(detectors[idet]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<idet));
      }
    }
    infile.close();
  } else {
    std::cout<< "Could not open file "<< fname << std::endl;
    return -1;
  }
  return ret;
}

int multiSlsDetector::writeCounterBlockFile(string const fname,int startACQ){

  int ret=OK, ret1=OK;
  short int arg[thisMultiDetector->numberOfChannels];
  ofstream outfile;
  outfile.open(fname.c_str(), ios_base::out);
  if (outfile.is_open()) {
#ifdef VERBOSE
    std::cout<< std::endl<< "Reading Counter to \""<<fname;
    if(startACQ==1)
      std::cout<<"\" and Restarting Acquisition";
    std::cout<<std::endl;
#endif
    for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
      if (detectors[idet]) {
	ret1=detectors[idet]->getCounterBlock(arg,startACQ);
	if(ret1!=OK)
	  ret=FAIL;
	else{
	  ret1=detectors[idet]->writeDataFile(outfile,arg);
	  if(ret1!=OK)
	    ret=FAIL;
	}
	if(detectors[idet]->getErrorMask())
	  setErrorMask(getErrorMask()|(1<<idet));
      }
    }
    outfile.close();
  } else {
    std::cout<< "Could not open file "<< fname << std::endl;
    return -1;
  }
  return ret;
}


int multiSlsDetector::resetCounterBlock(int startACQ){

  int ret=-100, ret1;
  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      ret1=detectors[idet]->resetCounterBlock(startACQ);
      if(detectors[idet]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<idet));
      if (ret==-100)
	ret=ret1;
      else if (ret!=ret1)
	ret=-1;
    }
  }
  return ret;
}



int multiSlsDetector::setCounterBit(int i){
	int ret=-100,ret1;
	for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet)
		if (detectors[idet]){
			ret1=detectors[idet]->setCounterBit(i);
		    if(detectors[idet]->getErrorMask())
			  setErrorMask(getErrorMask()|(1<<idet));
			if(ret==-100)
				ret=ret1;
			else if (ret!=ret1)
				ret=-1;
		}
	return ret;
}




int multiSlsDetector::setDynamicRange(int p) {

    int ret=-100;
    thisMultiDetector->dataBytes=0;
    thisMultiDetector->numberOfChannels=0;

    if(!threadpool){
        cout << "Error in creating threadpool. Exiting" << endl;
        return -1;
    }else{
        //return storage values
        int* iret[thisMultiDetector->numberOfDetectors];
        for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
            if(detectors[idet]){
                iret[idet]= new int(-1);
                Task* task = new Task(new func1_t<int,int>(&slsDetector::setDynamicRange,
                        detectors[idet],p,iret[idet]));
                threadpool->add_task(task);
            }
        }
        threadpool->startExecuting();
        threadpool->wait_for_tasks_to_complete();
        for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
            if(detectors[idet]){
                if(iret[idet] != NULL){
                    thisMultiDetector->dataBytes+=detectors[idet]->getDataBytes();
                    thisMultiDetector->numberOfChannels+=detectors[idet]->getTotalNumberOfChannels();
                    if (ret==-100)
                        ret=*iret[idet];
                    else if (ret!=*iret[idet])
                        ret=-1;
                    delete iret[idet];
                }else ret=-1;
                if(detectors[idet]->getErrorMask())
                    setErrorMask(getErrorMask()|(1<<idet));
            }
        }
    }

    //for usability for the user
    if (getDetectorsType() == EIGER){
        if(p == 32){
            std::cout << "Setting Clock to Quarter Speed to cope with Dynamic Range of 32" << std::endl;
            setSpeed(CLOCK_DIVIDER,2);
        }
        else if(p == 16){
            std::cout << "Setting Clock to Half Speed for Dynamic Range of 16" << std::endl;
            setSpeed(CLOCK_DIVIDER,1);
        }
    }
    return ret;
}



int multiSlsDetector::getMaxMods() {


  int ret=0, ret1;
  
  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      ret1=detectors[idet]->getMaxMods();
#ifdef VERBOSE
      cout << "detector " << idet << " maxmods " << ret1 << endl;
#endif
      ret+=ret1;
    }
  }
#ifdef VERBOSE
  cout << "max mods is " << ret << endl;
#endif

  return ret;

}




  int multiSlsDetector::getTotalNumberOfChannels(){thisMultiDetector->numberOfChannels=0; for (int id=0; id< thisMultiDetector->numberOfDetectors; ++id) thisMultiDetector->numberOfChannels+=detectors[id]->getTotalNumberOfChannels(); return thisMultiDetector->numberOfChannels;};

  //int multiSlsDetector::getTotalNumberOfChannels(dimension d){thisMultiDetector->numberOfChannel[d]=0; for (int id=0; id< thisMultiDetector->numberOfDetectors; ++id) thisMultiDetector->numberOfChannel[d]+=detectors[id]->getTotalNumberOfChannels(d); return thisMultiDetector->numberOfChannel[d];};
  int multiSlsDetector::getTotalNumberOfChannels(dimension d){return thisMultiDetector->numberOfChannel[d];};

  int multiSlsDetector::getMaxNumberOfChannels(){thisMultiDetector->maxNumberOfChannels=0; for (int id=0; id< thisMultiDetector->numberOfDetectors; ++id) thisMultiDetector->maxNumberOfChannels+=detectors[id]->getMaxNumberOfChannels();return thisMultiDetector->maxNumberOfChannels;};

 // int multiSlsDetector::getMaxNumberOfChannels(dimension d){thisMultiDetector->maxNumberOfChannel[d]=0; for (int id=0; id< thisMultiDetector->numberOfDetectors; ++id) thisMultiDetector->maxNumberOfChannel[d]+=detectors[id]->getMaxNumberOfChannels(d);return thisMultiDetector->maxNumberOfChannel[d];};
  int multiSlsDetector::getMaxNumberOfChannels(dimension d){return thisMultiDetector->maxNumberOfChannel[d];};






int multiSlsDetector::getMaxMod(dimension d){

  int ret=0, ret1;

  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      ret1=detectors[idet]->getNMaxMod(d);
#ifdef VERBOSE
      cout << "detector " << idet << " maxmods " << ret1 << " in direction " << d << endl;
#endif
      ret+=ret1;
    }
  }
#ifdef VERBOSE
  cout << "max mods in direction "<< d << " is " << ret << endl;
#endif

  return ret;

}


int multiSlsDetector::getMaxNumberOfModules(dimension d) {

  int ret=0, ret1;
  
  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      ret1=detectors[idet]->getMaxNumberOfModules(d);
      ret+=ret1;
    }
  }
  return ret;

}


int multiSlsDetector::getFlippedData(dimension d){
	int ret=-100,ret1;
	for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet)
		if (detectors[idet]){
			ret1=detectors[idet]->getFlippedData(d);
			if(ret==-100)
				ret=ret1;
			else if (ret!=ret1)
				ret=-1;
		}
	return ret;
}

int multiSlsDetector::setNumberOfModules(int p, dimension d) {

  int ret=0;//, ret1;
  int nm, mm, nt=p;

  thisMultiDetector->dataBytes=0;
  thisMultiDetector->numberOfChannels=0;

  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {

    //   cout << "detector " << idet << endl;
    if (detectors[idet]) {
      if (p<0)
	nm=p;
      else {
	mm=detectors[idet]->getMaxNumberOfModules();
	//mm=detectors[idet]->getMaxMods();
	if (nt>mm) {
	  nm=mm;
	  nt-=nm;
	} else {
	  nm=nt;
	  nt-=nm;
	}
      }
      ret+=detectors[idet]->setNumberOfModules(nm);
      if(detectors[idet]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<idet));
      thisMultiDetector->dataBytes+=detectors[idet]->getDataBytes();
      thisMultiDetector->numberOfChannels+=detectors[idet]->getTotalNumberOfChannels();
    }
  }

  if(p != -1)
    updateOffsets();
  return ret;

}



int multiSlsDetector::setFlippedData(dimension d, int value){
	int ret=-100,ret1;
	for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet)
		if (detectors[idet]){
			ret1=detectors[idet]->setFlippedData(d,value);
			if(ret==-100)
				ret=ret1;
			else if (ret!=ret1)
				ret=-1;
		}
	return ret;
}




int multiSlsDetector::decodeNMod(int i, int &id, int &im) {
#ifdef VERBOSE
  cout << " Module " << i << " belongs to detector " << id << endl;;
  cout << getMaxMods();
#endif

  if (i<0 || i>=getMaxMods()) {
    id=-1;
    im=-1;
#ifdef VERBOSE
    cout  << " A---------" << id << " position " << im << endl;
#endif

    return -1;
  }
  int nm;
  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      nm=detectors[idet]->getNMods();
      if (nm>i) {
	id=idet;
	im=i;
#ifdef VERBOSE
	cout  << " B---------" <<id << " position " << im << endl;
#endif
	return im;
      } else {
	i-=nm;
      }
    }
  }
  id=-1;
  im=-1;
#ifdef VERBOSE
  cout  <<" C---------" << id << " position " << im << endl;
#endif
  return -1;
  

}


 
int64_t multiSlsDetector::getId(idMode mode, int imod) {

  int id, im;
  int64_t ret;

  if (decodeNMod(imod, id, im)>=0) {
    if (detectors[id]) {
      ret = detectors[id]->getId(mode, im);
      if(detectors[id]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<id));
      return ret;
    }
  }

  ret = -100; int64_t ret1;
    for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet)
        if (detectors[idet]){
            ret1=detectors[idet]->getId(mode, imod);
            if(ret==-100)
                ret=ret1;
            else if (ret!=ret1)
                ret=-1;
        }
    return ret;

  return -1;

}

int multiSlsDetector::digitalTest(digitalTestMode mode, int imod) {

  int id, im, ret;

  if (decodeNMod(imod, id, im)>=0) {
    if (detectors[id]) {
      ret = detectors[id]->digitalTest(mode, im);
      if(detectors[id]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<id));
      return ret;
    }
  }

  return -1;

}





int multiSlsDetector::executeTrimming(trimMode mode, int par1, int par2, int imod) {
	int ret=100;

	// single
	{
		int id=-1, im=-1;
		if (decodeNMod(imod, id, im)>=0) {
			if (detectors[id]) {
				ret = detectors[id]->executeTrimming(mode, par1, par2, im);
				if(detectors[id]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<id));
				return ret;
			}
			return -1;
		}
	}


	// multi
	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return -1;
	}

	int* iret[thisMultiDetector->numberOfDetectors];
	for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
		if(detectors[idet]){
			iret[idet]= new int(-1);
			Task* task = new Task(new func4_t<int,trimMode,int,int,int>(&slsDetector::executeTrimming,
					detectors[idet],mode,par1,par2,imod,iret[idet]));
			threadpool->add_task(task);
		}
	}
	threadpool->startExecuting();
	threadpool->wait_for_tasks_to_complete();
	for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
		if(detectors[idet]){
			if(iret[idet] != NULL){
				if (ret==-100)
					ret=*iret[idet];
				else if (ret!=*iret[idet])
					ret=-1;
				delete iret[idet];
			}else ret=-1;
			if(detectors[idet]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<idet));
		}
	}

	return ret;
}



int multiSlsDetector::programFPGA(string fname){
	int ret=OK, ret1=OK;

	for (int i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
		if (detectors[i]) {
			ret=detectors[i]->programFPGA(fname);
			if(detectors[i]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<i));
			if (ret==FAIL)
				ret1=FAIL;
		}
	}
	return ret1;
}



int multiSlsDetector::resetFPGA(){
	int ret=OK, ret1=OK;

	for (int i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
		if (detectors[i]) {
			ret=detectors[i]->resetFPGA();
			if(detectors[i]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<i));
			if (ret==FAIL)
				ret1=FAIL;
		}
	}
	return ret1;
}



int multiSlsDetector::powerChip(int ival){
	int ret=OK, ret1=OK;

	for (int i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
		if (detectors[i]) {
			ret=detectors[i]->powerChip(ival);
			if(detectors[i]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<i));
			if (ret==FAIL)
				ret1=FAIL;
		}
	}
	return ret1;
}


int multiSlsDetector::setAutoComparatorDisableMode(int ival) {
    int ret=OK, ret1=OK;

    for (int i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
        if (detectors[i]) {
            ret=detectors[i]->setAutoComparatorDisableMode(ival);
            if(detectors[i]->getErrorMask())
                setErrorMask(getErrorMask()|(1<<i));
            if (ret==FAIL)
                ret1=FAIL;
        }
    }
    return ret1;
}


int multiSlsDetector::loadSettingsFile(string fname, int imod) {
	int ret=OK;

	// single
	{
		int id=-1, im=-1;
		if (decodeNMod(imod, id, im)>=0) {
			if(detectors[id]){
				ret = detectors[id]->loadSettingsFile(fname, im);
				if(detectors[id]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<id));
				return ret;
			}
			return -1;
		}
	}

	// multi
	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return -1;
	}

	int* iret[thisMultiDetector->numberOfDetectors];
	for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
		if(detectors[idet]){
			iret[idet]= new int(OK);
			Task* task = new Task(new func2_t<int,string,int>(&slsDetector::loadSettingsFile,
					detectors[idet],fname,imod,iret[idet]));
			threadpool->add_task(task);
		}
	}
	threadpool->startExecuting();
	threadpool->wait_for_tasks_to_complete();
	for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
		if(detectors[idet]){
			if(iret[idet] != NULL){
				if(*iret[idet] != OK)
					ret = FAIL;
				delete iret[idet];
			}else  ret = FAIL;
			if(detectors[idet]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<idet));
		}
	}

	return ret;
}


int multiSlsDetector::saveSettingsFile(string fname, int imod) {
	int id=-1, im=-1, ret;

	if (decodeNMod(imod, id, im)>=0) {
		if (detectors[id]) {
			ret = detectors[id]->saveSettingsFile(fname, im);
			if(detectors[id]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<id));
			return ret;
		}
		return -1;
	}
	for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
		if (detectors[idet]) {
			ret=detectors[idet]->saveSettingsFile(fname, imod);
			if(detectors[idet]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<idet));
		}
	}
	return ret;
}



int multiSlsDetector::setAllTrimbits(int val, int imod){

	int ret=-100;

	// single
	{
		int id=-1, im=-1;
		if (decodeNMod(imod, id, im)>=0) {
			if(detectors[id]){
				ret = detectors[id]->setAllTrimbits(val,im);
				if(detectors[id]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<id));
				return ret;
			}
			return -1;
		}
	}

	// multi
	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return -1;
	}
	int* iret[thisMultiDetector->numberOfDetectors];
	for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
		if(detectors[idet]){
			iret[idet]= new int(-1);
			Task* task = new Task(new func2_t<int,int,int>(&slsDetector::setAllTrimbits,
					detectors[idet],val,imod,iret[idet]));
			threadpool->add_task(task);
		}
	}
	threadpool->startExecuting();
	threadpool->wait_for_tasks_to_complete();
	for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
		if(detectors[idet]){
			if(iret[idet] != NULL){
				if (ret==-100)
					ret=*iret[idet];
				else if (ret!=*iret[idet])
					ret=-1;
				delete iret[idet];
			}else ret=-1;
			if(detectors[idet]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<idet));
		}
	}
	return ret;
}





int multiSlsDetector::loadCalibrationFile(string fname, int imod) {
	int ret = OK;

	// single
	{
		int id=-1, im=-1;
		if (decodeNMod(imod, id, im)>=0) {
			if(detectors[id]){
				ret = detectors[id]->loadCalibrationFile(fname, im);
				if(detectors[id]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<id));
				return ret;
			}
			return -1;
		}
	}

	// multi
	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return -1;
	}

	int* iret[thisMultiDetector->numberOfDetectors];
	for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
		if(detectors[idet]){
			iret[idet]= new int(OK);
			Task* task = new Task(new func2_t<int,string,int>(&slsDetector::loadCalibrationFile,
					detectors[idet],fname,imod,iret[idet]));
			threadpool->add_task(task);
		}
	}
	threadpool->startExecuting();
	threadpool->wait_for_tasks_to_complete();
	for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
		if(detectors[idet]){
			if(iret[idet] != NULL){
				if(*iret[idet] != OK)
					ret = FAIL;
				delete iret[idet];
			}else  ret = FAIL;
			if(detectors[idet]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<idet));
		}
	}

	return ret;
}


int multiSlsDetector::saveCalibrationFile(string fname, int imod) {
	int id=-1, im=-1, ret;

	if (decodeNMod(imod, id, im)>=0) {
		if (detectors[id]) {
			ret = detectors[id]->saveCalibrationFile(fname, im);
			if(detectors[id]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<id));
			return ret;
		}
		return -1;
	}
	for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
		if (detectors[idet]) {
			ret=detectors[idet]->saveCalibrationFile(fname, imod);
			if(detectors[idet]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<idet));
		}
	}
	return ret;
}





uint32_t multiSlsDetector::writeRegister(uint32_t addr, uint32_t val){

	uint32_t ret, ret1;

	for (int i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
		if (detectors[i]) {
			ret=detectors[i]->writeRegister(addr,val);
			if(detectors[i]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<i));
			if (i==0)
				ret1=ret;
			else if (ret!=ret1) {
				// not setting it to -1 as it is a possible value
				std::cout << "Error: Different Values for function writeRegister [" << ret << "," << ret1 << "]" << endl;
				setErrorMask(getErrorMask()|MULTI_HAVE_DIFFERENT_VALUES);
			}
		}
	}

	return ret1;
}



int multiSlsDetector::writeAdcRegister(int addr, int val){

	int ret, ret1=-100;

	for (int i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
		if (detectors[i]) {
			ret=detectors[i]->writeAdcRegister(addr,val);
			if(detectors[i]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<i));
			if (ret1==-100)
				ret1=ret;
			else if (ret!=ret1) {
				// not setting it to -1 as it is a possible value
				std::cout << "Error: Different Values for function writeAdcRegister [" << ret << "," << ret1 << "]" << endl;
				setErrorMask(getErrorMask()|MULTI_HAVE_DIFFERENT_VALUES);
			}
		}
	}

	return ret1;
}


uint32_t multiSlsDetector::readRegister(uint32_t addr){

	uint32_t ret, ret1;

	for (int i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
		if (detectors[i]) {
			ret=detectors[i]->readRegister(addr);
			if(detectors[i]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<i));
			if (i==0)
				ret1=ret;
			else if (ret!=ret1) {
				// not setting it to -1 as it is a possible value
				std::cout << "Error: Different Values for function readRegister [" << ret << "," << ret1 << "]" << endl;
				setErrorMask(getErrorMask()|MULTI_HAVE_DIFFERENT_VALUES);
			}
		}
	}

	return ret1;
}


uint32_t multiSlsDetector::setBit(uint32_t addr, int n) {
	uint32_t ret, ret1;

	for (int i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
		if (detectors[i]) {
			ret1=detectors[i]->setBit(addr,n);
			if(detectors[i]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<i));
			if (i==0)
				ret=ret1;
			else if (ret!=ret1) {
				// not setting it to -1 as it is a possible value
				std::cout << "Error: Different Values for function setBit [" << ret << "," << ret1 << "]" << endl;
				setErrorMask(getErrorMask()|MULTI_HAVE_DIFFERENT_VALUES);
			}
		}
	}

	return ret;
}


uint32_t multiSlsDetector::clearBit(uint32_t addr, int n) {
	uint32_t ret, ret1;

	for (int i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
		if (detectors[i]) {
			ret1=detectors[i]->clearBit(addr,n);
			if(detectors[i]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<i));
			if (i==0)
				ret=ret1;
			else if (ret!=ret1) {
				// not setting it to -1 as it is a possible value
				std::cout << "Error: Different Values for function clearBit [" << ret << "," << ret1 << "]" << endl;
				setErrorMask(getErrorMask()|MULTI_HAVE_DIFFERENT_VALUES);
			}
		}
	}

	return ret;
}




int multiSlsDetector::printReceiverConfiguration(){
	int  i;
	int ret, ret1=-100;

	std::cout << "Printing Receiver configurations for all detectors..." << std::endl;

	for (i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
		if (detectors[i]) {
			std::cout << std::endl << "#Detector " << i << ":" << std::endl;

			ret=detectors[i]->printReceiverConfiguration();
			if(detectors[i]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<i));
			if (ret1==-100)
				ret1=ret;
			else if (ret!=ret1)
				ret1=-1;
		}
	}

	return ret1;
}


int multiSlsDetector::readConfigurationFile(string const fname){



  int nd=thisMultiDetector->numberOfDetectors;

  for (int i=0; i<nd; ++i) {
    if (detectors[i]) {
      detectors[i]->freeSharedMemory();
    }
  }
  thisMultiDetector->numberOfDetectors=0;

  multiSlsDetectorClient *cmd;


  setAcquiringFlag(false);
  clearAllErrorMask();

  string ans;
  string str;
  ifstream infile;
  int iargval;
  int interrupt=0;
  char *args[1000];

  char myargs[1000][1000];


  string sargname, sargval;
  int iline=0;
  std::cout<< "config file name "<< fname << std::endl;
  infile.open(fname.c_str(), ios_base::in);
  if (infile.is_open()) {


    while (infile.good() and interrupt==0) {
      sargname="none";
      sargval="0";
      getline(infile,str);
      ++iline;
#ifdef VERBOSE
      std::cout<<  str << std::endl;
#endif
      if (str.find('#')!=string::npos) {
#ifdef VERBOSE
	std::cout<< "Line is a comment " << std::endl;
	std::cout<< str << std::endl;
#endif
	continue;
      } else if (str.length()<2) {
#ifdef VERBOSE
	std::cout<< "Empty line " << std::endl;
#endif
	continue;
      } else {
	istringstream ssstr(str);
	iargval=0;
	while (ssstr.good()) {
	  ssstr >> sargname;
	  //if (ssstr.good()) {
#ifdef VERBOSE 
	  std::cout<< iargval << " " << sargname  << std::endl;
#endif

	  strcpy(myargs[iargval], sargname.c_str());
	  args[iargval]=myargs[iargval];

#ifdef VERBOSE 
	  std::cout<< "--" << iargval << " " << args[iargval]  << std::endl;
#endif

	  ++iargval;
	  //}
	}

#ifdef VERBOSE
	cout << endl;
	for (int ia=0; ia<iargval; ia++) cout << args[ia] << " ??????? ";
	cout << endl;
#endif
	cmd=new multiSlsDetectorClient(iargval, args, PUT_ACTION, this);
	delete cmd;
      }
      ++iline;
    }




    infile.close();

	//if(getDetectorsType() != MYTHEN)
	//	printReceiverConfiguration();

  } else {
    std::cout<< "Error opening configuration file " << fname << " for reading" << std::endl;
    return FAIL;
  }
#ifdef VERBOSE
  std::cout<< "Read configuration file of " << iline << " lines" << std::endl;
#endif

  setNumberOfModules(-1);
  getMaxNumberOfModules();

  if (getErrorMask()){
	  int c;
	  cprintf(RED,"\n----------------\n Error Messages\n----------------\n%s\n",
			  getErrorMessage(c).c_str());
	  return FAIL;
  }

  return OK;


};  





int multiSlsDetector::writeConfigurationFile(string const fname){

	string names[]={				\
			"detsizechan",			\
			"hostname",				\
			"master",				\
			"sync",					\
			"outdir",				\
			"ffdir",				\
			"headerbefore",			\
			"headerafter",			\
			"headerbeforepar",		\
			"headerafterpar",		\
			"badchannels",			\
			"angconv",				\
			"globaloff",			\
			"binsize",				\
			"threaded"				};

	int nvar=15;
	char *args[100];
	for (int ia=0; ia<100; ++ia) {
		args[ia]=new char[1000];
	}
	int ret=OK,ret1=OK;



	ofstream outfile;
	int iline = 0;

	outfile.open(fname.c_str(),ios_base::out);
	if (outfile.is_open()) {



		slsDetectorCommand *cmd=new slsDetectorCommand(this);

		// complete size of detector
		cout << iline << " " << names[iline] << endl;
		strcpy(args[0],names[iline].c_str());
		outfile << names[iline] << " " << cmd->executeLine(1,args,GET_ACTION) << std::endl;
		++iline;

		// hostname of the detectors
		cout << iline << " " << names[iline] << endl;
		strcpy(args[0],names[iline].c_str());
		outfile << names[iline] << " " << cmd->executeLine(1,args,GET_ACTION) << std::endl;
		++iline;

		// single detector configuration
		for (int i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
			//    sprintf(ext,".det%d",i);
			if (detectors[i]) {
				outfile << endl;
				ret1 = detectors[i]->writeConfigurationFile(outfile,i);
				if(detectors[i]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<i));
				if (ret1 == FAIL)
					ret = FAIL;
			}
		}

		outfile << endl;
		//other configurations
		while (iline < nvar) {
			cout << iline << " " << names[iline] << endl;
			strcpy(args[0],names[iline].c_str());
			outfile << names[iline] << " " << cmd->executeLine(1,args,GET_ACTION) << std::endl;
			++iline;
		}



		delete cmd;
		outfile.close();
#ifdef VERBOSE
		std::cout<< "wrote " <<iline << " lines to configuration file " << std::endl;
#endif
	}  else {
		std::cout<< "Error opening configuration file " << fname << " for writing" << std::endl;
		ret = FAIL;
	}

	for (int ia=0; ia<100; ++ia) {
		delete [] args[ia];
	}

	return ret;
}














int multiSlsDetector::writeDataFile(string fname, double *data, double *err, double *ang, char dataformat, int nch) {

#ifdef VERBOSE
  cout << "using overloaded multiSlsDetector function to write formatted data file " << getTotalNumberOfChannels()<< endl;
#endif


  ofstream outfile;
  int choff=0, off=0; //idata, 
  double *pe=err, *pa=ang;
  int nch_left=nch, n;//, nd;

  if (nch_left<=0)
    nch_left=getTotalNumberOfChannels();


  if (data==NULL)
    return FAIL;

  outfile.open (fname.c_str(),ios_base::out);
  if (outfile.is_open())
    {
    
      for (int i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
     
	if (detectors[i]) {
	  n=detectors[i]->getTotalNumberOfChannels();
	  if (nch_left<n)
	    n=nch_left;

#ifdef VERBOSE
	  cout << " write " << i << " position " << off << " offset " << choff << endl;
#endif
	  //detectors[i]->writeDataFile(outfile,n, data+off, pe, pa, dataformat, choff);
	  fileIOStatic::writeDataFile(outfile,n, data+off, pe, pa, dataformat, choff);
	  if(detectors[i]->getErrorMask())
	    setErrorMask(getErrorMask()|(1<<i));

	  nch_left-=n;

	  choff+=detectors[i]->getMaxNumberOfChannels();
       
	  off+=n;
    
	  if (pe)
	    pe=err+off;
       
	  if (pa)
	    pa=ang+off;
     
	}
     
      }
      outfile.close();
      return OK;
    } else {
    std::cout<< "Could not open file " << fname << "for writing"<< std::endl;
    return FAIL;
  }
}


int multiSlsDetector::writeDataFile(string fname, int *data) {
  ofstream outfile;
  int choff=0, off=0;

#ifdef VERBOSE
  cout << "using overloaded multiSlsDetector function to write raw data file " << endl;
#endif

  if (data==NULL)
    return FAIL;

  outfile.open (fname.c_str(),ios_base::out);
  if (outfile.is_open())
    {
      for (int i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
	if (detectors[i]) {
#ifdef VERBOSE
	  cout << " write " << i << " position " << off << " offset " << choff << endl;
#endif
	  detectors[i]->writeDataFile(outfile, detectors[i]->getTotalNumberOfChannels(), data+off, choff);
	  if(detectors[i]->getErrorMask())
	    setErrorMask(getErrorMask()|(1<<i));
	  choff+=detectors[i]->getMaxNumberOfChannels();
	  off+=detectors[i]->getTotalNumberOfChannels();
	}
      }


      outfile.close();
      return OK;
    } else {
    std::cout<< "Could not open file " << fname << "for writing"<< std::endl;
    return FAIL;
  }
}


int multiSlsDetector::readDataFile(string fname, double *data, double *err, double *ang, char dataformat){

#ifdef VERBOSE
  cout << "using overloaded multiSlsDetector function to read formatted data file " << endl;
#endif

  ifstream infile;
  int iline=0;//ichan, 
  //int interrupt=0;
  string str;
  int choff=0, off=0;
  double *pe=err, *pa=ang;

#ifdef VERBOSE
  std::cout<< "Opening file "<< fname << std::endl;
#endif
  infile.open(fname.c_str(), ios_base::in);
  if (infile.is_open()) {

    for (int i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
      if (detectors[i]) {
	iline+=detectors[i]->readDataFile(detectors[i]->getTotalNumberOfChannels(), infile, data+off, pe, pa, dataformat, choff);
	if(detectors[i]->getErrorMask())
	  setErrorMask(getErrorMask()|(1<<i));
	choff+=detectors[i]->getMaxNumberOfChannels();
	off+=detectors[i]->getTotalNumberOfChannels();
	if (pe)
	  pe=pe+off;
	if (pa)
	  pa=pa+off;
      }
    }


    infile.close();
  } else {
    std::cout<< "Could not read file " << fname << std::endl;
    return -1;
  }
  return iline;

}

int multiSlsDetector::readDataFile(string fname, int *data) {

#ifdef VERBOSE
  cout << "using overloaded multiSlsDetector function to read raw data file " << endl;
#endif

  ifstream infile;
  int  iline=0;//ichan,
  //int interrupt=0;
  string str;
  int choff=0, off=0;

#ifdef VERBOSE
  std::cout<< "Opening file "<< fname << std::endl;
#endif
  infile.open(fname.c_str(), ios_base::in);
  if (infile.is_open()) {

    for (int i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
      if (detectors[i]) {
	iline+=detectors[i]->readDataFile(infile, data+off,detectors[i]->getTotalNumberOfChannels(), choff);
	if(detectors[i]->getErrorMask())
	  setErrorMask(getErrorMask()|(1<<i));
	choff+=detectors[i]->getMaxNumberOfChannels();
	off+=detectors[i]->getTotalNumberOfChannels();
      }
    }
    infile.close();
  } else {
    std::cout<< "Could not read file " << fname << std::endl;
    return -1;
  }
  return iline;
}





//receiver


int multiSlsDetector::setReceiverOnline(int off) {

	if (off != GET_ONLINE_FLAG) {
	    int ret=-100;
		if(!threadpool){
			cout << "Error in creating threadpool. Exiting" << endl;
			return -1;
		}else{
			//return storage values
			int* iret[thisMultiDetector->numberOfDetectors];
			for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
				if(detectors[idet]){
					iret[idet]= new int(-1);
					Task* task = new Task(new func1_t<int,int>(&slsDetector::setReceiverOnline,
							detectors[idet],off,iret[idet]));
					threadpool->add_task(task);
				}
			}
			threadpool->startExecuting();
			threadpool->wait_for_tasks_to_complete();
			for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
				if(detectors[idet]){
					if(iret[idet] != NULL){
						if (ret==-100)
							ret=*iret[idet];
						else if (ret!=*iret[idet])
							ret=-1;
						delete iret[idet];
					}else ret=-1;
					if(detectors[idet]->getErrorMask())
						setErrorMask(getErrorMask()|(1<<idet));
				}
			}
		}
		thisMultiDetector->receiverOnlineFlag=ret;
	}
	return thisMultiDetector->receiverOnlineFlag;
}



string multiSlsDetector::checkReceiverOnline() {
  string retval1 = "",retval;
  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      retval=detectors[idet]->checkReceiverOnline();
      if(!retval.empty()){
	retval1.append(retval);
	retval1.append("+");
      }
    }
  }
  return retval1;
}




string multiSlsDetector::setFilePath(string s) {

	string ret="errorerror", ret1;
	//if the sls file paths are different, it should be realized by always using setfilepath even if string empty
	//if(!s.empty()){

		for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
			if (detectors[idet]) {
				ret1=detectors[idet]->setFilePath(s);
			      if(detectors[idet]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<idet));
				if (ret=="errorerror")
					ret=ret1;
				else if (ret!=ret1)
					ret="";
			}
		}
		fileIO::setFilePath(ret);
	//}

	return fileIO::getFilePath();
}



string multiSlsDetector::setFileName(string s) {

	string ret = "error";
	int posmax = thisMultiDetector->numberOfDetectors;

	if(!s.empty()){
		fileIO::setFileName(s);
		if (thisMultiDetector->receiverOnlineFlag == ONLINE_FLAG)
			s=createReceiverFilePrefix();
	}

	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return string("");
	} else {
		string* sret[thisMultiDetector->numberOfDetectors];
		for(int idet=0; idet<posmax; ++idet){
			if(detectors[idet]){
				sret[idet]=new string("error");
				Task* task = new Task(new func1_t<string,string>(&slsDetector::setFileName,
						detectors[idet],s,sret[idet]));
				threadpool->add_task(task);
			}
		}
		threadpool->startExecuting();
		threadpool->wait_for_tasks_to_complete();
		for(int idet=0; idet<posmax; ++idet){
			if(detectors[idet]){
				if(sret[idet]!= NULL) {
					if(ret == "error")
						ret = *sret[idet];
					else if (ret != *sret[idet])
						ret="";
					delete sret[idet];
				}else ret = "";
				//doing nothing with the return values
				if(detectors[idet]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<idet));
			}
		}
	}

	if ((thisMultiDetector->receiverOnlineFlag == ONLINE_FLAG) && ((ret != "error") || (ret != ""))) {
#ifdef VERBOSE
			std::cout << "Complete file prefix from receiver: " << ret << std::endl;
#endif
		fileIO::setFileName(getNameFromReceiverFilePrefix(ret));
	}

	return ret;
}



slsReceiverDefs::fileFormat multiSlsDetector::setFileFormat(fileFormat f) {
	int ret=-100, ret1;

	for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
		if (detectors[idet]) {
			ret1=(int)detectors[idet]->setFileFormat(f);
			if(detectors[idet]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<idet));
			if (ret==-100)
				ret=ret1;
			else if (ret!=ret1)
				ret=-1;
		}
	}
	return (fileFormat)ret;
}



int multiSlsDetector::setFileIndex(int i) {

	int ret=-100;

	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return -1;
	}else{
		//return storage values
		int* iret[thisMultiDetector->numberOfDetectors];
		for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
			if(detectors[idet]){
				iret[idet]= new int(-1);
				Task* task = new Task(new func1_t<int,int>(&slsDetector::setFileIndex,
						detectors[idet],i,iret[idet]));
				threadpool->add_task(task);
			}
		}
		threadpool->startExecuting();
		threadpool->wait_for_tasks_to_complete();
		for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
			if(detectors[idet]){
				if(iret[idet] != NULL){
					if (ret==-100)
						ret=*iret[idet];
					else if (ret!=*iret[idet])
						ret=-1;
					delete iret[idet];
				}else ret=-1;
				if(detectors[idet]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<idet));
			}
		}
	}

	return ret;
}



int multiSlsDetector::startReceiver(){
	int i=0;
	int ret=OK;
	int posmin=0, posmax=thisMultiDetector->numberOfDetectors;

	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return FAIL;
	}else{
		int* iret[posmax-posmin];
		for(int idet=posmin; idet<posmax; ++idet){
			if((idet!=thisMultiDetector->masterPosition) && (detectors[idet])){
				iret[idet]= new int(OK);
				Task* task = new Task(new func0_t<int>(&slsDetector::startReceiver,
						detectors[idet],iret[idet]));
				threadpool->add_task(task);
			}
		}
		threadpool->startExecuting();
		threadpool->wait_for_tasks_to_complete();
		for(int idet=posmin; idet<posmax; ++idet){
			if((idet!=thisMultiDetector->masterPosition) && (detectors[idet])){
				if(iret[idet] != NULL){
					if(*iret[idet] != OK)
						ret = FAIL;
					delete iret[idet];
				}else  ret = FAIL;
				if(detectors[idet]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<idet));
			}
		}
	}

	//master
	int ret1=OK;
	i=thisMultiDetector->masterPosition;
	if (thisMultiDetector->masterPosition>=0) {
		if (detectors[i]) {
			ret1=detectors[i]->startReceiver();
			if(detectors[i]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<i));
			if (ret1!=OK)
				ret=FAIL;
		}
	}
	return ret;
}




int multiSlsDetector::stopReceiver(){
	int i=0;
	int ret=OK,ret1=OK;
	int posmin=0, posmax=thisMultiDetector->numberOfDetectors;

	i=thisMultiDetector->masterPosition;
	if (thisMultiDetector->masterPosition>=0) {
		if (detectors[i]) {
			ret1=detectors[i]->stopReceiver();
			if(detectors[i]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<i));
			if (ret1!=OK)
				ret=FAIL;
		}
	}

	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return FAIL;
	}else{
		int* iret[posmax-posmin];
		for(int idet=posmin; idet<posmax; ++idet){
			if((idet!=thisMultiDetector->masterPosition) && (detectors[idet])){
				iret[idet]= new int(OK);
				Task* task = new Task(new func0_t<int>(&slsDetector::stopReceiver,
						detectors[idet],iret[idet]));
				threadpool->add_task(task);
			}
		}
		threadpool->startExecuting();
		threadpool->wait_for_tasks_to_complete();
		for(int idet=posmin; idet<posmax; ++idet){
			if((idet!=thisMultiDetector->masterPosition) && (detectors[idet])){
				if(iret[idet] != NULL){
					if(*iret[idet] != OK)
						ret = FAIL;
					delete iret[idet];
				}else  ret = FAIL;
				if(detectors[idet]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<idet));
			}
		}
	}

	return ret;
}


slsDetectorDefs::runStatus multiSlsDetector::startReceiverReadout(){
	int i=0;
	runStatus s = IDLE,s1 = IDLE;
	i=thisMultiDetector->masterPosition;
	if (thisMultiDetector->masterPosition>=0) {
		if (detectors[i]) {
			s1=detectors[i]->startReceiverReadout();
			if(detectors[i]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<i));

		}
	}
	for (i=0; i<thisMultiDetector->numberOfDetectors; ++i) {
		if (detectors[i]) {
			s=detectors[i]->startReceiverReadout();
			if(detectors[i]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<i));
			if(s == ERROR)
				s1 = ERROR;
			/*if(s1 != s)
				s1 = ERROR;*/
			if(s != IDLE)
				s1 = s;
		}
	}

	/**stoppedFlag=1;*/
	return s1;
}

slsDetectorDefs::runStatus multiSlsDetector::getReceiverStatus(){
	int i=0;
	runStatus ret=IDLE;
	int posmin=0, posmax=thisMultiDetector->numberOfDetectors;

	i=thisMultiDetector->masterPosition;
	if (thisMultiDetector->masterPosition>=0) {
		if (detectors[i]) {
			ret=detectors[i]->getReceiverStatus();
			if(detectors[i]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<i));
			return ret;
		}
	}

	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return ERROR;
	}else{
		runStatus* iret[posmax-posmin];
		for(int idet=posmin; idet<posmax; ++idet){
			if((idet!=thisMultiDetector->masterPosition) && (detectors[idet])){
				iret[idet]= new runStatus(ERROR);
				Task* task = new Task(new func0_t<runStatus>(&slsDetector::getReceiverStatus,
						detectors[idet],iret[idet]));
				threadpool->add_task(task);
			}
		}
		threadpool->startExecuting();
		threadpool->wait_for_tasks_to_complete();
		for(int idet=posmin; idet<posmax; ++idet){
			if((idet!=thisMultiDetector->masterPosition) && (detectors[idet])){
				if(iret[idet] != NULL){
					if(*iret[idet] == (int)ERROR)
						ret = ERROR;
					if(*iret[idet] != IDLE)
						ret = *iret[idet];
					delete iret[idet];
				}else  ret = ERROR;
				if(detectors[idet]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<idet));
			}
		}
	}

	return ret;
}


int multiSlsDetector::getFramesCaughtByAnyReceiver() {

	int ret = 0;
	int i = thisMultiDetector->masterPosition;

	if (i >=0 ) {
		if (detectors[i]){
			ret=detectors[i]->getFramesCaughtByReceiver();
			if(detectors[i]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<i));
			// return master receivers frames caught
			return ret;
		}
	}

	for (int i=0; i<thisMultiDetector->numberOfDetectors; ++i)
		if (detectors[i]){
			ret=detectors[i]->getFramesCaughtByReceiver();
			if(detectors[i]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<i));
			// return the first one that works
			return ret;
		}

	return -1;
}

int multiSlsDetector::getFramesCaughtByReceiver() {

	int ret=0,ret1=0;
	int posmax = thisMultiDetector->numberOfDetectors;

	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return -1;
	}else{
		int* iret[posmax];
		for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
			if(detectors[idet]){
				iret[idet]= new int(0);
				Task* task = new Task(new func0_t<int>(&slsDetector::getFramesCaughtByReceiver,
						detectors[idet],iret[idet]));
				threadpool->add_task(task);
			}
		}
		threadpool->startExecuting();
		threadpool->wait_for_tasks_to_complete();

		for(int idet=0; idet<posmax; ++idet){
			if(detectors[idet]){
				if(iret[idet] != NULL){
					if(*iret[idet] == -1)	// could not connect
						ret = -1;
					else
						ret1+=(*iret[idet]);
					delete iret[idet];
				}else  ret = -1;
				if(detectors[idet]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<idet));
			}
		}
	}
	if ((!thisMultiDetector->numberOfDetectors) ||  (ret == -1))
		return ret;

	ret=(int)(ret1/thisMultiDetector->numberOfDetectors);
	return ret;
}



int multiSlsDetector::getReceiverCurrentFrameIndex() {
  int ret=0,ret1=0;
  for (int i=0; i<thisMultiDetector->numberOfDetectors; ++i)
    if (detectors[i]){
      ret1+=detectors[i]->getReceiverCurrentFrameIndex();
      if(detectors[i]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<i));
    }
  if(!thisMultiDetector->numberOfDetectors)
	  return ret;
  ret=(int)(ret1/thisMultiDetector->numberOfDetectors);

  return ret;
}



int multiSlsDetector::resetFramesCaught() {

	int ret=OK;
	int posmax = thisMultiDetector->numberOfDetectors;

	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return FAIL;
	}else{
		int* iret[posmax];
		for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
			if(detectors[idet]){
				iret[idet]= new int(OK);
				Task* task = new Task(new func0_t<int>(&slsDetector::resetFramesCaught,
						detectors[idet],iret[idet]));
				threadpool->add_task(task);
			}
		}
		threadpool->startExecuting();
		threadpool->wait_for_tasks_to_complete();

		for(int idet=0; idet<posmax; ++idet){
			if(detectors[idet]){
				if(iret[idet] != NULL){
					if(*iret[idet] != OK)
						ret = FAIL;
					delete iret[idet];
				}else  ret = FAIL;
				if(detectors[idet]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<idet));
			}
		}
	}

	return ret;
}



int multiSlsDetector::createReceivingDataSockets(const bool destroy){

	//number of sockets
	int numSockets = thisMultiDetector->numberOfDetectors;
	int numSocketsPerDetector = 1;
	if(getDetectorsType() == EIGER){
		numSocketsPerDetector = 2;
	}
	numSockets *= numSocketsPerDetector;

	if(destroy){
		 cprintf(MAGENTA,"Going to destroy data sockets\n");
		 //close socket
			for(int i=0;i<MAXDET;++i) {
				if (zmqSocket[i]) {
					delete zmqSocket[i];
					zmqSocket[i] = 0;
				}
			}
			client_downstream = false;
		 cout << "Destroyed Receiving Data Socket(s)" << endl;
		 return OK;
	}

	cprintf(MAGENTA,"Going to create data sockets\n");

	for(int i=0;i<numSockets; ++i){
		uint32_t portnum = 0;
		sscanf(detectors[i/numSocketsPerDetector]->getClientStreamingPort().c_str(),"%d",&portnum);
		portnum += (i%numSocketsPerDetector);

		zmqSocket[i] = new ZmqSocket(detectors[i/numSocketsPerDetector]->getReceiver().c_str(), portnum);
		if (zmqSocket[i]->IsError()) {
			cprintf(RED, "Error: Could not create Zmq socket on port %d\n", portnum);
			createReceivingDataSockets(true);
			return FAIL;
		}
		printf("Zmq Client[%d] at %s\n",i, zmqSocket[i]->GetZmqServerAddress());
	}

	client_downstream = true;
	cout << "Receiving Data Socket(s) created" << endl;
	return OK;
}









int multiSlsDetector::getData(const int isocket, const bool masking, int* image, const int size,
		uint64_t &acqIndex, uint64_t &frameIndex, uint32_t &subframeIndex, string &filename) {

	//fail is on parse error or end of acquisition
	if (!zmqSocket[isocket]->ReceiveHeader(isocket, acqIndex, frameIndex, subframeIndex, filename))
		return FAIL;

	//receiving incorrect size is replaced by 0xFF
	zmqSocket[isocket]->ReceiveData(isocket, image, size);

	//jungfrau masking adcval
	if(masking){
		unsigned int snel = size/sizeof(int);
		for(unsigned int i=0;i<snel;++i){
		    if((unsigned int)image[i] != 0xFFFFFFFF)
		        image[i] = (image[i] & 0x3FFF3FFF);
		}
	}

	return OK;
}





void multiSlsDetector::readFrameFromReceiver(){

	//determine number of half readouts and maxX and maxY
	int maxX=0,maxY=0;
	int numSockets = thisMultiDetector->numberOfDetectors;
	int numSocketsPerSLSDetector = 1;
	bool jungfrau = false;
	bool eiger = false;
	switch(getDetectorsType()){
	case EIGER:
	    eiger = true;
		numSocketsPerSLSDetector = 2;
		numSockets *= numSocketsPerSLSDetector;
		maxX = thisMultiDetector->numberOfChannel[X];
		maxY = thisMultiDetector->numberOfChannel[Y];
		break;
	case JUNGFRAU:
		jungfrau = true;
        maxX = thisMultiDetector->numberOfChannel[X];
        maxY = thisMultiDetector->numberOfChannel[Y];
		break;
	default:
		break;
	}

	//gui variables
	uint64_t currentAcquisitionIndex = -1;
	uint64_t currentFrameIndex = -1;
	uint32_t currentSubFrameIndex = -1;
	string currentFileName = "";

	//getting sls values
	int slsdatabytes = 0, slsmaxchannels = 0, slsmaxX = 0, slsmaxY=0, nx=0, ny=0;
	double bytesperchannel = 0;
	if(detectors[0]){
		slsdatabytes = detectors[0]->getDataBytes();
		slsmaxchannels = detectors[0]->getMaxNumberOfChannels();
		bytesperchannel = (double)slsdatabytes/(double)slsmaxchannels;
		slsmaxX = detectors[0]->getTotalNumberOfChannels(X);
		slsmaxY = detectors[0]->getTotalNumberOfChannels(Y);
	}

	//getting multi values
	nx = getTotalNumberOfChannels(slsDetectorDefs::X);
	ny = getTotalNumberOfChannels(slsDetectorDefs::Y);
	//calculating offsets (for eiger interleaving ports)
	int offsetX[numSockets]; int offsetY[numSockets];
	int bottom[numSockets];
	if(eiger){
		for(int i=0; i<numSockets; ++i){
			offsetY[i] = (maxY - (thisMultiDetector->offsetY[i/numSocketsPerSLSDetector] + slsmaxY)) * maxX * bytesperchannel;
			//the left half or right half
			if(!(i%numSocketsPerSLSDetector))
				offsetX[i] = thisMultiDetector->offsetX[i/numSocketsPerSLSDetector];
			else
				offsetX[i] = thisMultiDetector->offsetX[i/numSocketsPerSLSDetector] + (slsmaxX/numSocketsPerSLSDetector);
			offsetX[i] *= bytesperchannel;
			bottom[i] = detectors[i/numSocketsPerSLSDetector]->getFlippedData(X);/*only for eiger*/
		}
	}

	int expectedslssize = slsdatabytes/numSocketsPerSLSDetector;
	int* image = new int[(expectedslssize/sizeof(int))]();
	int nel=(thisMultiDetector->dataBytes)/sizeof(int);
	if(nel <= 0){
		cprintf(RED,"Error: Multislsdetector databytes not valid : %d\n", thisMultiDetector->dataBytes);
		return;
	}
	int* multiframe=new int[nel]();
	int nch;


	bool runningList[numSockets];
	bool connectList[numSockets];
	for(int i = 0; i < numSockets; ++i) {
		if(!zmqSocket[i]->Connect()) {
			connectList[i] = true;
			runningList[i] = true;
		} else {
			connectList[i] = false;
			cprintf(RED,"Error: Could not connect to socket  %s\n",zmqSocket[i]->GetZmqServerAddress());
			runningList[i] = false;
		}
	}
	int numRunning = numSockets;


	//wait for real time acquisition to start
	bool running = true;
	sem_wait(&sem_newRTAcquisition);
	if(checkJoinThread())
		running = false;


	//exit when last message for each socket received
	while(running){
		memset(((char*)multiframe),0xFF,slsdatabytes*thisMultiDetector->numberOfDetectors);	//reset frame memory

		//get each frame
		for(int isocket=0; isocket<numSockets; ++isocket){

			//if running
			if (runningList[isocket]) {
				//get individual images
				if(FAIL == getData(isocket, jungfrau, image, expectedslssize, currentAcquisitionIndex,currentFrameIndex,currentSubFrameIndex,currentFileName)){
					runningList[isocket] = false;
					--numRunning;
					continue;
				}

				//assemble data with interleaving
				if(eiger){

					//bottom
					if(bottom[isocket]){
					//if((((isocket/numSocketsPerSLSDetector)+1)%2) == 0){
						for(int i=0;i<slsmaxY;++i){
							memcpy(((char*)multiframe) + offsetY[isocket] + offsetX[isocket] + (int)((slsmaxY-1-i)*maxX*bytesperchannel),
									(char*)image+ (int)(i*(slsmaxX/numSocketsPerSLSDetector)*bytesperchannel),
									(int)((slsmaxX/numSocketsPerSLSDetector)*bytesperchannel));
						}
					}
					//top
					else{
						for(int i=0;i<slsmaxY;++i){
							memcpy(((char*)multiframe) + offsetY[isocket] + offsetX[isocket] + (int)(i*maxX*bytesperchannel),
									(char*)image+ (int)(i*(slsmaxX/numSocketsPerSLSDetector)*bytesperchannel),
									(int)((slsmaxX/numSocketsPerSLSDetector)*bytesperchannel));
						}
					}
				}

				//assemble data with no interleaving, assumed detectors appended vertically
				else{
				    for(int i=0;i<slsmaxY;++i){
				        memcpy(((char*)multiframe) + (((thisMultiDetector->offsetY[isocket] + i) * maxX) + thisMultiDetector->offsetX[isocket])* (int)bytesperchannel,
				                (char*)image+ (i*slsmaxX*(int)bytesperchannel),
				                (slsmaxX*(int)bytesperchannel));
				    }
				}
			}

		}


		//all done
		if(!numRunning){
			sem_wait(&sem_newRTAcquisition);
			//done with complete acquisition
			if(checkJoinThread())
				break;
			else{
				//starting a new scan/measurement
				for(int i = 0; i < numSockets; ++i)
					runningList[i] = true;
				numRunning = numSockets;
				running = false;
			}
		}

		//send data to callback
		if(running){
		  fdata = decodeData(multiframe,nch);
			if ((fdata) && (dataReady)){
				thisData = new detectorData(fdata,NULL,NULL,getCurrentProgress(),currentFileName.c_str(),nx,ny);
				dataReady(thisData, currentFrameIndex, currentSubFrameIndex, pCallbackArg);
				delete thisData;
				fdata = NULL;
				//cout<<"Send frame #"<< currentFrameIndex << " to gui"<<endl;
			}
			setCurrentProgress(currentAcquisitionIndex+1);
		}

		//setting it back for each scan/measurement
		running = true;
	}

	// Disconnect resources
	for (int i = 0; i < numSockets; ++i)
		if (connectList[i])
			zmqSocket[i]->Disconnect();

	//free resources
	delete [] image;
	delete[] multiframe;
}


int multiSlsDetector::lockReceiver(int lock) {

  int ret=-100, ret1;

  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      ret1=detectors[idet]->lockReceiver(lock);
      if(detectors[idet]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<idet));
       if (ret==-100)
	ret=ret1;
      else if (ret!=ret1)
	ret=-1;
    }
  }

  return ret;

}





string multiSlsDetector::getReceiverLastClientIP() {
  string s0="", s1="",s ;

  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      s=detectors[idet]->getReceiverLastClientIP();
      if(detectors[idet]->getErrorMask())
	setErrorMask(getErrorMask()|(1<<idet));
 
      if (s0=="")
	s0=s;
      else
	s0+=string("+")+s;
      if (s1=="")
	s1=s;
      else if (s1!=s)
	s1="bad";
    }
  }
  if (s1=="bad")
    return s0;
  else
    return s1;
}




int multiSlsDetector::exitReceiver() {

  int ival=FAIL, iv;
  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
    if (detectors[idet]) {
      iv=detectors[idet]->exitReceiver();
      if (iv==OK)
	ival=iv;
    }
  }
  return ival;
}



int multiSlsDetector::enableWriteToFile(int enable){
	int ret=-100, ret1;

	for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
		if (detectors[idet]) {
			ret1=detectors[idet]->enableWriteToFile(enable);
			if(detectors[idet]->getErrorMask())
			  setErrorMask(getErrorMask()|(1<<idet));
 			if (ret==-100)
				ret=ret1;
			else if (ret!=ret1)
				ret=-1;
		}
	}

	return ret;
}



int multiSlsDetector::overwriteFile(int enable){
	int ret=-100, ret1;

	for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
		if (detectors[idet]) {
			ret1=detectors[idet]->overwriteFile(enable);
			if(detectors[idet]->getErrorMask())
			  setErrorMask(getErrorMask()|(1<<idet));
 			if (ret==-100)
				ret=ret1;
			else if (ret!=ret1)
				ret=-1;
		}
	}

	return ret;
}


int multiSlsDetector::setFrameIndex(int index){
	int ret=-100;
	int posmax = thisMultiDetector->numberOfDetectors;
	fileIO::setFrameIndex(index);
	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return -1;
	}else{
		//return storage values
		int* iret[thisMultiDetector->numberOfDetectors];
		for(int idet=0; idet<posmax; ++idet){
			if(detectors[idet]){
				iret[idet]= new int(-1);
				Task* task = new Task(new func1_t<int,int>(&slsDetector::setFrameIndex,
						detectors[idet],index,iret[idet]));
				threadpool->add_task(task);
			}
		}
		threadpool->startExecuting();
		threadpool->wait_for_tasks_to_complete();
		for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
			if(detectors[idet]){
				if(iret[idet] != NULL){
					if (ret==-100)
						ret=*iret[idet];
					else if (ret!=*iret[idet])
						ret=-1;
					delete iret[idet];
				}else ret=-1;
				if(detectors[idet]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<idet));
			}
		}
	}
	return ret;
}




string multiSlsDetector::getErrorMessage(int &critical){

	int64_t multiMask,slsMask=0;
	string retval="";
	char sNumber[100];
	critical=0;

	multiMask = getErrorMask();
	if(multiMask){
		 if(multiMask & MULTI_DETECTORS_NOT_ADDED) {
			 retval.append("Detectors not added:\n"+string(getNotAddedList())+string("\n"));
			 critical = 1;
		 }
		 if(multiMask & MULTI_HAVE_DIFFERENT_VALUES) {
			 retval.append("A previous multi detector command gave different values\n"
					 "Please check the console\n");
			 critical = 0;
		 }

		  for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
		    if (detectors[idet]) {
		    	//if the detector has error
		    	if(multiMask&(1<<idet)){
					//append detector id
					sprintf(sNumber,"%d",idet);
					retval.append("Detector " + string(sNumber)+string(":\n"));
					//get sls det error mask
					slsMask=detectors[idet]->getErrorMask();
#ifdef VERYVERBOSE
					//append sls det error mask
					sprintf(sNumber,"0x%lx",slsMask);
					retval.append("Error Mask " + string(sNumber)+string("\n"));
#endif
					//get the error critical level
					if((slsMask>0xFFFFFFFF)|critical)
						critical = 1;
					//append error message
					retval.append(errorDefs::getErrorMessage(slsMask));

		    	}
		    }
		  }
	}
	return retval;
}


int64_t multiSlsDetector::clearAllErrorMask(){
	clearErrorMask();
	clearNotAddedList();
	for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet)
		if (detectors[idet])
			detectors[idet]->clearErrorMask();

	return getErrorMask();
}



int multiSlsDetector::calibratePedestal(int frames){
	int ret=-100, ret1;

	for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
		if (detectors[idet]) {
			ret1=detectors[idet]->calibratePedestal(frames);
			if(detectors[idet]->getErrorMask())
			  setErrorMask(getErrorMask()|(1<<idet));
 			if (ret==-100)
				ret=ret1;
			else if (ret!=ret1)
				ret=-1;
		}
	}

	return ret;
}

int multiSlsDetector::setReadReceiverFrequency(int freq){
	int ret=-100, ret1;

	for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
		if (detectors[idet]) {
			ret1=detectors[idet]->setReadReceiverFrequency(freq);
			if(detectors[idet]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<idet));
			if (ret==-100)
				ret=ret1;
			else if (ret!=ret1)
				ret=-1;
		}
	}

	return ret;
}



int multiSlsDetector::setReceiverReadTimer(int time_in_ms){
	int ret=-100, ret1;

	for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet) {
		if (detectors[idet]) {
			ret1=detectors[idet]->setReceiverReadTimer(time_in_ms);
			if(detectors[idet]->getErrorMask())
				setErrorMask(getErrorMask()|(1<<idet));
			if (ret==-100)
				ret=ret1;
			else if (ret!=ret1)
				ret=-1;
		}
	}

	return ret;
}

int multiSlsDetector::enableDataStreamingToClient(int enable) {
	if(enable >= 0){

		//destroy data threads
		if (!enable)
			createReceivingDataSockets(true);

		//create data threads
		else {
			if(createReceivingDataSockets() == FAIL){
				std::cout << "Could not create data threads in client." << std::endl;
				//only for the first det as theres no general one
				setErrorMask(getErrorMask()|(1<<0));
				detectors[0]->setErrorMask((detectors[0]->getErrorMask())|(DATA_STREAMING));
			}
		}
	}
	return client_downstream;
}

int multiSlsDetector::enableDataStreamingFromReceiver(int enable){
	if(enable >= 0){
		int ret=-100;
		if(!threadpool){
			cout << "Error in creating threadpool. Exiting" << endl;
			return -1;
		}else{
			//return storage values
			int* iret[thisMultiDetector->numberOfDetectors];
			for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
				if(detectors[idet]){
					iret[idet]= new int(-1);
					Task* task = new Task(new func1_t<int,int>(&slsDetector::enableDataStreamingFromReceiver,
							detectors[idet],enable,iret[idet]));
					threadpool->add_task(task);
				}
			}
			threadpool->startExecuting();
			threadpool->wait_for_tasks_to_complete();
			for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
				if(detectors[idet]){
					if(iret[idet] != NULL){
						if (ret==-100)
							ret=*iret[idet];
						else if (ret!=*iret[idet])
							ret=-1;
						delete iret[idet];
					}else ret=-1;
					if(detectors[idet]->getErrorMask())
						setErrorMask(getErrorMask()|(1<<idet));
				}
			}
		}
		thisMultiDetector->receiver_upstream = ret;
	}

	return thisMultiDetector->receiver_upstream;
}

int multiSlsDetector::enableReceiverCompression(int i){
	int ret=-100,ret1;
	for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet)
		if (detectors[idet]){
			ret1=detectors[idet]->enableReceiverCompression(i);
		    if(detectors[idet]->getErrorMask())
			  setErrorMask(getErrorMask()|(1<<idet));
			if(ret==-100)
				ret=ret1;
			else if (ret!=ret1)
				ret=-1;
		}
	return ret;
}



int multiSlsDetector::enableTenGigabitEthernet(int i){
	int ret=-100,ret1;
	for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet)
		if (detectors[idet]){
			ret1=detectors[idet]->enableTenGigabitEthernet(i);
		    if(detectors[idet]->getErrorMask())
			  setErrorMask(getErrorMask()|(1<<idet));
			if(ret==-100)
				ret=ret1;
			else if (ret!=ret1)
				ret=-1;
		}
	return ret;
}



int multiSlsDetector::setReceiverFifoDepth(int i){
	int ret=-100,ret1;
	for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet)
		if (detectors[idet]){
			ret1=detectors[idet]->setReceiverFifoDepth(i);
		    if(detectors[idet]->getErrorMask())
			  setErrorMask(getErrorMask()|(1<<idet));
			if(ret==-100)
				ret=ret1;
			else if (ret!=ret1)
				ret=-1;
		}
	return ret;
}


int multiSlsDetector::setReceiverSilentMode(int i){
	int ret=-100,ret1;
	for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet)
		if (detectors[idet]){
			ret1=detectors[idet]->setReceiverSilentMode(i);
		    if(detectors[idet]->getErrorMask())
			  setErrorMask(getErrorMask()|(1<<idet));
			if(ret==-100)
				ret=ret1;
			else if (ret!=ret1)
				ret=-1;
		}
	return ret;
}




  /** opens pattern file and sends pattern to CTB 
      @param fname pattern file to open
      @returns OK/FAIL
  */
int multiSlsDetector::setCTBPattern(string fname) {

	uint64_t word;
 
     int addr=0;

     FILE *fd=fopen(fname.c_str(),"r");
     if (fd>0) {
       while (fread(&word, sizeof(word), 1,fd)) {
	 for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet)
	   if (detectors[idet]){
	     detectors[idet]->setCTBWord(addr,word);
	   }
	// cout << hex << addr << " " << word << dec << endl;
	 ++addr;
       }
       
       fclose(fd);
     } else
       return -1;
     




  return addr;

}

  
  /** Writes a pattern word to the CTB
      @param addr address of the word, -1 is I/O control register,  -2 is clk control register
      @param word 64bit word to be written, -1 gets
      @returns actual value
  */
uint64_t multiSlsDetector::setCTBWord(int addr,uint64_t word) {
	uint64_t ret=-100,ret1;
	for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet)
		if (detectors[idet]){
		  ret1=detectors[idet]->setCTBWord(addr, word);
		    if(detectors[idet]->getErrorMask())
			  setErrorMask(getErrorMask()|(1<<idet));
			if(ret==(long long unsigned int)-100)
				ret=ret1;
			else if (ret!=ret1)
				ret=(long long unsigned int)-1;
		}
	return ret;
}
  



  /** Sets the pattern or loop limits in the CTB
      @param level -1 complete pattern, 0,1,2, loop level
      @param start start address if >=0
      @param stop stop address if >=0
      @param n number of loops (if level >=0)
      @returns OK/FAIL
  */
int multiSlsDetector::setCTBPatLoops(int level,int &start, int &stop, int &n) {


	int ret=-100,ret1;
	for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet)
		if (detectors[idet]){
		  ret1=detectors[idet]->setCTBPatLoops(level, start, stop, n);
		    if(detectors[idet]->getErrorMask())
			  setErrorMask(getErrorMask()|(1<<idet));
			if(ret==-100)
				ret=ret1;
			else if (ret!=ret1)
				ret=-1;
		}
	return ret;


}


  /** Sets the wait address in the CTB
      @param level  0,1,2, wait level
      @param addr wait address, -1 gets
      @returns actual value
  */
int multiSlsDetector::setCTBPatWaitAddr(int level, int addr) {

  

	int ret=-100,ret1;
	for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet)
		if (detectors[idet]){
		  ret1=detectors[idet]->setCTBPatWaitAddr(level, addr);
		    if(detectors[idet]->getErrorMask())
			  setErrorMask(getErrorMask()|(1<<idet));
			if(ret==-100)
				ret=ret1;
			else if (ret!=ret1)
				ret=-1;
		}
	return ret;



}

   /** Sets the wait time in the CTB
      @param level  0,1,2, wait level
      @param t wait time, -1 gets
      @returns actual value
  */
int multiSlsDetector::setCTBPatWaitTime(int level, uint64_t t) {


	int ret=-100,ret1;
	for (int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet)
		if (detectors[idet]){
		  ret1=detectors[idet]->setCTBPatWaitTime(level,t);
		    if(detectors[idet]->getErrorMask())
			  setErrorMask(getErrorMask()|(1<<idet));
			if(ret==-100)
				ret=ret1;
			else if (ret!=ret1)
				ret=-1;
		}
	return ret;

}


int multiSlsDetector::pulsePixel(int n,int x,int y) {
	int ret=-100;

	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return -1;
	}else{
		//return storage values
		int* iret[thisMultiDetector->numberOfDetectors];
		for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
			if(detectors[idet]){
				iret[idet]= new int(-1);
				Task* task = new Task(new func3_t<int,int,int,int>(&slsDetector::pulsePixel,
						detectors[idet],n,x,y,iret[idet]));
				threadpool->add_task(task);
			}
		}
		threadpool->startExecuting();
		threadpool->wait_for_tasks_to_complete();
		for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
			if(detectors[idet]){
				if(iret[idet] != NULL){
					if (ret==-100)
						ret=*iret[idet];
					else if (ret!=*iret[idet])
						ret=-1;
					delete iret[idet];
				}else ret=-1;
				if(detectors[idet]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<idet));
			}
		}
	}
	return ret;
}


int multiSlsDetector::pulsePixelNMove(int n,int x,int y) {
	int ret=-100;

	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return -1;
	}else{
		//return storage values
		int* iret[thisMultiDetector->numberOfDetectors];
		for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
			if(detectors[idet]){
				iret[idet]= new int(-1);
				Task* task = new Task(new func3_t<int,int,int,int>(&slsDetector::pulsePixelNMove,
						detectors[idet],n,x,y,iret[idet]));
				threadpool->add_task(task);
			}
		}
		threadpool->startExecuting();
		threadpool->wait_for_tasks_to_complete();
		for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
			if(detectors[idet]){
				if(iret[idet] != NULL){
					if (ret==-100)
						ret=*iret[idet];
					else if (ret!=*iret[idet])
						ret=-1;
					delete iret[idet];
				}else ret=-1;
				if(detectors[idet]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<idet));
			}
		}
	}
	return ret;
}


int multiSlsDetector::pulseChip(int n) {
	int ret=-100;

	if(!threadpool){
		cout << "Error in creating threadpool. Exiting" << endl;
		return -1;
	}else{
		//return storage values
		int* iret[thisMultiDetector->numberOfDetectors];
		for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
			if(detectors[idet]){
				iret[idet]= new int(-1);
				Task* task = new Task(new func1_t<int,int>(&slsDetector::pulseChip,
						detectors[idet],n,iret[idet]));
				threadpool->add_task(task);
			}
		}
		threadpool->startExecuting();
		threadpool->wait_for_tasks_to_complete();
		for(int idet=0; idet<thisMultiDetector->numberOfDetectors; ++idet){
			if(detectors[idet]){
				if(iret[idet] != NULL){
					if (ret==-100)
						ret=*iret[idet];
					else if (ret!=*iret[idet])
						ret=-1;
					delete iret[idet];
				}else ret=-1;
				if(detectors[idet]->getErrorMask())
					setErrorMask(getErrorMask()|(1<<idet));
			}
		}
	}

	return ret;
}


void multiSlsDetector::setAcquiringFlag(bool b){
	thisMultiDetector->acquiringFlag = b;
}

bool multiSlsDetector::getAcquiringFlag(){
	return thisMultiDetector->acquiringFlag;
}


bool multiSlsDetector::isAcquireReady() {
	if (thisMultiDetector->acquiringFlag) {
		std::cout << "Acquire has already started. If previous acquisition terminated unexpectedly, reset busy flag to restart.(sls_detector_put busy 0)" << std::endl;
		return FAIL;
	}

	thisMultiDetector->acquiringFlag = true;
	return OK;
}

