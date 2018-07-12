INCLUDES	=   -I . 
SRC_TEST	= 	RegExUtils.cpp CameraReceiver.cpp CameraReceivers.cpp Camera.cpp main.cpp
LIBDIR		= 	../../bin
ZLIBDIR		= 	../../slsReceiverSoftware/include
LDFLAG_DET 	=	-I. -L$(LIBDIR) -L$(ZLIBDIR) -lSlsReceiver -lSlsDetector -L/usr/lib64/ -pthread -lrt -L. -lzmq
#LDFLAG_DET 	=	-I. -L$(LIBDIR) -L$(ZLIBDIR) -lSlsReceiver -lSlsDetector -pthread -lrt -L. -lzmq32

all: detTest 
	
detTest:$(SRC_TEST)
	echo "creating test software"
	mkdir -p bin
	g++ -o bin/detTest $(SRC_TEST) $(INCLUDES) $(LDFLAG_DET) -lm -lstdc++
	cp bin/detTest ../../bin
	
clean:
	echo "cleaning test software"
	rm -rf bin/detTest
