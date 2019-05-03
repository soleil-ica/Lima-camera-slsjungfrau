#!/bin/bash

mvn clean install -f pom_64.xml -o
read -rsn1 -p"Press any key to continue";echo

cp target/nar/lib/amd64-Linux-g++/shared/libLimaSlsJungfrau-amd64-Linux-gcc-shared-release-1.0.0.so ../../../DeviceServers/Linux64/

