OBJ=node.o CTime.o CSerialConnection.o CLidarPacketSender.o CArcTime.o CLidarPacketReceiver.o CLidarPacket.o CCountDown.o C3iroboticsLidar.o  CDeviceConnection.o modify.o uart.o carcontrol.o led.o motor.o inputdetect.o driverInit.o
#CC=g++
CC=arm-linux-gnueabihf-g++
#lidar_ubuntu:$(OBJ)
lidar_de10nano:$(OBJ)
#stop:$(OBJ)
#	$(CC) -lrt -pthread -o lidar_ubuntu $(OBJ)
	$(CC) -std=c++11 -lrt -pthread -o lidar_de10nano $(OBJ)
#	$(CC) -std=c++11 -lrt -pthread -o stop $(OBJ)

node.o:node.cpp
	$(CC) -std=c++11 -lrt -pthread -c node.cpp

modify.o:modify.cpp
	$(CC) -std=c++11 -lrt -pthread -c modify.cpp

uart.o:uart.cpp
	$(CC) -std=c++11 -lrt -pthread -c uart.cpp

carcontrol.o:carcontrol.cpp
	$(CC) -std=c++11 -lrt -pthread -c carcontrol.cpp
    
CTime.o:CTime.cpp
	$(CC) -std=c++11 -lrt -pthread -c CTime.cpp

CSerialConnection.o:CSerialConnection.cpp
	$(CC) -std=c++11 -lrt -pthread -c CSerialConnection.cpp

CLidarPacketSender.o:CLidarPacketSender.cpp
	$(CC) -std=c++11 -lrt -pthread -c CLidarPacketSender.cpp

CLidarPacketReceiver.o:CLidarPacketReceiver.cpp
	$(CC) -std=c++11 -lrt -pthread -c CLidarPacketReceiver.cpp

CLidarPacket.o:CLidarPacket.cpp
	$(CC) -std=c++11 -lrt -pthread -c CLidarPacket.cpp

CArcTime.o:CArcTime.cpp
	$(CC) -std=c++11 -lrt -pthread -c CArcTime.cpp

C3iroboticsLidar.o:C3iroboticsLidar.cpp
	$(CC) -std=c++11 -lrt -pthread -c C3iroboticsLidar.cpp

CDeviceConnection.o:CDeviceConnection.cpp
	$(CC) -std=c++11 -lrt -pthread -c CDeviceConnection.cpp

CCountDown.o:CCountDown.cpp
	$(CC) -std=c++11 -lrt -pthread -c CCountDown.cpp
	
led.o:led.cpp
	$(CC) -std=c++11 -lrt -pthread -c led.cpp 
	
motor.o:motor.cpp
	$(CC) -std=c++11 -lrt -pthread -c motor.cpp
	
inputdetect.o:inputdetect.cpp
	$(CC) -std=c++11 -lrt -pthread -c inputdetect.cpp
	
driverInit.o:driverInit.cpp
	$(CC) -std=c++11 -lrt -pthread -c driverInit.cpp


.PHONY: clean 
clean:
	rm -f $(TARGET) *.a *.o *~  
