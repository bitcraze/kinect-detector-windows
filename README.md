# kinect-detector-windows
Crazyflie detector using Kinect 2. Windows implementation using Microsoft's Kinect SDK.
This program detects the Crazyflie quadcopter and sends its position (x,y,z, angle) using a ZeroMQ socket. You can find more
information about the system setup (ie. how to autonomously fly a Crazyflie) in the Bitcraze wiki: https://wiki.bitcraze.io/doc:crazyflie:vision:index

The project is developed using Visual Studio 2013 Community edition.

Dependencies:
 - The Microsoft Kinect for Windows SDK 2.0
 - ZeroMQ version 4.0.4 installed in default location. You may need to copy the ZeroMQ DLL in the project folder
 - Opencv 2.4.11 at location set in the environment variable $(OPENCV_DIR)
