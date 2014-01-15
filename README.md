dfDrone
=======

detect and follow drone

**Note to compile the cpp library do**
* g++ -c -fPIC foo.cpp -o foo.o
* g++ -shared -Wl,-soname,libfoo.so -o libfoo.so  foo.o

**TODO**
* currently only using a typical webcam, need to integrate use of kinect with depth sensor
* create data base of positive blobs
* create ROS environment (generic)
* add velocity vectors into queue (from center!)
* create/send twist messages based on velocity vectors

