dfDrone
=======

detect and follow drone

**Note to compile the cpp library do**
* g++ -c -fPIC foo.cpp -o foo.o
* g++ -shared -Wl,-soname,libfoo.so -o libfoo.so  foo.o

**TODO**
* on a side note, I need to get this on bitbucket
* currently only using a typical webcam, need to integrate use of kinect with depth sensor
* create data base of positive blobs
* add velocity vectors into queue (from center!)
* create/send twist messages based on velocity vectors

