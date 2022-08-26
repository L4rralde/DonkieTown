# DonkieTown
## A low-cost experimental platform for research on Intelligent Vehicles. 

DonkieTown consists of one or more differential-drive robots called Asinus cars, a ground station, a localization system and a series of trusted techniques that easily allow the development of testbeds to implement and validate different strategies for collaborative autonomous driving, and study a variety of cases of study.

## Trouble shooting.
cv2-contrib-python is not available for python2 for jetson nano. 
filterpy recquires a version of numpy uncompatible with cv2-contrib.
I could install scipy for python2 and python3. I recommend scipy for linear assignment problem because it is a wrapper for c code.

### Solutions
#### The easy one
Use ground station for tracking. 

### Keep trying pyhon2
Scipy and openCV shall be installed from source.
 


#### Kalman Filter module and linear assignment function depends on scipy. To install scipy for python2 on jetson nano you shall try:
```
python -m pip install scipy
```
To install filterpy (Kalman Filter), try:
```
pip install filterpy
```

#### cv2 module has no attribute named aruco
Build source of openCV 4.5.0 for jetson nano with cuda enabled as described in: https://qengineering.eu/install-opencv-4.5-on-jetson-nano.html

