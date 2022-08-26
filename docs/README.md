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
To install cv2-contrib. 
However, you may damage OS again...

### Migrate to python3
Replace filterpy with custom Kalman-Filter. 
However, you will have to regularly source tf2_ros...
 


#### Kalman Filter module and linear assignment function depends on scipy. To install scipy for python2 on jetson nano you shall try:
```
python -m pip install scipy
```
To install filterpy (Kalman Filter), try:
```
pip install filterpy
```

#### cv2 module has no attribute named aruco
cv2.aruco comes from opencv-contrib. To install it, try:
```
pip install opencv-contrib-python
```
Before building, you could will also need to install packages that numpy and scipy depend on
