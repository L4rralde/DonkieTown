# DonkieTown
## A low-cost experimental platform for research on Intelligent Vehicles. 

DonkieTown consists of one or more differential-drive robots called Asinus cars, a ground station, a localization system and a series of trusted techniques that easily allow the development of testbeds to implement and validate different strategies for collaborative autonomous driving, and study a variety of cases of study.

## Instructions
- Flash Jetpack 4.5 on 64GB pendrive with balena etcher or flash jetbot 0.4.3 image as described in https://jetbot.org/master/software_setup/sd_card.html

- Install ros melodic and ros packages for jetson_nano as described in https://github.com/dusty-nv/jetbot_ros/tree/melodic 

- Install scipy:
```
python -m pip install scipy
```
- Install filterpy
```
pip install filterpy
```
- Build pylibi2c from: https://github.com/amaork/libi2c

- Build source of openCV 4.5.0 for jetson nano with cuda enabled as described in: https://qengineering.eu/install-opencv-4.5-on-jetson-nano.html


## Troubleshooting