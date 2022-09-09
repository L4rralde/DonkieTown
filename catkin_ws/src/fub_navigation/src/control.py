#!/usr/bin/env python2
import math
import numpy as np
import rospkg
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int16
import tf
import cv2 
from auto_model import get_AutoModel
         
#TODO: 
#   -Enhance vector matrix selection. 
#   -Modify inhouse desired path to be reliable for car-like robots. How to draw curves in inkspace? Drawing curves separately splits svg path. 
#       Solution: Draw a circle. Done!
#   -Generate 0, +-25, and +-50 lanes. 
    
class MapConfig(object):
    def __init__(self,map_name,look_ahead='50cm'): 
        rospack = rospkg.RosPack()
        path = rospack.get_path('fub_navigation')+'/scripts/maps/'+map_name+'/'
        map_img = cv2.imread(path+'map.png')
        (h,w,l) = map_img.shape
        self.map_size_x = w  # cm
        self.map_size_y = h  # cm
        self.resolution = 1  #More than an argument, it's a parameter to meet. 

       #Do not charge all of them 

        self.matrix_lane_1 = np.load(path + 'matrix'+look_ahead+'_lane2.npy')
        self.matrix_lane_2 = np.load(path + 'matrix'+look_ahead+'_lane1.npy')
        #self.matrix_lane_3 = np.load(path + 'matrix'+look_ahead+'_lane3.npy')
        self.matrix_nlane_1 = np.load(path + 'matrix-'+look_ahead+'_lane2.npy')
        self.matrix_nlane_2 = np.load(path + 'matrix-'+look_ahead+'_lane1.npy')
        #self.matrix_nlane_3 = np.load(path + 'matrix-'+look_ahead+'_lane3.npy')
        self.distance_lane_1 = np.load(path + 'matrix0cm_lane2.npy')
        self.distance_lane_2 = np.load(path + 'matrix0cm_lane1.npy')
        #self.distance_lane_3 = np.load(path + 'matrix0cm_lane3.npy')        
        
    
class VectorfieldController(MapConfig):
    def __init__(self,map_name):
	rospy.init_node('VectorfieldController')
        model = rospy.get_param('~model','AutoModelMini')
        self.lane = rospy.get_param('~lane',1)
        look_ahead = rospy.get_param('~look_ahead','25cm')
	car_id = rospy.get_param('~car_id','11')
        super(VectorfieldController,self).__init__(map_name,look_ahead)
        #self.model_car = AutoModelMini([self.callback],model)
        #callbacks = [self.callback,self.on_obs_detection] #Ackermann
        callbacks = [self.ddr_callback,self.on_obs_detection] #ddr
        
        self.model_car = get_AutoModel(model,callbacks=callbacks,
					fake_gps=True, car_id=car_id)
        
        self.x = 0.0
        self.y = 0.0 

        self.last_lane_change = rospy.Time.now()
        self.speed_value = rospy.get_param('~speed',0.3)
        #self.last_var = [0.0] #AutoMiny: Last angle
        self.last_var = [0.0,0.0] #DDR. Last p*, last q*.
        #self.Ks = [4.0,0.2,0.000] #PID for autominy
        self.Ks = [0.4,0.4] #Kn1, kn2 for ddr
        self.last_time = rospy.Time.now()
        self.integral_error = 0.0
        self.listener = tf.TransformListener()

        print("speed", self.speed_value)
        if self.lane == 1:
            self.matrix = self.matrix_lane_1
            self.rmatrix = self.matrix_nlane_1
        else:
            self.matrix = self.matrix_nlane_2
            self.rmatrix = self.matrix_lane_2

        rospy.on_shutdown(self.shutdown)

        self.shutdown_ = False

        #self.lidar_sub = rospy.Subscriber("/sensors/rplidar/scan", LaserScan, self.on_lidar, queue_size=1)

    def ddr_callback(self,data):
        dt = (data.header.stamp - self.last_time).to_sec()
        # 25hz
        if dt < 0.04:
            return
        self.last_time = data.header.stamp
        self.x = data.pose.pose.position.x #Actual coord-x
        self.y = data.pose.pose.position.y #Actual coord-y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list) #Actual yaw
        delta_xd,delta_yd = get_coords_from_vf(self.x,self.y,self.resolution,self.map_size_x,self.map_size_y,self.matrix)
        #eccentric point ddr controller
        ps = delta_xd+self.x
        qs = delta_yd+self.y
	print(ps, qs)
        dps = (ps-self.last_var[0])/dt
        dqs = (qs-self.last_var[1])/dt
        self.last_var[0] = ps
        self.last_var[1] = qs
        nu1 = dps+self.Ks[0]*(delta_xd)
        nu2 = dqs+self.Ks[1]*(delta_yd)
        speed = np.cos(yaw)*nu1+np.sin(yaw)*nu2
        omega = 14.2857*(-np.sin(yaw)*nu1+np.cos(yaw)*nu2) #where 14.2857 = 1/mu, mu:eccentrecity. Taking mu=0.07

        self.model_car.publish_steer(omega)
        if not self.shutdown_:
            self.model_car.publish_speed(speed)

        
    def callback(self, data):
        dt = (data.header.stamp - self.last_time).to_sec()
        # 25hz
        if dt < 0.04:
            return
        self.last_time = data.header.stamp
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        x3,y3 = get_coords_from_vf(self.x,self.y,self.resolution,self.map_size_x,self.map_size_y,self.matrix)
        f_x = np.cos(yaw) * x3 + np.sin(yaw) * y3
        f_y = -np.sin(yaw) * x3 + np.cos(yaw) * y3

        angle = np.arctan2(f_y, f_x)

        self.integral_error = self.integral_error + angle * dt
        steering = self.Ks[0] * angle + self.Ks[1] * ((angle - self.last_var[0]) / dt) + self.Ks[2] * self.integral_error
        self.last_var[0] = angle

        if f_x > 0:
            speed = -self.speed_value
        else:
            speed = self.speed_value

        gain = 1#+np.exp(-10.0*abs(steering))
        if f_x > 0:
            speed = max(self.speed_value*gain, gain*(speed* ((np.pi / 3) / (abs(steering) + 1))))

        self.model_car.publish_steer(steering)

        if not self.shutdown_:
            self.model_car.publish_speed(speed)

    def shutdown(self):
        print("shutdown!")
        self.shutdown_ = True
        self.model_car.publish_speed(0)
        rospy.sleep(1)

    def lane_change(self):
        if (rospy.Time.now() - self.last_lane_change).to_sec() < 1.0:
            return

        if self.lane == 1:
            self.matrix = self.matrix_lane_2
            self.lane = 2
        else:
            self.matrix = self.matrix_lane_1
            self.lane = 1
        self.last_lane_change = rospy.Time.now()
        print("lane change")

    def on_lidar(self, msg):
        points = []
        for i in range(len(msg.ranges) - 1, int(len(msg.ranges) * 0.85), -1):
            dist = msg.ranges[i]
            angle = i * msg.angle_increment
            if dist < 1.3:
                points.append((-dist * np.cos(angle), -dist * np.sin(angle)))

        for i in range(0, int(len(msg.ranges) * 0.15), 1):
            dist = msg.ranges[i]
            angle = i * msg.angle_increment
            if dist < 1.3:
                points.append((-dist * np.cos(angle), -dist * np.sin(angle)))

        points_on_track = 0

        if len(points) > 0:
            (t,r) = self.listener.lookupTransform("map", msg.header.frame_id, rospy.Time(0))
            mat44 = np.dot(tf.transformations.translation_matrix(t), tf.transformations.quaternion_matrix(r))

            for (x, y) in points:
                (xm, ym, zm) = tuple(np.dot(mat44, np.array([x, y, 0, 1.0])))[:3]
                (xi, yi) = int(xm * (100 / self.resolution)), int(ym * (100 / self.resolution))

                if 0 <= xi < 600 and 0 <= yi < 430:
                    if self.lane == 1:
                        (xd, yd) = self.distance_lane_1[xi, yi, :]
                    else:
                        (xd, yd) = self.distance_lane_2[xi, yi, :]
                    dist = np.sqrt(xd ** 2.0 + yd ** 2.0)
                    if dist < 0.15:
                        points_on_track += 1

        if points_on_track > 50:
            self.lane_change()
        pass

    def on_obs_detection(self,msg): 
        pts = msg.points 
        for pt in pts: 
            dist = np.sqrt((self.x-pt.x)**2+(self.y-pt.y)**2)
            print(dist)
            xm = pt.x+self.map_size_x/200.0
            ym = pt.y+self.map_size_y/200.0

            (xi, yi) = int(xm*(100/self.resolution)), int(ym*(100/self.resolution))
            if self.lane == 1:
                (xd, yd) = self.distance_lane_1[xi, yi, :]
            else:
                (xd, yd) = self.distance_lane_2[xi, yi, :]
            lane_dist = np.sqrt(xd ** 2.0 + yd ** 2.0)
            if lane_dist < 0.1 and dist < 0.75:
                print("Obstacle detected")
                self.lane_change()
                break

def get_coords_from_vf(raw_x, raw_y, resolution, map_size_x, map_size_y, matrix):
    #To match vector field
    x = raw_x+map_size_x/200.0
    y = raw_y+map_size_y/200.0
    x_index_floor = int(math.floor(x * (100.0 / resolution))) #Converting to cm
    y_index_floor = int(math.floor(y * (100.0 / resolution)))

    x_index_ceil = x_index_floor + 1
    y_index_ceil = y_index_floor + 1

    ceil_ratio_x = x * (100.0 / resolution) - x_index_floor #To check how much interpolate between floor and ceil.
    ceil_ratio_y = y * (100.0 / resolution) - y_index_floor

    if x_index_floor < 0:
        x_index_floor = 0
    if x_index_floor > map_size_x / resolution - 1:
        x_index_floor = map_size_x / resolution - 1

    if y_index_floor < 0:
        y_index_floor = 0
    if y_index_floor > map_size_y / resolution - 1:
        y_index_floor = map_size_y / resolution - 1

    if x_index_ceil < 0:
        x_index_ceil = 0
    if x_index_ceil > map_size_x / resolution - 1:
        x_index_ceil = map_size_x / resolution - 1

    if y_index_ceil < 0:
        y_index_ceil = 0
    if y_index_ceil > map_size_y / resolution - 1:
        y_index_ceil = map_size_y / resolution - 1

    vx_floor, vy_floor = matrix[x_index_floor, y_index_floor, :]
    vx_ceil, vy_ceil = matrix[x_index_ceil, y_index_ceil, :]
    vx = vx_floor * (1.0 - ceil_ratio_x) + vx_ceil * ceil_ratio_x
    vy = vy_floor * (1.0 - ceil_ratio_y) + vy_ceil * ceil_ratio_y
    return (vx,vy)

def main():
    mname = 'inhouse'
    try:
        VectorfieldController(map_name=mname)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("VectorfieldController node terminated.")


if __name__ == '__main__':
    main()
