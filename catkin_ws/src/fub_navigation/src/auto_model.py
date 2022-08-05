import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped as PCS
from std_msgs.msg import Int16
#from autominy_msgs.msg import NormalizedSteeringCommand, SpeedCommand
from sensor_msgs.msg import LaserScan, PointCloud

def get_AutoModel(model, callbacks, fake_gps=False, car_id=7): 
    if("AutoModelMini" in model): 
        return AutoModelMini(callbacks,model,fake_gps,car_id)
    if("AutoModel_Obstacle" in model): 
        return AutoModelMini(callbacks,model,fake_gps,car_id)
    if("AutoMiny" in model): 
        return AutoMiny(callbacks,model,fake_gps,car_id)
    print("Couldn't find model named: "+str(model))

class AutoModelMini(): 
    def __init__(self,callbacks,model,fake_gps=False,car_id=7):
        odom_callback = callbacks[0] 
        obs_callback = callbacks[1] 
        ns = "/"+model
        self.pub_speed = rospy.Publisher(ns+"/manual_control/speed", Int16,
                                         queue_size=1, tcp_nodelay=True)
        self.pub = rospy.Publisher(ns+"/manual_control/steering", Int16,
                                   queue_size=1, tcp_nodelay=True)
        if fake_gps: 
            self.sub_odom = rospy.Subscriber("/fake_gps/ego_pose_raw/"+str(car_id),PCS, odom_callback, queue_size=1)
        else: 
            self.sub_odom = rospy.Subscriber(ns+"/Odometry", Odometry, odom_callback, queue_size=1)
        self.sub_obs = rospy.Subscriber("/sensors/obstacles",PointCloud,obs_callback, queue_size=1)
    def publish_speed(self,speed): 
        self.pub_speed.publish(int(-826.66*speed))
    def publish_steer(self,steering):
        steer_deg = steering*90
        steering = steer_deg+90
        self.pub.publish(int(steering))
"""
class AutoMiny():
    def __init__(self,callbacks,model,fake_gps=False,car_id=7): 
        odom_callback = callbacks[0]
        lidar_callback = callbacks[1]
        self.pub_speed = rospy.Publisher("/actuators/speed", SpeedCommand,
                                         queue_size=1, tcp_nodelay=True)
        self.pub = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand,
                                   queue_size=1, tcp_nodelay=True)
        if fake_gps: 
            self.sub_odom = rospy.Subscriber("/fake_gps/ego_pose_raw/"+str(car_id),PCS, odom_callback, queue_size=1)
        else:
            self.sub_odom = rospy.Subscriber("/sensors/localization/filtered_map", Odometry, odom_callback, queue_size=1)
        self.lidar_sub = rospy.Subscriber("/sensors/rplidar/scan", LaserScan, lidar_callback, queue_size=1)
    def publish_speed(self,speed):
        msg = SpeedCommand()
        msg.value = speed
        msg.header.frame_id = "base_link"
        msg.header.stamp = rospy.Time.now()
        self.pub_speed.publish(msg)
    def publish_steer(self,steering):
        if steering > 1:
            steering = 1
        if steering < -1:
            steering = -1
        steerMsg = NormalizedSteeringCommand()
        steerMsg.value = steering
        steerMsg.header.frame_id = "base_link"
        steerMsg.header.stamp = rospy.Time.now()
        self.pub.publish(steerMsg)
"""
