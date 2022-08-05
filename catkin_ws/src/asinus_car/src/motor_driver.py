#!/usr/bin/python 
import rospy 
from asinus import AsinusMotors as motors 
from donkietown_msgs.msg import MotorsSpeed, MotorsState
from time import sleep

class motor_driver_node: 
	def __init__(self,car_id): 
		self.car_id = car_id
		self.motors = motors()
		self.motors.stop()
		rospy.on_shutdown(self.shutdown)
		self.measures_pub = rospy.Publisher('/asinus_cars/'+str(car_id)+'/motors_raw_data',MotorsState,queue_size=1)
		self.driver_sub = rospy.Subscriber('/asinus_cars/'+str(car_id)+'/motors_driver',MotorsSpeed, self.on_drive)
		self.motor_st = MotorsState()
		#self.motor_st.header.frame_id = "base_link" #Unnecessary 
	def on_drive(self,speed_msg):
		self.motors.setSpeeds([speed_msg.leftMotor,speed_msg.rightMotor])
		self.motor_st.header.stamp = rospy.Time.now()
		sleep(0.01) #Increase time gap between i2c transactions.
		(sp_left,sp_right,volt) = self.motors.getMeasures()
		self.motor_st.speed.leftMotor = sp_left
		self.motor_st.speed.rightMotor = sp_right
		self.motor_st.voltage = volt
		sleep(0.01) #Increase time gap between i2c transactions.
		try:
			self.measures_pub.publish(motor_st)
		except: 
			pass
	def shutdown(self):
		print("shutdown!")
		rospy.sleep(1)
		self.motors.stop()

def main():
	rospy.init_node('motor_driver_',anonymous=True)
	robot_id = rospy.get_param("~car_id") #Unique for each vehicle  
	node = motor_driver_node(robot_id)
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("motorDriver node terminated.")

if __name__ == '__main__':
    main()

