#!/usr/bin/env python2
#Works only for all-same-oriented-fiducial-marks scenarios
#Always taking paramters.yaml as camera model, even when gazebo model is the one. To be fixed. Still pending?
import rospy
import cv2
import sys
import yaml 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tf
import tf2_ros
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from framework import FrameBr as fw
from node_cfg import marks_offset
from camera_misc import CameraModel
from geometry_msgs.msg import PoseWithCovarianceStamped as PCS

class Marker(object):
	def __init__(self,mid,upcam_id,stamp,corners):
		self.mid = mid
		self.tf_name = str(upcam_id)+"_marker_"+mid
		self.frame = fw("map",self.tf_name)
		self.update(stamp,corners)
		self.upcam_id = upcam_id
	def update(self,stamp,corners):
		self.stamp = stamp
		self.corners = corners
	def broadcast_tf(self,rvec,tvec):
		rvec = np.reshape(rvec,(3))
		tvec = np.reshape(tvec,(3))
		self.frame.update_from_rotvec(rvec,tvec)
		self.frame.broadcast()

class MobileMarker(Marker):
	def __init__(self,robot_id,upcam_id,stamp,corners):
		super(MobileMarker,self).__init__(robot_id,upcam_id,stamp,corners)
		self.posePub = rospy.Publisher("/fake_gps/ego_pose_raw/"+str(robot_id),PCS,queue_size=1)
		self.init_message()
	def init_message(self):
		self.pose_msg = PCS()
		self.pose_msg.header.frame_id = "map"
		self.pose_msg.pose.covariance = [0.01,0,0,0,0,0,
										 0,0.01,0,0,0,0,
										 0,0,0,0,0,0, 
										 0,0,0,1e-3,0,0,
										 0,0,0,0,1e-3,0,
										 0,0,0,0,0,0.03]
		self.pose_msg.pose.pose.position.z = 0.0	
	def get_relative_pose(self,refmark_id,lookup_transform):
		father_tf = str(self.upcam_id)+"_marker_"+str(refmark_id)
		trans = lookup_transform(father_tf,self.tf_name,rospy.Time(0))
		(xo,yo,tho) = marks_offset[str(refmark_id)] #Add reference mark pose offset. 
		self.pose_msg.header.stamp = trans.header.stamp 
		self.pose_msg.pose.pose.position.x = trans.transform.translation.x+xo
		self.pose_msg.pose.pose.position.y = trans.transform.translation.y+yo
		#Assume ego mark is inside x-y plane. 
		qx = trans.transform.rotation.x
		qy = trans.transform.rotation.y
		qz = trans.transform.rotation.z
		qw = trans.transform.rotation.w
		(roll,pitch,yaw) = euler_from_quaternion([qx,qy,qz,qw])
		quat = quaternion_from_euler(0.0,0.0,yaw+tho)
		self.pose_msg.pose.pose.orientation.x = quat[0]
		self.pose_msg.pose.pose.orientation.y = quat[1]
		self.pose_msg.pose.pose.orientation.z = quat[2]
		self.pose_msg.pose.pose.orientation.w = quat[3]
	def publish(self):
		self.posePub.publish(self.pose_msg) #Publish ego mark pose. 
		
class fake_gps: 
	def __init__(self, cam=0, arucoDict=cv2.aruco.DICT_4X4_50, cam_id=0,camera_src='internal', refids={'0'}, markerLen=0.1, paramfile='calibration.yaml'): 
		self.bridge = CvBridge() #When using rosbag as src image
		self.arucoDict = cv2.aruco.Dictionary_get(arucoDict) 
		self.markerLen = markerLen #square mark lenght 
		self.cam_id = cam_id #To allow multiple cameras running
		self.refids = refids #Set of known reference markers' ids
		self.time = rospy.Time.now()
		self.tfBuffer = tf2_ros.Buffer() #This and the following are used for tf transformations
		self.listener = tf2_ros.TransformListener(self.tfBuffer) #This is recquired to import all TF transformations	
		#topic_bn = '/sensors/global_camera_'+str(self.cam_id)
		topic_bn = '/sensors/global_camera'
		self.cam_model = CameraModel(paramfile=paramfile,topic=topic_bn+'/info')
		if camera_src == 'internal': 
			self.cam = cv2.VideoCapture(cam)
		else:
			self.view_sub = rospy.Subscriber(topic_bn+'/image', Image, self.callback)
			if camera_src == 'gazebo': 
				del self.cam_model
				self.cam_model = CameraModel(kind="subscriber",topic=topic_bn+'/info')
	def talker(self):
		while not rospy.is_shutdown():
			foo,frame = self.cam.read()
			self.get_raw_poses(frame)
	def callback(self,img): #Only executed alongside rosbag.
		try: 
			frame = self.bridge.imgmsg_to_cv2(img,'bgr8')
		except CvBridgeError as e:
			print(e)
		self.get_raw_poses(frame)

	def get_raw_poses(self,frame):
		corners,ids,rejected = cv2.aruco.detectMarkers(frame,self.arucoDict) #Aruco basis function. 
																			#Get detected marks corners
																			#And possible marks corners
		if ids is None:
			return
		stamp = rospy.Time.now()
		#Classify them as reference markers and mobile markers.
		ref_marks = {}
		mob_marks = {}
		for i in range(len(ids)):
			id_dict = str(ids[i,0])
			#Get marker to camera transformation in (rotational vector, translation vector)
			rvec,tvec = cv2.aruco.estimatePoseSingleMarkers([corners[i]],
						self.markerLen,self.cam_model.mat,self.cam_model.dist)
			if id_dict in self.refids:
				marker = Marker(id_dict,self.cam_id,stamp,corners) 
				marker.broadcast_tf(rvec,tvec)
				ref_marks[id_dict] = marker
				continue #To next detected marker.
			else:
				marker = MobileMarker(id_dict,self.cam_id,stamp,corners) 
				marker.broadcast_tf(rvec,tvec)
				mob_marks[id_dict] = marker
			#Pass if its a mobile marker.
		for mm in mob_marks.values():
			#Find nearest reference marker
			min_dist = 1E9
			for rm in ref_marks.values():
				dist = np.sum(np.linalg.norm(np.asarray(mm.corners)-np.asarray(rm.corners)))
				if dist < min_dist: 
					min_dist = dist
					near_rm = rm
			if min_dist == 1E9:
				return #No reference markers found, absolute pose cannob be calculated.
			#Publish position.
			try:
				mm.get_relative_pose(near_rm.mid,self.tfBuffer.lookup_transform)
				mm.publish()
			except:
				print("Couldn't compute absolute pose of mobile marker"+str(mm.mid))
def main(args):
	rospy.init_node('fakegps',anonymous=True)
	cam_src = rospy.get_param("~cam_src", "internal") #Unique for each vehicle
	upper_cam_id = rospy.get_param("~upcam_id",0)
	ref_marks = marks_offset.keys()
	node = fake_gps(0,cam_id=upper_cam_id,camera_src=cam_src,refids=ref_marks) 
	if not cam_src=='internal':	
		try:
			rospy.spin()
		except KeyboardInterrupt:
			print("Shutting down")
	else:
		try: 
			node.talker()
		except rospy.ROSInterruptException:
			pass

if __name__=='__main__':
	main(sys.argv)