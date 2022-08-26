#!/usr/bin/env python2
#ego_pose node = ego_mark_tf+ego_pose_raw nodes 
#Remember to mirgate to docker

import rospy 
import numpy as np  
import cv2
from donkietown_msgs.msg import MarkerEdge, MarkerEdgeArray, Square
from geometry_msgs.msg import PoseWithCovarianceStamped as PCS
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf2_ros 
from scipy.optimize import linear_sum_assignment
from geometry_figure import order_corners
import filterpy.common as kalman
from framework import FrameBr as fw
from node_cfg import marks_offset
from camera_misc import CameraModel

class MarkCornersEstimator:
	def __init__(self,egoid, ref_ids): 
		self.egoid = egoid
		self.ref_ids = ref_ids
		#Define a particle-knimeatic-based Kalman filter per square corner. 
		self.mark_corners = np.zeros((4,2))
		self.mark_corners_estimation = np.zeros((4,2))
		self.corners_kf = [kalman.kinematic_kf(2,1) for i in range(4)] 
		self.filters_init()
		self.ltime = rospy.get_rostime()
	def match_mark_corners(self,msgdata):
		#compute de
		time = msgdata.header.stamp
		dt = time.to_sec()-self.ltime.to_sec()
		self.corner_estimate(dt) #Estimate ego mark corners coordinates. 
		self.ltime = time
		markers = msgdata.MarkerEdges
		foundMark = False 
		ref_marks = []
		for m in markers: #Check whether ego mark was previously detected. 
			if (m.id == self.egoid) and (not foundMark): 
				foundMark = True
				this_mark = m
			elif m.id in self.ref_ids: #While building an array of detected reference marks
				ref_marks.append(m)
		if foundMark: #If mark was previously detected: 
			self.mark_corners = np.reshape(this_mark.corners,(4,2)).astype(int) 
			self.update_kf() #Update kalman filter with ego mark data. 
			return (self.mark_corners, ref_marks) #continue to compute pose. 
		if msgdata.rejected is not None: 
			min_norm = 32 #Distance trheshold to look at local region. 
			befstfit_corner = np.zeros((4,2))
			mark_corners = order_corners(self.mark_corners_estimation)
			for r in msgdata.rejected: #For every corner of possible marks. 
				square = np.reshape(r.corners,(4,2))
				square = order_corners(square)
				#Change to mark estimation
				norm = np.linalg.norm(square-mark_corners) #Compute sum of distances from estimated corners to detected corner.
				if norm < min_norm: #Save the nearest and discard the previous nearest. 
					min_norm = norm
					bestfit_corner = square
				else: 
					continue
			if min_norm<32: #If found match from distance-threshold-defined region: 
				costs = [np.linalg.norm(bestfit_corner-p, axis=1) for p in self.mark_corners] #Order corners to its fiducial mark form. 
																							#Hungarian algorithm with distances per corner 
																							#as cost is used. 
				row_ind, col_ind = linear_sum_assignment(np.reshape(costs, (4,4)))
				bestfit_corner = np.array([bestfit_corner[i] for i in row_ind], dtype="int32")
				self.mark_corners = np.reshape(bestfit_corner,(4,2)).astype(int)  #Take selection as ego mark 
				self.update_kf() #Update Kalman filter with ego mark
				return (self.mark_corners, ref_marks) #Continue to compute pose. 
		return (None, ref_marks)
	def corner_estimate(self,dt=1):
		for kf in self.corners_kf: 
			kf.F = np.array([[1,dt,0,0],
							 [0,1,0,0],
							 [0,0,1,dt],
							 [0,0,0,1]])
			kf.predict()
		self.mark_corners_estimation = self.kf2markCorners()
	def filters_init(self,P0=100,R0=1,Q0=10):
		for kf in self.corners_kf:
			kf.P *= P0
			kf.R *= R0
			kf.Q *= Q0
	def kf2markCorners(self): 
		corners = []
		for i in range(4): 
			kfx = self.corners_kf[i].x
			corners.append([kfx[0,0],kfx[2,0]])
		return np.asarray(corners)
	def update_kf(self):
		for i in range(4): 
			z = self.mark_corners[i]
			self.corners_kf[i].update(z)
		self.mark_corners_estimation = self.kf2markCorners() 

class EgoPose: 
	def __init__(self,robot_id,f_frame="map"): 
		self.tfBuffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tfBuffer)
		self.c_frame = "rel_car_link_"+str(robot_id)
		self.posePub = rospy.Publisher("/fake_gps/ego_pose_raw/"+str(robot_id),PCS,queue_size=1)
		self.pose_msg = PCS()
		self.pose_msg.header.frame_id = f_frame
		self.pose_msg.pose.covariance = [0.01,0,0,0,0,0,
										 0,0.01,0,0,0,0,
										 0,0,0,0,0,0, 
										 0,0,0,1e-3,0,0,
										 0,0,0,0,1e-3,0,
										 0,0,0,0,0,0.03]
		self.pose_msg.pose.pose.position.z = 0.0
	def publish(self,ref_mark):
		#Computes relative-to-given-reference-mark ego mark pose. 
		try:
			trans = self.tfBuffer.lookup_transform("ref_mark_"+str(ref_mark),self.c_frame,rospy.Time())
		except: 
			print("Error: Could not compute relative pose from ref marker to self mobile marker")
			return
		#trans = self.tfBuffer.lookup_transform(self.c_frame,"ref_mark_"+str(ref_mark),rospy.Time())
		(xo,yo,tho) = marks_offset[str(ref_mark)] #Add reference mark pose offset. 
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
		try:
			self.posePub.publish(self.pose_msg) #Publish ego mark pose. 
		except:
			print("Error: Could not publish self pose message")
			pass

class ego_pose_node:
	def __init__(self,egoid,ref_ids=[0],markerLen=0.1,arucoDict=cv2.aruco.DICT_4X4_50,paramfile='calibration.yaml'): 
		#Aruco info: 
		self.egoid = egoid
		self.markerLen = markerLen
		self.arucoDict = cv2.aruco.Dictionary_get(arucoDict)
		#Camera parameters: 
		self.cam_model = CameraModel(kind="subscriber",topic='/sensors/global_camera/info')
		#Frames transformations: 
		self.ref_ids = ref_ids
		#self.physFrame = fw("base_link_cam","base_link_mark_"+str(egoid))
		self.physFrame = fw("map","rel_car_link_"+str(egoid))
		#Node conections: 
		#self.ego_pose_pub = rospy.Publisher('/fake_gps/ego_mark_tf/'+str(egoid),PCS,queue_size=1)
		self.global_gps_sub = rospy.Subscriber('/fake_gps/marks_corners',MarkerEdgeArray, self.on_globalgps)
		#Kalman filter for mark corners matching: 
		self.mark_finder = MarkCornersEstimator(egoid,ref_ids) 
		#Ego pose 
		self.pose = EgoPose(egoid)
	def on_globalgps(self, msgdata):
		#time = msgdata.header.stamp
		#Get ego mark corners: 
		(mark_corners,ref_marks) = self.mark_finder.match_mark_corners(msgdata)
		if mark_corners is None: 
			return 
		#Check nearest ref mark:  
		min_dist = 1E4
		near_mark_id = self.ref_ids[0]
		if ref_marks is not None: 
			for m in ref_marks:
				ref_corners = np.reshape(m.corners,(4,2)).astype(int)
				dist = np.sum(np.linalg.norm(mark_corners-ref_corners))
				if dist < min_dist: 
					min_dist = dist
					near_mark_id = m.id 
		#get and broadcast ego mark pose 
		corner = np.reshape(mark_corners,(1,4,2)).astype("float32")
		try:
			rvec,tvec,_ = cv2.aruco.estimatePoseSingleMarkers([corner],
									self.markerLen,self.cam_model.mat,self.cam_model.dist)
		except: 
			print("Error: Could not estimate self marker pose")
			return	
		self.physFrame.update_from_rotvec(rvec[0,0],tvec[0,0])
		self.physFrame.broadcast() #Broadcast relative ego mark pose. 
		self.pose.publish(near_mark_id) #Compute and broadcast absolut ego mark pose via
										#relative-to-nearest-reference-mark pose.

def main():
	rospy.init_node("ego_pose", anonymous=True)
	robot_id = rospy.get_param("~car_id") #Unico para cada robot
	ref_ids = [int(k) for k in marks_offset.keys()]
	node =  ego_pose_node(robot_id,ref_ids=ref_ids) 
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
if __name__=='__main__':
	main()
