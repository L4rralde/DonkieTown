import numpy as np 
import rospy
from tf.transformations import quaternion_from_euler,quaternion_matrix
import tf2_ros 
import geometry_msgs.msg

def quaternion_from_vector(rvec):
	rvec = np.reshape(rvec,(3))
	rvec = np.asarray(rvec,dtype=float)
	if rvec.ndim not in [1, 2] or rvec.shape[len(rvec.shape)-1] != 3:
            raise ValueError("Expected `rot_vec` to have shape (3,) "
                             "or (N, 3), got {}".format(rvec.shape))
 	angle = np.linalg.norm(rvec)
 	if angle <= 1e-3: 
 		angle2 = angle*angle 
 		scale = 0.5-angle2/48+angle2*angle2/3840
 	else: 
 		scale = np.sin(angle/2)/angle
	quat = np.zeros(4)
 	quat[0] = scale*rvec[0]
 	quat[1] = scale*rvec[1]
 	quat[2] = scale*rvec[2]
 	quat[3] = np.cos(angle/2)
 	return quat

def axis_matrix(rvec,tvec):
	q = quaternion_from_vector(rvec)
	matrix = quaternion_matrix(q)
	tvec = np.reshape(tvec,(3))
	matrix[0,3] = tvec[0]
	matrix[1,3] = tvec[1]
	matrix[2,3] = tvec[2]
	return matrix
	
class FrameWork(object): 
	def __init__(self,father,child): 
		self.t = geometry_msgs.msg.TransformStamped()
		self.t.header.frame_id = father
		self.t.child_frame_id = child
	def set_translation(self,tvec):
		tv = np.reshape(tvec,(3))
		self.t.transform.translation.x = tv[0]
		self.t.transform.translation.y = tv[1]
		self.t.transform.translation.z = tv[2]
	def set_orientation(self,q): 
		q = np.reshape(q,(4))
		self.t.transform.rotation.x = q[0]
		self.t.transform.rotation.y = q[1]
		self.t.transform.rotation.z = q[2]	
		self.t.transform.rotation.w = q[3]
	def update_from_rotvec(self,rvec,tvec): 
		self.t.header.stamp = rospy.Time.now()
		self.set_translation(tvec)
		q = quaternion_from_vector(rvec)
		self.t.transform.rotation.x = q[0]
		self.t.transform.rotation.y = q[1]
		self.t.transform.rotation.z = q[2]	
		self.t.transform.rotation.w = q[3]
	def update_from_quaternion(self,quat,tvec):
		self.t.header.stamp = rospy.Time.now()
		self.set_translation(tvec)
		q = np.reshape(quat,(4))
		self.t.transform.rotation.x = q[0]
		self.t.transform.rotation.y = q[1]
		self.t.transform.rotation.z = q[2]	
		self.t.transform.rotation.w = q[3]


class FrameBr(FrameWork): 
	def __init__(self,father,child):
		super(FrameBr,self).__init__(father,child)
		self.br = tf2_ros.TransformBroadcaster()
	def broadcast(self):
		self.br.sendTransform(self.t)

def broadcast_static_tf(s_m_ids): 
	time = rospy.Time.now()
	br = tf2_ros.StaticTransformBroadcaster()
	s_trans = [] 
	for mid in s_m_ids: 
		fr = FrameWork("map","mark_offset"+mid)
		fr.t.header.stamp = time
		mark_offset = marks_offset[mid]
		xo = mark_offset[0]
		yo = mark_offset[1]
		tho = mark_offset[2]
		q = quaternion_from_euler(0,0,tho)
		fr.set_translation([xo,yo,0.0])
		fr.set_orientation(q)
		s_trans.append(fr.t)
	br.sendTransform(s_trans)





	