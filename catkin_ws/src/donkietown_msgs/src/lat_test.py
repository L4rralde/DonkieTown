#!/usr/bin/env python2
import rospy
import sys
from std_msgs.msg import Header

class NetworkAgent(object):
	def __init__(self,source,target):
		self.publisher = rospy.Publisher(target,Header,queue_size=1)
		self.subscriber = rospy.Subscriber(source,Header,self.callback)		
	def create_msg(self,trx_id=0):
		msg = Header()
		msg.seq = trx_id
		msg.stamp = rospy.Time.now()
		return msg
	def callback(self,msg):
		pass

class Repeater(NetworkAgent):
	def __init__(self,source,target):
		super(Repeater,self).__init__(source,target)
		self.pub_msg = self.create_msg()
	def callback(self,rx_msg):
		self.update_pubmsg(rx_msg.seq)
		print(self.pub_msg.stamp.to_sec()-rx_msg.stamp.to_sec())
		self.publisher.publish(self.pub_msg)
	def update_pubmsg(self,trx_id):
		now = rospy.Time.now()
		self.pub_msg.seq = trx_id
		self.pub_msg.stamp = now

class TalkerNListener(NetworkAgent):
	def __init__(self,source,target):
		super(TalkerNListener,self).__init__(source,target)
		self.outstanding_msgs = {}
		self.time_sum = 0.0
		self.rx_cnt = 0
	def callback(self,rep_msg):
		now = rospy.Time.now()
		if(not rep_msg.seq in self.outstanding_msgs.keys()):
			return
		src_msg = self.outstanding_msgs[rep_msg.seq]
		print(rep_msg.stamp.to_sec()-src_msg.stamp.to_sec())
		delay = now.to_sec()-src_msg.stamp.to_sec() 
		print(delay)
		print("------------------------")
		self.time_sum = self.time_sum+delay
		self.rx_cnt = self.rx_cnt+1
		del self.outstanding_msgs[rep_msg.seq]
	def talker(self,rate):
		count = 0
		rate = rospy.Rate(rate)
		while not rospy.is_shutdown():
			pub_msg = self.create_msg(count)
			self.outstanding_msgs[count] = pub_msg
			self.publisher.publish(pub_msg)
			rate.sleep()
			count = (count+1)%100
		print("Average:")
		print(self.time_sum/self.rx_cnt)

def main(args):
	rospy.init_node('latency_test',anonymous=True)
	kind = rospy.get_param("~kind","Sourcer")
	comp_id = rospy.get_param("~computer_id","ground_station")
	node_bn = "/latency_test/"+str(comp_id)
	if(kind=="repeater"):
		node = Repeater(node_bn+"/source",node_bn+"/repeated")
		try:
			rospy.spin()
		except KeyboardInterrupt:
			print("Shutting down")
	else:
		node = TalkerNListener(node_bn+"/repeated",node_bn+"/source")
		try:
			node.talker(50)
		except rospy.ROSInterruptException:
			pass

if __name__=='__main__':
	main(sys.argv)






