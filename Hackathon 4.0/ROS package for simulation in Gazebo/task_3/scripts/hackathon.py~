#!/usr/bin/env python

import rospy
import time
import datetime
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseArray, Pose, Twist

#/*working variables*/

class PID:

	def __init__(self, p_gain, i_gain, d_gain, now):
		self.last_error = 0.0
		self.last_time = now
		self.p_gain = p_gain
		self.i_gain = i_gain
		self.d_gain = d_gain
		self.i_error = 0.0

	def Compute(self, input_, target, now):
		dt = (now - self.last_time)*100000
		secs=dt.seconds #timedelta has everything below the day level stored in seconds
		if(secs==0): secs+=1
		error = target - input_
		p_error = error
		self.i_error += (error + self.last_error) * (secs)
		i_error = self.i_error
		d_error = (error - self.last_error) / (secs)
		p_output = self.p_gain * p_error
		i_output = self.i_gain * i_error
		d_output = self.d_gain * d_error
		self.last_error = error
		self.last_time = now
		return p_output, i_output, d_output


class request_data():
	"""docstring for request_data"""
	def __init__(self):
		rospy.init_node('PID_flight')
		rospy.Subscriber('whycon/poses', PoseArray, self.indentify_key)
		self.command_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=20)
		self.final = Twist()
		self.output = Pose()
		self.set_p = Pose()
		self.drone_p = Pose()
		self.error= Twist()

		self.error.linear.x = 0
		self.error.linear.y = 0
		self.error.linear.z = 0
		self.error.angular.x = 0
		self.error.angular.y = 0
		self.error.angular.z = 0
		self.roll = self.error.linear.x
		self.pitch = self.error.linear.y
		self.high = self.error.linear.z
		self.x=0
		self.y=0
		self.time_now=datetime.datetime.now()
		self.roll_pid = PID(1, 0, 0, self.time_now)
		self.pitch_pid = PID(1, 0, 0, self.time_now)
		#self.yaw_pid = PID(40, 0, 0, self.time_now)
		#self.throttle_pid = PID(-100, 0, 0, self.time_now)
		self.flag=0
		

	def indentify_key(self, msg):
		self.flag=1
		pos_a = msg.poses[0].position
		pos_b = msg.poses[1].position
		if(pos_a.z>12):
			self.set_p = msg.poses[0].position
			self.drone_p = msg.poses[1].position
		else:
			self.set_p= msg.poses[1].position
			self.drone_p= msg.poses[0].position

		self.output.position.y = self.set_p.x - self.drone_p.x
		self.output.position.x = self.set_p.y - self.drone_p.y
		self.output.position.z = self.set_p.z - self.drone_p.z
		self.out = self.output.position
		print(self.out)#self.high=drone_p.z
		"""self.roll =self.x+5
		self.pitch =self.y+0
		rospy.sleep(5)
		self.roll =self.x-5
		self.pitch =self.y+0
		rospy.sleep(5)
		self.roll =self.x+0
		self.pitch =self.y+5
		rospy.sleep(5)
		self.roll =self.x+0
		self.pitch =self.y-5
		rospy.sleep(5)"""
		self.calc_x(self.out.x)
		self.calc_y(self.out.y)
		
		if(self.flag==0):
			self.hold()
			self.command_pub.publish(self.final)
			if(self.output.position.z<11.5):self.flag=1
		if(self.flag==1):
			#self.command_pub.publish(self.cmd)
			self.time_now=datetime.datetime.now()
			[p_outr, i_outr, d_outr] = self.roll_pid.Compute(self.set_p.x, self.roll, self.time_now)
			[p_outp, i_outp, d_outp] = self.pitch_pid.Compute(self.set_p.y, self.pitch, self.time_now)
			#[p_outt, i_outt, d_outt] = self.throttle_pid.Compute(self.set_p.y, self.set_p.y , self.time_now)
			#[p_outy, i_outy, d_outy] = self.yaw_pid.Compute(self.set_p.y,, 145 , self.time_now)
			roll_out = p_outr + i_outr + d_outr
			pitch_out = p_outp + i_outp + d_outp
			#throttle_out = p_outt + i_outt + d_outt
			#yaw_out = p_outy + i_outy + d_outy
			self.update(self.roll, self.pitch)#, throttle_out, yaw_out)
			
	def hold(self):
		self.final.linear.x = 0
		self.final.linear.y = 0
		self.final.linear.z = 0
		self.final.angular.x = 0
		self.final.angular.y = 0
		self.final.angular.z = 0
	
	def calc_x(self, x):
		if (x <= (-5)): self.roll =self.x+1
		elif (x <= (-4)): self.roll =self.x+1
		elif (x <= (-3)): self.roll =self.x+1
		elif (x <= (-1)): self.roll =self.x+1
		elif (x >= (5)): self.roll =self.x-1
		elif (x >= (4)): self.roll =self.x-1
		elif (x >= (3)): self.roll =self.x-1
		elif (x >= (1)): self.roll =self.x-1
		else: self.roll =self.x

	def calc_y(self, y):
		if (y <= (-5)): self.pitch =self.y+1
		elif (y <= (-4)): self.pitch =self.y+1
		elif (y <= (-3)): self.pitch =self.y+1
		elif (y <= (-1)): self.pitch =self.y+1
		elif (y >= (5)): self.pitch =self.y-1
		elif (y >= (4)): self.pitch =self.y-1
		elif (y >= (3)): self.pitch =self.y-1
		elif (y >= (1)): self.pitch =self.y-1
		else: self.pitch =self.y
		
	def update(self, roll, pitch):#, throttle, yaw):
		self.final.linear.x= int(roll)
		self.final.linear.y= int(pitch)
		#self.final.linear.z= int(throttle)
		"""if self.final.linear.x < -1:
			self.final.linear.x = -1
		if self.final.linear.x > 1:
			self.final.linear.x = 1
		if self.final.linear.x > -1 and self.final.linear.x < 1:
			self.final.linear.x = 0
		if self.final.linear.y < -1:
			self.final.linear.y = -1
		if self.final.linear.y > 1:
			self.final.linear.y = 1
		if self.final.linear.y > -1 and self.final.linear.y < 1:
			self.final.linear.y = 0"""
		"""if self.cmd.rcThrottle < 1500:
			self.cmd.rcThrottle = 1500
		if self.cmd.rcThrottle > 2000:
			self.cmd.rcThrottle = 2000
		if self.cmd.rcYaw < 1490:
			self.cmd.rcYaw = 1490
		if self.cmd.rcYaw > 1510:
			self.cmd.rcYaw = 1510
		self.cmd.rcAUX2 =2000"""
		self.command_pub.publish(self.final)
		print(self.final.linear.x,self.final.linear.y)#,self.cmd.rcYaw,self.cmd.rcThrottle,self.cmd.rcAUX2)

	def control_drone(self):
		while True:
			if self.flag == 0:
				self.hold()
			if self.flag == 1:
				self.command_pub.publish(self.final)
				self.flag =0

if __name__ == '__main__':
	while not rospy.is_shutdown():
		test = request_data()
		test.control_drone()
		rospy.spin()
		sys.exit(1)
