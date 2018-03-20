#!/usr/bin/env python
from plutodrone.srv import *
from plutodrone.msg import *
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseArray, Pose
import rospy
import datetime

#/*working variables*/
hower_point=(0, 0, 15)

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
		self.data = rospy.Service('PlutoService', PlutoPilot, self.access_data)
		rospy.Subscriber('whycon/poses', PoseArray, self.indentify_key)
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
		self.cmd = PlutoMsg()
		self.cmd.rcRoll =1500
		self.cmd.rcPitch =1500
		self.cmd.rcYaw =1500
		self.cmd.rcThrottle =1500
		self.cmd.rcAUX1 =1500
		self.cmd.rcAUX2 =1500
		self.cmd.rcAUX3 =1500
		self.cmd.rcAUX4 =1000
		self.roll=0
		self.pitch=1
		self.high=0
		self.yaw=160
		self.x=0
		self.y=0
		self.time_now=datetime.datetime.now()
		self.roll_pid = PID(15, 0, 0, self.time_now)
		self.pitch_pid = PID(15, 0, 0, self.time_now)
		self.yaw_pid = PID(40, 0, 0, self.time_now)
		self.throttle_pid = PID(-100, 0, 0, self.time_now)
		self.flag=0
		self.output = Pose()
		self.set_p= Pose()
		self.drone_p= Pose()

	def arm(self):
		self.cmd.rcRoll=1500
		self.cmd.rcYaw=1500
		self.cmd.rcPitch =1500
		self.cmd.rcThrottle =1000
		self.cmd.rcAUX4 =1500
		print("arm")
		self.command_pub.publish(self.cmd)
		rospy.sleep(.1)

	def disarm(self):
		self.cmd.rcThrottle =1300
		self.cmd.rcAUX4 = 1200
		self.command_pub.publish(self.cmd)
		print("disarm")
		rospy.sleep(0.1)

	def indentify_key(self, msg):
		pos_a = msg.poses[0].position
		pos_b = msg.poses[1].position
		if(pos_a.z>20):
			self.set_p = msg.poses[0].position
			self.drone_p = msg.poses[1].position
		else:
			self.set_p= msg.poses[1].position
			self.drone_p= msg.poses[0].position
		self.set_p.z=15
		self.output.position.y = self.set_p.x - self.drone_p.x
		self.output.position.x = self.set_p.y - self.drone_p.y
		self.output.position.z = self.set_p.z - self.drone_p.z
		self.out = self.output.position
		self.high=self.drone_p.z
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
		print(self.out.x, self.out.y, self.out.z)
			
	def calc_x(self, x):
		if (x <= (-5)): self.roll =self.x+2
		elif (x <= (-4)): self.roll =self.x+2
		elif (x <= (-3)): self.roll =self.x+1
		elif (x <= (-1)): self.roll =self.x+1
		elif (x >= (5)): self.roll =self.x-2
		elif (x >= (4)): self.roll =self.x-2
		elif (x >= (3)): self.roll =self.x-1
		elif (x >= (1)): self.roll =self.x-1
		else: self.roll =self.x

	def calc_y(self, y):
		if (y <= (-5)): self.pitch =self.y+2
		elif (y <= (-4)): self.pitch =self.y+2
		elif (y <= (-3)): self.pitch =self.y+1
		elif (y <= (-1)): self.pitch =self.y+1
		elif (y >= (5)): self.pitch =self.y-2
		elif (y >= (4)): self.pitch =self.y-2
		elif (y >= (3)): self.pitch =self.y-1
		elif (y >= (1)): self.pitch =self.y-1
		else: self.pitch =self.y

	def access_data(self, req):
		print "roll = " + str(req.roll), "pitch = " + str(req.pitch), "yaw = " + str(req.yaw)
		print "altitude = " +str(req.alt)
		if(self.flag==0):
			self.disarm()
			rospy.sleep(1)
			self.command_pub.publish(self.cmd)
			self.flag=1
		if(self.flag==1):
			self.arm()
			rospy.sleep(2)
			self.command_pub.publish(self.cmd)
			self.flag=2
		if(self.flag==2):
			self.command_pub.publish(self.cmd)
			self.time_now=datetime.datetime.now()
			[p_outr, i_outr, d_outr] = self.roll_pid.Compute(req.roll, self.roll, self.time_now)
			[p_outp, i_outp, d_outp] = self.pitch_pid.Compute(req.pitch, self.pitch, self.time_now)
			[p_outt, i_outt, d_outt] = self.throttle_pid.Compute(self.high, self.set_p[2] , self.time_now)
			[p_outy, i_outy, d_outy] = self.yaw_pid.Compute(req.yaw, 145 , self.time_now)
			roll_out = p_outr + i_outr + d_outr
			pitch_out = p_outp + i_outp + d_outp
			throttle_out = p_outt + i_outt + d_outt
			yaw_out = p_outy + i_outy + d_outy
			self.update(roll_out, pitch_out, throttle_out, yaw_out)
		rospy.sleep(0.1)
		return PlutoPilotResponse(rcAUX2 =2000)
		
	def update(self, roll, pitch, throttle, yaw):
		self.cmd.rcRoll= int(1500 + roll)
		self.cmd.rcPitch= int(1500 + pitch)
		self.cmd.rcThrottle= int(1500+throttle)
		self.cmd.rcYaw= int(1500 + yaw)
		if self.cmd.rcRoll < 1450:
			self.cmd.rcRoll = 1450
		if self.cmd.rcRoll > 1550:
			self.cmd.rcRoll = 1550
		if self.cmd.rcPitch < 1450:
			self.cmd.rcPitch = 1450
		if self.cmd.rcPitch > 1550:
			self.cmd.rcPitch = 1550
		if self.cmd.rcThrottle < 1500:
			self.cmd.rcThrottle = 1500
		if self.cmd.rcThrottle > 2000:
			self.cmd.rcThrottle = 2000
		if self.cmd.rcYaw < 1490:
			self.cmd.rcYaw = 1490
		if self.cmd.rcYaw > 1510:
			self.cmd.rcYaw = 1510
		self.cmd.rcAUX2 =2000
		self.command_pub.publish(self.cmd)
		print(self.cmd.rcRoll,self.cmd.rcPitch,self.cmd.rcYaw,self.cmd.rcThrottle,self.cmd.rcAUX2)

	def control_drone(self):
		while True:
			if self.flag == 0:
				self.disarm()
			if self.flag == 1:
				self.arm()
			self.command_pub.publish(self.cmd)

if __name__ == '__main__':
	while not rospy.is_shutdown():
		test = request_data()
		test.control_drone()
		rospy.spin()
		sys.exit(1)
