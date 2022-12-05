#!/usr/bin/python
# -*- coding: utf-8 -*-

#from calendar import c
#from sys import float_repr_style
from E160_state import *
from E160_robot import *
import math
import dwa
from cost_map import *


class P_controller:

	def __init__(self, robot, logging = False):
		self.robot = robot  # do not delete this line
		self.kp = 5  # k_rho
		self.ka = 12  # k_alpha
		self.kb = -1.5  # k_beta
		self.finish = 10  # finish distance requirement
		self.fltrC_w = 50
		self.fltrC_v = 100
		self.logging = logging
		self.robot_lengh = 20
		self.robot_width = 16

		# __init__ for DWA approach
		self.base = [-3., -2.5, +3., +2.5]#[-3.0, -2.5, +3.0, +2.5]
		self.config = dwa.Config(
                max_speed = 25.0,  # 3.0 # 16.0 30
                min_speed = -7.0, # -1.0  # -5.0 -7.0
                max_yawrate = np.radians(310.0),    # np.radians(40.0), # np.radians(310.0), np.radians(210.0),
                max_accel = 15.0, #15
                max_dyawrate = np.radians(410.0),  # np.radians(110.0)  # np.radians(410.0), np.radians(310.0),
                velocity_resolution = 0.1,
                yawrate_resolution = np.radians(1.0),
                dt = 0.1,
                predict_time = 0.3, # 3.0  # is this the key?
                heading = 0.15,   # 0.15  # 5.15  # Weights
                clearance = 0.1,  # 1.0
                velocity = 2.0,  # 1.0  # 3.0
                base = self.base)


		if(logging == True):
			#self.robot.make_headers(['pos_X','posY','posZ','vix','viy','wi','vr','wr'])
			#self.robot.make_headers(['pos_X','rho', 'd_theta', 'alpha', 'beta'])
			self.robot.make_headers(['pos_X','pos_y','vix','viy','wi','c_theta','goal_X','goal_y','d_theta'])
		self.set_goal_points()

	#Edit goal point list below, if you click a point using mouse, the points programmed
	#will be washed out
	def set_goal_points(self):
		do_nothing=None
		# here is the example of destination code
		
		# self.robot.state_des.add_destination(x=120,y=130,theta=-0.43)  #goal point 1
		# self.robot.state_des.add_destination(x=190,y=-0,theta=2.1)     #goal point 2
		# self.robot.state_des.add_destination(x=-150,y=-175,theta=0)    #goal point 3
		# self.robot.state_des.add_destination(x=-150,y=-25,theta=-1.57) #goal point 4
		# self.robot.state_des.add_destination(x=-25,y=200,theta=2.5)    #goal point 5


	def track_point(self):

		# All d_ means destination

		(d_posX, d_posY, d_theta) = self.robot.state_des.get_des_state()  # get next destination configuration

		# All c_ means current_

		(c_posX, c_posY, c_theta) = self.robot.state.get_pos_state()  # get current position configuration
		(c_vix, c_viy, c_wi) = self.robot.state.get_global_vel_state() #get current velocity configuration, in the global frame
		(c_v, c_w) = self.robot.state.get_local_vel_state() #get current local velocity configuration

		#Dynamic Window approach
		poise=(c_posX,c_posY,c_theta)
		velocity=(c_v,c_w)
		goal=(d_posX,d_posY)
		point_cloud=np.array([[3000,3000],[3000,3000],[3000,3000],[3000,3000],[3000,3000],[3000,3000]], dtype=np.float32)
		#print(self.costmap.cost_map.get_cloud())

		# c_v, c_w = dwa.planning(poise,velocity,goal,point_cloud,self.config)
		# c_v=4*c_v
		# c_w=4*c_w

		# Overall speed of the robot
		s = math.sqrt(c_vix**2 + c_viy**2)
		phi = math.pi/2 - c_theta
		
		# Ackermann Forward Kinematics 
		x_dot = s*math.cos(c_theta)
		y_dot = s*math.cos(c_theta)
		theta_dot = (s/self.robot_lengh)*math.tan(phi)  # theta_dot = (s/l)*math.tan(phi)

		c_v = math.sqrt(x_dot**2 + y_dot**2)
		c_w = theta_dot

		# self.robot.set_motor_control(linear velocity (cm), angular velocity (rad))
		# self.robot.set_motor_control(c_v, c_w)  # use this command to set robot's speed in local frame

		# Outer wheels dynamics
		phi_l = math.atan(2*self.robot_lengh*math.sin(phi)/(2*self.robot_lengh*math.cos(phi) - self.robot_width*math.sin(phi)))  # phi_l = math.atan(2*l*math.sin(phi)/(2*l*math.cos(phi) - w*math.sin(phi)))
		phi_r = math.atan(2*self.robot_lengh*math.sin(phi)/(2*self.robot_lengh*math.cos(phi) + self.robot_width*math.sin(phi)))  # phi_r = math.atan(2*l*math.sin(phi)/(2*l*math.cos(phi) + w*math.sin(phi)))


		c_v, c_w = dwa.planning(poise,velocity,goal,point_cloud,self.config)


		rho=math.sqrt((d_posX-c_posX)**2+(d_posY-c_posY)**2)

		
		# self.robot.set_motor_control(linear velocity (cm), angular velocity (rad))
		self.robot.set_motor_control(c_v, c_w)  # use this command to set robot's speed in local frame
		

		self.robot.send_wheel_speed(float("{:06.1f}".format(phi_l)),float("{:06.1f}".format(phi_r)))

		# self.robot.send_wheel_speed(float("{:06.1f}".format(phi_l)),float("{:06.1f}".format(phi_r))) #unit rad/s phi_l = 6.0,phi_r = 6.0
		# self.robot.send_wheel_speed(float("{:.1f}".format(phi_l)),float("{:.1f}".format(phi_r))) #unit rad/s phi_l = 6.0,phi_r = 6.0


		# use the following to log the variables, use [] to bracket all variables you want to store
		# stored values are in log folder
		if self.logging == True:
			#self.robot.log_data([c_posX,c_posY,c_theta,c_vix,c_viy,c_wi,c_v,c_w])
			#self.robot.log_data([ c_posX , rho , d_theta, alpha, beta ])
			self.robot.log_data([c_posX,c_posY,c_vix,c_viy,c_wi,c_theta,d_posX,d_posY,d_theta ])

		if abs(rho) < self.finish:# and abs(d_theta - c_theta) < 8: #you need to modify the reach way point criteria  if abs(c_posX - d_posX) < 80:
			if(self.robot.state_des.reach_destination()): 
				print("final goal reached")
				self.robot.set_motor_control(.0, .0)  # stop the motor
				self.robot.send_wheel_speed(.0, .0)
				return True
			else:
				print("one goal point reached, continute to next goal point")
		
		return False
