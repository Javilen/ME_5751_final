		# # Most of your program should be here, compute rho, alpha and beta using d_pos and c_pos
		rho=math.sqrt((d_posX-c_posX)**2+(d_posY-c_posY)**2)
		c_v, c_w = dwa.planning(poise, velocity, goal, point_cloud, self.config)
		# c_v=4*c_v
		# c_w=4*c_w

		# Overall speed of the robot
		s = sqrt(c_vix^2 + c_viy^2)
		phi = pi/2 - c_theta
		
		# Ackermann Forward Kinematics 
		x_dot = s*math.cos(c_theta)
		y_dot = s*math.cos(c_theta)
		theta_dot = (s/l)*math.tan(phi)

		c_v = sqrt(x_dot^2 + y_dot^2)
		c_w = theta_dot

		# self.robot.set_motor_control(linear velocity (cm), angular velocity (rad))
		self.robot.set_motor_control(c_v, c_w)  # use this command to set robot's speed in local frame

		# Outer wheels dynamics
		phi_l = math.atan(2*l*math.sin(phi)/(2*l*math.cos(phi) - w*math.sin(phi)))
		phi_r = math.atan(2*l*math.sin(phi)/(2*l*math.cos(phi) + w*math.sin(phi)))
