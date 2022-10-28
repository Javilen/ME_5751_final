from cmath import cos
import cv2
import numpy as np
import math
import warnings
from PIL import Image, ImageTk
# Added some libraries
import queue
import itertools

from bsTree import *
from Path import *
from cost_map import cost_map
# from Queue import Queue


class path_planner:
	def __init__(self,graphics):
		self.graphics = graphics
		# self.graphics.scale = 400 #half pixel number on canvas, the map should be 800 x 800
		# self.graphics.environment.robots[0].set_bot_size(body_cm = 2*self.inflation_radius)
		#self.graphics.environment.width/height = 2

		self.costmap = self.graphics.map
		self.map_width = self.costmap.map_width
		self.map_height = self.costmap.map_height

		self._init_path_img()
		self.path = Path()

	
		self.set_start(world_x = 0, world_y = 0)
		self.set_goal(world_x = 10.0, world_y = 20.0, world_theta = .0)  #self.set_goal(world_x = 100.0, world_y = 200.0, world_theta = .0)

		self.plan_path()
		self._show_path()


	def set_start(self, world_x = 0, world_y = 0, world_theta = 0):
		self.start_state_map = Pose()
		map_i, map_j = self.world2map(world_x,world_y)
		print("Start with %d, %d on map"%(map_i,map_j))
		self.start_state_map.set_pose(map_i,map_j,world_theta)


	def set_goal(self, world_x, world_y, world_theta = 0):
		self.goal_state_map = Pose()
		map_i, map_j = self.world2map(world_x, world_y)
		print ("our new goal is %d, %d on map"%(map_i,map_j))
		self.goal_state_map.set_pose(map_i, map_j, world_theta)


	#convert a point a map to the actual world position
	def map2world(self,map_i,map_j):
		world_x = -self.graphics.environment.width/2*self.graphics.scale + map_j
		world_y = self.graphics.environment.height/2*self.graphics.scale - map_i
		return world_x, world_y

	#convert a point in world coordinate to map pixel
	def world2map(self,world_x,world_y):
		map_i = int(self.graphics.environment.width/2*self.graphics.scale - world_y)
		map_j = int(self.graphics.environment.height/2*self.graphics.scale + world_x)
		if(map_i<0 or map_i>=self.map_width or map_j<0 or map_j>=self.map_height):
			warnings.warn("Pose %f, %f outside the current map limit"%(world_x,world_y))

		if(map_i<0):
			map_i=int(0)
		elif(map_i>=self.map_width):
			map_i=self.map_width-int(1)

		if(map_j<0):
			map_j=int(0)
		elif(map_j>=self.map_height):
			map_j=self.map_height-int(1)

		return map_i, map_j

	def _init_path_img(self):
		self.map_img_np = 255*np.ones((int(self.map_width),int(self.map_height),4),dtype = np.int16)
		# self.map_img_np[0:-1][0:-1][3] = 0
		self.map_img_np[:,:,3] = 0

	def _show_path(self):
		for pose in self.path.poses:
			map_i = pose.map_i
			map_j = pose.map_j 
			self.map_img_np[map_i][map_j][1] =0
			self.map_img_np[map_i][map_j][2] =0
			self.map_img_np[map_i][map_j][3] =255

		self.path_img=Image.frombytes('RGBA', (self.map_img_np.shape[1],self.map_img_np.shape[0]), self.map_img_np.astype('b').tostring())
		self.graphics.draw_path(self.path_img)

		# If you want to save the path as an image, un-comment the following line:
		# self.path_img.save('Log\path_img.png')
		
		# If you want to output an image of map and path, un-comment the following two lines
		# self.path_img = toimage(self.map_img_np)
		# self.path_img.show()

	def plan_path(self):

		# The major program you are going to write!
		# The following simply demo that how you can add pose to path
		self.path.clear_path()
		'''
		# # # Our cost map is in the global frame while this method functions in the map frame where the robot always starts at zero with a specified end condition.
		# grid=np.copy(self.costmap.costmap)

		# # define occupied spaces only 1 = interference, 0 = free
		# grid[grid != math.inf] = 0
		# grid[grid == math.inf] = 1

		'''
		# Testing conversion from global to map coordinate system 
		# addr=self.map2world(0,0)
		# print(-addr[0],addr[1])
		# print(self.costmap.costmap[int(addr[0])][int(addr[1])])

		# Set up initial conditions
		#start_p=[[self.start_state_map.map_i,self.start_state_map.map_j] ]
		points=[]
		  # Set for the start condition
		end_point=[self.goal_state_map.map_i,self.goal_state_map.map_j]
		# set up queue and counter
		count=itertools.count()
		next(count)
		pq=queue.PriorityQueue()
		#print(int(len(self.costmap.costmap)))
		stored_cost=math.inf*np.ones((len(self.costmap.costmap),len(self.costmap.costmap)),dtype=int)
		visited=np.zeros((len(self.costmap.costmap),len(self.costmap.costmap)),dtype=int)
		parent=np.zeros((len(self.costmap.costmap),len(self.costmap.costmap)),dtype=int)
		pq.put([0, next(count), [self.start_state_map.map_i, self.start_state_map.map_j] ]) # start with zero cost here

		goal_flag= False
		'''
		tick = 0
		print("Start while")
		tick=tick+1
		print("queue?", pq.qsize())
		value=pq.get_nowait()
		#cost, itk, addr = pq.get()
		print("queue haulted?", value)
		'''
		tick = 0
		# issue: This continues to revisit previous points
		while   tick < 100 : #pq.qsize() > 0 and goal_flag == False :
			tick=tick+1
			print("queue?", tick)
			cost, itk, addr = pq.get()
			print("queue haulted?", tick)
			print(addr)
			points.append(addr)
			if addr == end_point:
				goal_flag = True
				break
			x=addr[0]
			y=addr[1]
			visited[x][y]= True
			parent[x][y]= True
			for [i,j] in ([x+1,y], [x-1,y], [x,y+1], [x,y-1]):
				if parent[i][j] == True:
					continue  # dont bother searching parent nodes since you dont backtrack
				cost_path=self.costmap.costmap[i][j]
				if visited[i][j] == False:
					stored_cost[i][j] = cost_path + cost
					pq.put([stored_cost[i][j], next(count),[i,j]])
					visited[i][j] == True
				if visited[i][j] == True:
					if stored_cost[i][j] > (cost_path + cost):
						stored_cost[i][j] = cost_path + cost
						pq.put([stored_cost[i][j], next(count),[i,j]])




		print("done with while")
		print(points)








































		'''
		# Attempt 10/27 does not work at all
		# Set up initial conditions
		start_p=[[self.start_state_map.map_i,self.start_state_map.map_j] ]
		points=[]
		  # Set for the start condition
		end_point=[self.goal_state_map.map_i,self.goal_state_map.map_j]
		#print(points)
		#print(self.costmap.costmap[points[0]][points[1]])

		#print(Pose(map_i=points[0],map_j=points[1],theta=0))
		#self.path.add_pose(Pose(map_i=points[0],map_j=points[1],theta=0))

		# set up queue and counter
		count=itertools.count()
		pq=queue.PriorityQueue()
		#print(int(len(self.costmap.costmap)))
		stored_cost=np.zeros((len(self.costmap.costmap),len(self.costmap.costmap)),dtype=int)
		#stored_cost[points]=self.costmap.costmap[points[0]][points[1]]
		node=[self.costmap.costmap[self.start_state_map.map_i][self.start_state_map.map_j],next(count),[250,250]]
		pq.put(node)
		stored_cost[self.start_state_map.map_i][self.start_state_map.map_j]= self.costmap.costmap[self.start_state_map.map_i][self.start_state_map.map_j]
		#pq.put([math.inf,69])
		break_flag = False
		g=0
		while   pq.qsize() > 0 and break_flag == False and g < 10:  # pq.empty == False
			cost, k, addr = pq.get()
			#print('size of the queue',pq.qsize())
			points.append(addr)
			if addr == end_point:
				break
			stored_cost[addr[0]][addr[1]]=cost
			g+=1

			for [i,j] in ([addr[0]+1,addr[1]],[addr[0]-1,addr[1]],[addr[0],addr[1]+1],[addr[0],addr[1]-1]):
				node=[self.costmap.costmap[i][j]+cost,next(count),[i,j]]
				if self.costmap.costmap[i][j] == math.inf:
					break
				print(node)
				if stored_cost[i][j] == 0:
					pq.put(node)
					stored_cost[i][j] = self.costmap.costmap[i][j] + cost
				else:
					if stored_cost[i][j] > (self.costmap.costmap[i][j] + cost):
						#print("stored_cost",stored_cost[i][j])
						#print("cost after travel", (self.costmap.costmap[i][j] + cost))
						pq.put(node)
						stored_cost[i][j] == self.costmap.costmap[i][j] + cost
					#stored_cost[addr[0]][addr[1]]=self.costmap.costmap[addr[0]][addr[1]]
			# 		else: 
			# 			if stored_cost[addr[0]][addr[1]] > self.costmap.costmap[addr[0]][addr[1]]:
			# 				#pq.put(node)
			# 				stored_cost[addr[0]][addr[1]]=self.costmap.costmap[addr[0]][addr[1]]
						
			# 		print(i,j)
			#print([cost,k,addr])



			#print(pq.qsize())
		print("out")
		#print(points)


		'''




		# self.path.add_pose(Pose(100,100,0))
		# self.path.print_path()


		#self.path.save_path(file_name="Log\path.csv")

		#np.savetxt('Log/point.csv',grid, delimiter=',')





		
		# # points = bresenham(self.start_state_map.map_i,self.start_state_map.map_j,self.goal_state_map.map_i,self.goal_state_map.map_j)
		# self.path.print_path()
	
		# for p in points:
		# 	self.path.add_pose(Pose(map_i=p[0],map_j=p[1],theta=0)) #theta is wrong
		# self.path.print_path()
			
		# self.path.save_path(file_name="Log\path.csv")
	

# bresenham algorithm for line generation on grid map
# from http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm
def bresenham(x1, y1, x2, y2):
    """Bresenham's Line Algorithm
    Produces a list of tuples from start and end
 
    >>> points1 = get_line((0, 0), (3, 4))
    >>> points2 = get_line((3, 4), (0, 0))
    >>> assert(set(points1) == set(points2))
    >>> print points1
    [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
    >>> print points2
    [(3, 4), (2, 3), (1, 2), (1, 1), (0, 0)]
    """
    # Setup initial conditions

    dx = x2 - x1
    dy = y2 - y1
 
    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)
 
    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
 
    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
 
    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1
 
    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1
 
    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
 
    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()

    # print points
    return points
