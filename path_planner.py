from cmath import cos
from multiprocessing.sharedctypes import Value
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
		self.set_goal(world_x = 100.0, world_y = 200.0, world_theta = .0)  #self.set_goal(world_x = 100.0, world_y = 200.0, world_theta = .0)

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
		# Huristic constant
		H=12
		option = 1 # 1 2 3 4
		def Heuristics(x,y):
			match option:
				case 1:  # euclidian distance
					val=math.sqrt((x-self.goal_state_map.map_i)**2+(y-self.goal_state_map.map_j)**2)
				case 2:  # arithmetic mean
					val=(abs(x-self.goal_state_map.map_i)+ abs(y-self.goal_state_map.map_j))/2
				case 3:  # geometric mean
					val=math.sqrt(abs(x-self.goal_state_map.map_i)*abs(y-self.goal_state_map.map_j))
				case 4:  # arithmetic mean with right shift
					val=(abs(x-self.goal_state_map.map_i)+ abs(y-self.goal_state_map.map_j))>>1
			return  val


		# Set up initial conditions
		#start_p=[[self.start_state_map.map_i,self.start_state_map.map_j] ]
		points=[]
		  # Set for the start condition
		end_point=[self.goal_state_map.map_i,self.goal_state_map.map_j]

		# set up costmap with boundary at the edges
		cost_map_copy=np.copy(self.costmap.costmap)
		for i in range(len(self.costmap.costmap)):
			cost_map_copy[i][0] = math.inf
			cost_map_copy[0][i] = math.inf
			cost_map_copy[i][500-1] = math.inf
			cost_map_copy[500-1][i] = math.inf
		#np.savetxt('Log/cost_map_copy.csv',cost_map_copy, delimiter=',')
		stored_cost=math.inf*np.ones((len(self.costmap.costmap),len(self.costmap.costmap)),dtype=int)
		stored_cost[self.start_state_map.map_i][self.start_state_map.map_j] = 0
		#np.savetxt('Log/distance_from_start.csv',distance_from_start, delimiter=',')
		parent_map=np.zeros((len(self.costmap.costmap),len(self.costmap.costmap)),dtype=bool)
		#np.savetxt('Log/parent_map.csv',parent_map, delimiter=',')
		seen_map=np.zeros((len(self.costmap.costmap),len(self.costmap.costmap)),dtype=bool)
		seen_map[self.start_state_map.map_i][self.start_state_map.map_j]= True
		#np.savetxt('Log/seen_map.csv',seen_map, delimiter=',')

		# node parent dictionary
		dict_node_parent={}


		# Configure queue and counter
		count=itertools.count()
		#next(count)  # Iterate with this command
		pq=queue.PriorityQueue()
		pq.put([stored_cost[self.start_state_map.map_i][self.start_state_map.map_j], next(count), [self.start_state_map.map_i, self.start_state_map.map_j] ]) # start with zero cost here
		

		if self.costmap.costmap[self.start_state_map.map_i][self.start_state_map.map_j] == math.inf or self.costmap.costmap[self.goal_state_map.map_i][self.goal_state_map.map_j] == math.inf :
			goal_flag = True
			print("Start or end goal out of bounds")
		else:
			goal_flag = False
		# goal_flag = False

		while pq.qsize() > 0 or goal_flag == False :
			cost, counter,[x,y] = pq.get()
			#print([x,y], end_point)
			# breakout condition if goal point is rached
			if [x,y] == end_point:
				goal_flag = True
				final_cost=cost
				#print('pushed point', [x,y], "goal point", end_point)
				break
			# if parent (popped before)  continue logic since we dont want this
			if parent_map[x][y] == True:
				continue
			# set value as parent that will not be popped again
			parent_map[x][y] = True
			# Set parent that will be used for back drive dictornary list 
			parent_value= [x,y]
			# set breakout condition for testing
			#if counter > 48:
			#	break
			#goal_flag = True # Lets do this for one interation

			# search connected nodes
			for [i,j] in ([x+1,y], [x-1,y], [x,y+1], [x,y-1]):
				cost_reference=cost_map_copy[i][j]
				#print("cost: ",cost, "cost Reference: ", cost_reference, "cost + costReference: ", cost+cost_reference)
				if cost_reference == math.inf:
					continue
				if seen_map[i][j]==False and parent_map[i][j] == False:
					# new cost function + Heuristic
					new_cost=cost_reference+cost+H*Heuristics(i,j)
					# add to queue
					pq.put([new_cost,next(count),[i,j]])
					# save node parent relationship
					dict_node_parent[str([i,j])]=[x,y]
					# specify that this has been seen
					seen_map[i][j] = True
					# add to stored cost matrix
					stored_cost[i][j] = new_cost

				if seen_map[i][j]==True and (stored_cost[i][j] > cost_reference+cost+H*Heuristics(i,j)) and parent_map[i][j] == False:
					# new cost function + Heuristic
					new_cost=cost_reference+cost+Heuristics(i,j)
					# add to queue
					pq.put([new_cost,next(count),[i,j]])
					# save node parent relationship
					dict_node_parent[str([i,j])]=[x,y]
					# add to stored cost matrix
					stored_cost[i][j] = new_cost
				# if parent_map[i][j] == True:
				# 	print('parent node dont back track')

		#print("end point" ,end_point , "parent",dict_node_parent[str(end_point)])

		# backtrack queue through the dictinary node parent relationship
		if goal_flag == True:
			points.append(end_point)
			track=dict_node_parent[str(end_point)]
			points.append(track)
			while track != [self.start_state_map.map_i, self.start_state_map.map_j]:
				points.append(track)
				track=dict_node_parent[str(track)]
			points.append([self.start_state_map.map_i, self.start_state_map.map_j])
		else:
			print("no path possible to goal")
		# print(points)

		# # Print maps to check
		# np.savetxt('Log/parent_map.csv',parent_map, delimiter=',')
		# np.savetxt('Log/seen_map.csv',seen_map, delimiter=',')
		# np.savetxt('Log/stored_cost.csv',stored_cost, delimiter=',')
		
		# add points to the map
		if goal_flag == True:
			for p in points:
				self.path.add_pose(Pose(map_i=p[0],map_j=p[1],theta=0)) #theta is wrong
			self.path.print_path()
		print("final cost: ", float("{:.1f}".format(final_cost)),"number of iterations: ", count)
	
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
