import cv2
import numpy as np
import math
import random
import warnings
from PIL import Image, ImageTk

from Path import *
# from Queue import Queue
# Deas edit
import copy

class prm_node:
	def __init__(self,map_i=int(0),map_j=int(0)):
		self.map_i = map_i
		self.map_j = map_j
		self.edges = [] #edges of child nodes
		self.parent = None #parent node


class prm_edge:
	def __init__(self,node1=None,node2=None):
		self.node1 = node1 #parent node
		self.node2 = node2 #child node

#You may modify it to increase efficiency as list.append is slow
class prm_tree:
	def __init__(self):
		self.nodes = []
		self.edges = []

	def add_nodes(self,node):
		self.nodes.append(node)

	#add an edge to our PRM tree, node1 is parent, node2 is the kid
	def add_edges(self,node1,node2): 
		edge = prm_edge(node1,node2)
		self.edges.append(edge)
		node1.edges.append(edge)
		node2.parent=edge.node1


class path_planner:
	def __init__(self,graphics):
		self.graphics = graphics
		# self.graphics.scale = 400 #half pixel number on canvas, the map should be 800 x 800
		# self.graphics.environment.robots[0].set_bot_size(body_cm = 2*self.inflation_radius)
		#self.graphics.environment.width/height = 2

		self.costmap = self.graphics.map
		self.map_width = self.costmap.map_width
		self.map_height = self.costmap.map_height

		self.pTree=prm_tree()

		self._init_path_img()
		self.path = Path()
		
		self.set_start(world_x = .0, world_y = .0)
		self.set_goal(world_x = 100.0, world_y = 200.0, world_theta = .0)

		self.plan_path()
		self._show_path()

	def set_start(self, world_x = 0, world_y = 0, world_theta = 0):
		self.start_state_map = Pose()
		map_i, map_j = self.world2map(world_x,world_y)
		print("Start with %d, %d on map"%(map_i,map_j))
		self.start_state_map.set_pose(map_i,map_j,world_theta)
		self.start_node = prm_node(map_i,map_j)
		self.pTree.add_nodes(self.start_node)

	def set_goal(self, world_x, world_y, world_theta = 0):
		self.goal_state_map = Pose()
		goal_i, goal_j = self.world2map(world_x, world_y)
		print ("goal is %d, %d on map"%(goal_i, goal_j))
		self.goal_state_map.set_pose(goal_i, goal_j, world_theta)
		self.goal_node = prm_node(goal_i,goal_j)
		self.pTree.add_nodes(self.goal_node)

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

		np.savetxt("file.txt", self.map_img_np[1])

		self.path_img=Image.frombytes('RGBA', (self.map_img_np.shape[1],self.map_img_np.shape[0]), self.map_img_np.astype('b').tostring())
		# self.path_img = toimage(self.map_img_np)
		#self.path_img.show()
		self.graphics.draw_path(self.path_img)

	def check_vicinity(self,x1,y1,x2,y2,threshold = 10.0):
		if(math.sqrt((x1-x2)**2+(y1-y2)**2)<threshold):
			return True
		else:
			return False

	def plan_path(self):

		# This is the function you are going to work on
		###############################################################
		# Edge index variables
		cost_map_copy=np.copy(self.costmap.costmap)
		# for i in range(len(self.costmap.costmap)):
		# 	cost_map_copy[i][0] = math.inf
		# 	cost_map_copy[0][i] = math.inf
		# 	cost_map_copy[i][500-1] = math.inf
		# 	cost_map_copy[500-1][i] = math.inf
		cost_map_copy[cost_map_copy != math.inf] = 0
		# used to for child dictionary configuration
		seen_map=np.zeros((len(self.costmap.costmap),len(self.costmap.costmap)),dtype=bool)
		#np.savetxt('Log/prm_map.csv',cost_map_copy, delimiter=',')

		start_point=[self.start_state_map.map_i,self.start_state_map.map_j]
		end_point=[self.goal_state_map.map_i,self.goal_state_map.map_j]
		node=[self.start_state_map.map_i,self.start_state_map.map_j]
		road=[]
		road.append(start_point)
		if self.costmap.costmap[self.start_state_map.map_i][self.start_state_map.map_j] == math.inf or self.costmap.costmap[self.goal_state_map.map_i][self.goal_state_map.map_j] == math.inf :
			stop_flag = True
			print("Start or end goal out of bounds")
		else:
			stop_flag = False
		#road=[[1,1],[2,2],[3,3],[4,4]]
		#print(len(road))
		#print(road[random.randint(0,len(road)-1)])


		# Is a direct path possible? (instant solution)
		easyFlag=True
		edge=bresenham(start_point[0],start_point[1],end_point[0],end_point[1])
		dictParent={} # dict_node_parent[str([i,j])]=[x,y]
		dictChild={}
		seen={}
		for p in edge:
			if cost_map_copy[p[0]][p[1]] == math.inf:
				easyFlag = False
		if easyFlag:
			print("That was easy")
			for p in edge:
				self.path.add_pose(Pose(map_i=p[0], map_j=p[1], theta=0))

		
		iterCount=1000000
		flag=False
		for id in range(iterCount):
			count=id
			if easyFlag or stop_flag:
				break
			# Set a random parent node to expand on on the established road map
			parent=road[random.randint(0,len(road)-1)]
			# choose a random node chosen parent
			Pnode=[random.randint(0, self.map_width-1), random.randint(0, self.map_height-1)]
			# reject node if it has been already assigned as a perent (we dont want a cycle)
			if seen_map[Pnode[0]][Pnode[1]] == 1:
				continue
			if seen_map[parent[0]][parent[1]] == 0:
				dictChild[str(parent)]= [Pnode]
				seen_map[parent[0]][parent[1]] = 1
			else:
				dictChild[str(parent)].append(Pnode)
			# See of there is a path to the 
			if is_clear(cost_map_copy,parent,Pnode):
				#print("No Collision")
				dictParent[str([Pnode[0],Pnode[1]])] = parent # establish parent node relationship
				road.append(Pnode)
				if path_planner.check_vicinity(self,Pnode[0],Pnode[1],end_point[0],end_point[1]):
					print("close enough")
					flag=True
					break
		#print(road)
		# print("Parent",dictParent)
		# print("Child",dictChild)
		#print("roadmap",road)
		# for p in road:
		# 	self.path.add_pose(Pose(map_i=p[0], map_j=p[1], theta=0))

		# trace back the path using the parent nodes
		a=road.pop()
		road_return=[]
		road_return.append(a)
		while a != start_point:
			if a == start_point:
				break
			a=dictParent[str(a)]
			road_return.append(a)
		#print("road_return",road_return,"len",len(road_return))


		# Look for a shorter path:
		#print(road_return)

		# # Look for shortcuts from the detemined path (11/20 fixed?)
		shorter=[]
		copy_road=copy.copy(road_return)
		#copy_road.pop(0)
		print("Road_return",road_return,len(road_return),"copy",copy_road,len(copy_road))
		shorter.append(copy_road[0])
		# for i in range(len(copy_road)):
		trimFlag= True
		i=0
		while trimFlag==True:
			if easyFlag:
				break
			j=1
			while is_clear(cost_map_copy,copy_road[i],copy_road[len(copy_road)-j]) == False:
				#print("no path to ",copy_road[len(copy_road)-j],"from ",copy_road[i] )
				j=j+1
			#print("Valid path to ",copy_road[len(copy_road)-j],"from ",copy_road[i] )
			if is_clear(cost_map_copy,copy_road[i],copy_road[len(copy_road)-j]):
				shorter.append(copy_road[len(copy_road)-j])
			else:
				raise Exception("there is a problem")
			if copy_road[len(copy_road)-j] == start_point:
				break
			# iterate at the next farthest point
			i = len(copy_road)-j
		print("shorter", shorter)
		print("iterations",count)
		road_return=shorter





		# # # Look for shortcuts from the detemined path (bugged)
		# shorter=[]
		# copy_road=copy.copy(road_return)
		# #copy_road.pop(0)
		# print("Road_return",road_return,len(road_return),"copy",copy_road,len(copy_road))
		# shorter.append(copy_road[0])
		# for i in range(len(copy_road)):
		# 	if easyFlag:
		# 		break
		# 	j=1
		# 	while is_clear(cost_map_copy,copy_road[i],copy_road[len(copy_road)-j]) == False:
		# 		#print("no path to ",copy_road[len(copy_road)-j],"from ",copy_road[i] )
		# 		j=j+1
		# 	#print("Valid path to ",copy_road[len(copy_road)-j],"from ",copy_road[i] )
		# 	if is_clear(cost_map_copy,copy_road[i],copy_road[len(copy_road)-j]):
		# 		shorter.append(copy_road[len(copy_road)-j])
		# 	else:
		# 		raise Exception("there is a problem")
		# 	if copy_road[len(copy_road)-j] == start_point:
		# 		break
		# print("shorter", shorter)


		#road_return=shorter
		
		# for curent in copy_road:
		# 	print(curent)
		# 	value=copy_road.pop(1)
		# 	if value == start_point:
		# 		shorter.append(start_point)
		# 		break
		# 	prev=copy.copy(value)
		# 	while is_clear(cost_map_copy,value,curent):
		# 		if value == start_point:
		# 			prev = value
		# 			break
		# 		value=copy_road.pop(1)
		# 		prev=copy.copy(value)
		# 		shorter.append(prev)
		# shorter.insert(0,road_return[0])
		# print("Shorter path",shorter)
		#road_return=shorter

		# i=1
		# for curent in copy_road:

		# 	test=copy_road.pop(0)
		# 	if i == 1:
		# 		test=copy_road.pop(0)
		# 		i = 0
		# 	#prev=copy.copy(test)
		# 	i=0
		# 	while is_clear(cost_map_copy,curent,test):
		# 		if test == start_point:
		# 			copy_road.append(start_point)
		# 			break
		# 		print('is good')
		# 		prev=copy.copy(test)
		# 		test=copy_road.pop(0)
		# 	copy_road.append(prev)
		# 	print("test",test,"previous",prev)
		# print("Road_return",road_return,len(road_return),"copy",copy_road,len(copy_road))
		# for current in copy_road:
		# 	test=copy_road.pop(0)
		# 	while is_clear(cost_map_copy,current,test):
		# 		if test == start_point:
		# 			break
		# 		test=copy_road.pop(0)
		# 	shorter.append(test)
		# 	copy_road.insert(0,test)
		# 	print("copy_road",copy_road,"Shorter",shorter)




		# for i in copy_road:
		# 	#copy_road=copy.copy(road_return)
		# 	value=copy_road.pop(0)
		# 	if value == start_point:
		# 		shorter.append(value)
		# 		break
		# 	while is_clear(cost_map_copy,value,i) == True:
		# 		if value == start_point:
		# 			break
		# 		value=copy_road.pop()
		# 		print("value Popper",value)
		# 	shorter.append(value)
			# if value == start_point:
			# 	break
			# print(value)
			# shorter.append(value)
			#search=copy_road.pop(0)
			#print('search',search)
		#shorter=list(set(shorter))
		#print("Road_return",road_return,"Shorter",shorter)



		# Printing the path
		for i in range(len(road_return)-1):
			edge=bresenham(road_return[i][0],road_return[i][1],road_return[i+1][0],road_return[i+1][1])
			#print(road_return[i][0],road_return[i+1][0])
			for p in edge:
				self.path.add_pose(Pose(map_i=p[0], map_j=p[1], theta=0))
			
			#print(road_return[0])

		# print("done",flag,"count",count, "last Node", a)
		# if easyFlag == False:
		# 	b=dictParent[str(a)]
		# else:
		# 	b=[0,0]
		# print("last parent", b)
		# self.path.add_pose(Pose(map_i=a[0], map_j=a[1], theta=0))
		# self.path.add_pose(Pose(map_i=b[0], map_j=b[1], theta=0))
		# plot everything
		#print(road)
		#print(dictParent)
			# for visulization test
			# for p in edge:
			# 	self.path.add_pose(Pose(map_i=p[0], map_j=p[1], theta=0))
			# edge=bresenham(Pnode[0],Pnode[1],end_point[0],end_point[1])
			# lastFlag=True
			# for p in edge:
			# 	if cost_map_copy[p[0]][p[1]] == math.inf:
			# 		lastFlag = False
			# if lastFlag == True:
			# 	for p in edge:
			# 		self.path.add_pose(Pose(map_i=p[0], map_j=p[1], theta=0))
			
			
			
			



		



		# #count=itertools.count()
		# itermax=100

		# for count in range(itermax):
		# 	print("count",count)
		





		# # Sean's work
		# a = 0  			
		# b = 0
		
		# # Initialize a list to store all valid points and append the initial configuration
		# road = []
		# ri_i = self.start_state_map.map_i
		# rj_i = self.start_state_map.map_j
		# road.append([ri_i, rj_i])

		# # Main loop
		# while True:

		# 	# Pick a random point on the map and draw a straight line connecting it to the start point
		# 	ri_f = random.randint(0, self.map_width)
		# 	rj_f = random.randint(0, self.map_height) # Let's make a random number!
		# 	points = bresenham(ri_i, rj_i, ri_f, rj_f)
			
		# 	# Check if the line from start initial to final point hits any obstacle
		# 	# Initialize boolean variable for obstacle interference
		# 	hit_obstacle = False

		# 	# Iterate through the points in the straight line
		# 	for p in points:
		# 		# If we hit an obstacle, change the boolean variable and break out of the loop
		# 		if self.costmap.costmap[p[0]][p[1]] < 255: # Depends on how you set the value of obstacle
		# 			hit_obstacle = True
		# 			# print ("From %d, %d to %d, %d we hit obstalce"%(self.start_node.map_i,self.start_node.map_j,ri,rj))
		# 			break

		# 		# If we didn't hit an obstacle, assign the coordinates as another node and append to the road
		# 		else:
		# 			# 
		# 			road.append([ri_f, rj_f])

		# 			#self.pTree.add_nodes(random_node)
		# 			#self.pTree.add_edges(self.start_node, random_node) # Add an edge from start node to random node

		# 	# Reinitialize variables for next loop iteration
		# 	ri_i = ri_f
		# 	rj_i = rj_f

		# 	##############################################################

		# 	# If you decide the path between start_node and random_node should be within your final path, you must do:
		# 	points = bresenham(self.start_node.map_i, self.start_node.map_j, ri_f, rj_f)
		# 	for p in points:
		# 		self.path.add_pose(Pose(map_i=p[0], map_j=p[1], theta=0))

		# 	# It is almost impossible for you to random a node that is coincident with goal node
		# 	# So everytime you randomed ri and rj, you should also check if it is within a vicinity of goal
		# 	# Define check_vicinity function and decide if you have reached the goal
		# 	if(self.check_vicinity(self.goal_node.map_i, self.goal_node.map_j, ri_f, rj_f, 2.0)):
		# 		print ("We hit goal!")
		# 		break

		# # Save the path to observe it later
		# self.path.save_path(file_name="Log\prm_path.csv")

# determines if a path is clear between 2 nodes on a map Returns a bool
def is_clear(map,point1,point2):
	value=True
	edge=bresenham(point1[0],point1[1],point2[0],point2[1])
	for p in edge:
		if map[p[0]][p[1]] == math.inf:
				value=False
				break
	return value


# bresenham algorithm for line generation on grid map
# from http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm
def bresenham(x1, y1, x2, y2):
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
