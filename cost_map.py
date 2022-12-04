from importlib.util import set_loader
from logging.config import valid_ident
import cv2
import numpy as np
import math
from PIL import Image, ImageTk
from queue import Queue

class cost_map:
	def __init__(self,graphics):
		self.graphics = graphics
		# self.graphics.scale = 400 # This value should be same as the pixel value of the image
		self.inflation_radius = 18 # radius of our robot is 18 pixel or cm
		self.cloud=[]  # cloudmap
		self.got_cloud= False
		self.gravity_scale = 10 # set the scale factor of the gravity map
		self.graphics.environment.robots[0].set_bot_size(body_cm = 2*self.inflation_radius)
		#self.graphics.environment.width/height = 2
		self.map_width = int(self.graphics.environment.width*self.graphics.scale)
		self.map_height = int(self.graphics.environment.height*self.graphics.scale)
		try:
			self.load_map(map = "maps/Group_testmap3.png") #load map  maps/testmap.png  maps/smol_test_map.png
		except:
			self.graphics.show_map_button.configure(state="disabled")
			print ("no map loaded") #if fail to find the map png
			return
		self.show_map()
		self.compute_costmap()

		#self.show_costmap()
		self.save_vis_map(map = "maps/testcostmap2.png")

	#load occupancy grid into self.map
	#self.map is a numpy 2d array
	#initialize self.costmap, a numpy 2d array, same as self.map
	def load_map(self,map="maps/testmap2.png"):
		self.map_img = Image.open(map).convert('L')
		self.map_img = self.map_img.resize((int(self.map_width),int(self.map_height)),Image.ANTIALIAS)
		# self.graphics.draw_map(map_img=self.map_img)
		self.map = cv2.imread(map,cv2.IMREAD_GRAYSCALE)
		print (self.map.dtype)
		print ("Loaded map dimension: %d x %d pixel"%(self.map.shape[0],self.map.shape[1]))
		self.map = cv2.resize(self.map, dsize=(int(self.map_width),int(self.map_height)), interpolation=cv2.INTER_CUBIC)
		self.vis_map=np.copy(self.map) #map for visualization
		self.distmap=np.copy(self.map).astype(np.float)
		self.costmap=np.copy(self.map).astype(np.float)

	#save your costmap into a grayscale image
	def save_vis_map(self,map="maps/costmap.png"):
		save_img = Image.fromarray(self.vis_map)
		save_img.save(map)

	def show_vis_map(self):
		self.get_vis_map()
		self.vis_map_img=Image.frombytes('L', (self.vis_map.shape[1],self.vis_map.shape[0]), self.vis_map.astype('b').tostring())
		self.graphics.draw_map(map_img=self.vis_map_img)

	#display costmap on the dialogue window
	def show_costmap(self):
		self.costmap_img=Image.frombytes('L', (self.costmap.shape[1],self.costmap.shape[0]), self.costmap.astype('b').tostring())
		self.graphics.draw_map(map_img=self.costmap_img)

	#display binary occupancy grid on the dialogue window 
	def show_map(self):
		self.graphics.draw_map(map_img=self.map_img)


	#This is the function you really should update!

	def compute_costmap(self):
		#The following is an example how you manipulate the value in self.costmap
		#It has nothing to do with brushfire, replace it
		#A whilte pixel has value of 255, a black one is 0
		#However, some block value may be dark gray
		grid=np.copy(self.costmap)
		grid[grid == 0] = 1  #convert to island problem
		grid[grid > 1] = 0
		L=len(grid)
		q=[]
		q2=[]
		gridImage=np.zeros((L,L),dtype=int)
		brushMap=np.zeros((L,L),dtype=int)
		for i in range(L):
			for j in range(L):
				if grid[i,j]==1:
					q.append([i,j,0])
					gridImage[i][j]
				if grid[i,j]==0:
					q2.append([i,j,0])
		if len(q) == L*L  or len(q2) == L*L :
			distance= -1
		else:            
			distance=0
		'''
        right=[0,1]
        left=[0,-1]
        up=[-1,0]
        down=[1,0]
        dir=[up, down, left, right]
        '''
		while len(q) > 0 and distance != -1:
			[x,y,depth]=q.pop(0)
			if grid[x][y] == 0:
				distance = max(distance,depth)
                # ensures that the max distance is used from the number of depth steps
			for [i,j] in ([x+1,y],[x-1,y],[x,y+1],[x,y-1],[x+1,y+1],[x-1,y+1],[x-1,y-1]):  # can modify for 8 directions here  for [i,j] in ([x+1,y],[x-1,y],[x,y+1],[x,y-1]):
                # around the point of interest
				if i >=0 and i < L and j >=0 and j < L and gridImage[i][j] == 0:  
                    # verifies the point to peak is both in the grid and open
					gridImage[i][j] = 1
                    # flags point on map as seen already
					q.append([i,j,depth+1])
					brushMap[i][j]=depth+1 
				if i >=0 and i < L and j >=0 and j < L and gridImage[i][j] != 0 and grid[i][j] != 1: #see if we are too close to another 
					if brushMap[i][j] > (depth+1):  # keep the smaller one for brushfire mission statement
						brushMap[i][j] = (depth+1)
                    # specifies that the point is one space away from the previous
		# verifies that the brushfire map displays values limited from 2 to 255 ()
		value=brushMap - 2000*grid # Ensuring the starting points are negative
		value=value-self.inflation_radius # Inflation Time!
		value[value < 0] = 0  # Setting the walls to a zero

		bigNum=np.amax(value)
		giga=math.inf
		value=self.gravity_scale*abs(value-bigNum)  # reverse the order of the cost map to make the gravity map and multiply by a scaling factor
		value[value == self.gravity_scale*bigNum] =  giga # High cost associated with obstruction (10^9) Giga Chad
		
		self.costmap=value

		np.savetxt('Log/cost_map.csv',self.costmap, delimiter=',')
		pass


	def get_cloud(self):
		if self.got_cloud == False:
			self.got_cloud = True
			cloud=[]
			L=len(self.costmap)
			for i in range(L):
				for j in range(L):
					if self.costmap[i][j]==math.inf:
						cloud.append([i,j])
			print("cloud: ", cloud)
		cloud="pass the function test?"
		return cloud
		

	#scale costmap to 0 - 255 for visualization
	def get_vis_map(self):
		self.vis_map = np.uint8(255-self.costmap/16.0)
		#self.vis_map=np.uint(255-self.costmap/1.5)  # better view of map
		np.savetxt("Log/vismap.csv",self.vis_map, delimiter=',')   # np.savetxt("Log/vismap.txt",self.vis_map)

