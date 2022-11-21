# PRM old attemps
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

# Sean's work

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

# More stuff:
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