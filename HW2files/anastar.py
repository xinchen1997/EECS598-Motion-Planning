import openravepy
import numpy as np
import Queue
import math
import time

class Node:
	def __init__(self, parent, config):
		self.parent = parent       		# node
		self.config = config     		# [x, y, theta]
		self.e = 0.0					# e(s) = (G - g(s)) / h(s)
		self.g_cost = float("inf")      # path cost to start

class ANAstar:
	def __init__(self, goalconfig):
		self.open_set = Queue.PriorityQueue()
		self.close_set = {}
		self.startconfig = [-3.4, -1.4, 0]
		self.goalconfig = goalconfig
		self.path = []
		self.path_cost = 0.0
		self.collision_free_config = []
		self.collision_config = []
		self.G = 10000.0     		# the cost of the current-best solution
		self.E = 10000.0

	def collision(self, config, env, robot):
		with env:
			robot.SetActiveDOFValues(config)
    		return env.CheckCollision(robot)

	def isGoal(self, config):
		if (abs(config[0] - self.goalconfig[0]) < 1e-5) and (abs(config[1] - self.goalconfig[1]) < 1e-5) and (abs(config[2] - self.goalconfig[2]) < 1e-5):
			return True
		return False

	def Euclidean_heuristic(self, config):
		# w = 0.1 # weight for theta
		dis_x = abs(config[0] - self.goalconfig[0])
		dis_y = abs(config[1] - self.goalconfig[1])
		dis_theta = min(abs(config[2] - self.goalconfig[2]), 2 * math.pi - abs(config[2] - self.goalconfig[2]))# * w
		return np.sqrt(dis_x * dis_x + dis_y * dis_y + dis_theta * dis_theta)

	def Step_Cost(self, p_config, s_config):
		# w = 0.1 # weight for theta
		dis_x = abs(p_config[0] - s_config[0])
		dis_y = abs(p_config[1] - s_config[1])
		dis_theta = min(abs(p_config[2] - s_config[2]), 2 * math.pi - abs(p_config[2] - s_config[2]))# * w
		return np.sqrt(dis_x * dis_x + dis_y * dis_y + dis_theta * dis_theta)

	def calculate_theta(self, theta):
		while theta > math.pi:
			theta -= 2 * math.pi
		while theta < -math.pi:
			theta += 2 * math.pi
		return theta

	def findPath(self, node):
		del self.path[::]
		self.path_cost = node.g_cost
		while (node.parent != None):
			self.path.append(node.config)
			node = node.parent
		self.path.append(node.config)
		self.path.reverse()

	def compute_e(self, g, h):
		e = (self.G - g) / (h + 1e-10)
		return e

	def eightConnectedANAstar(self, env, robot):
		start_time = time.clock()
		start = Node(None, self.startconfig)
		start.g_cost = 0.0
		h_cost = self.Euclidean_heuristic(self.startconfig)
		start.e = self.compute_e(start.g_cost, h_cost)
		self.open_set.put((-start.e, start))
		self.close_set[tuple(start.config)] = start.g_cost

		print 'planning begin'
		i = 1
		while self.open_set.qsize():
			# print 'i', i
			self.improveSolution(env, robot)
			# Update keys e(s)in OPEN and prune if g(s)+h(s)>=G
			print "time cost:", time.clock() - start_time
			print "solution cost:", self.path_cost
			print ""

			# if i == 3:
			# 	with env:
			# 		handles = []
			# 		lastconfig = self.path[0]
   #  				for j in range(1, len(self.path)):
   #      				currentconfig = self.path[j]
   #      				handles.append(env.drawlinestrip(points=np.array(((lastconfig[0], lastconfig[1], 0.3), (currentconfig[0], currentconfig[1], 0.3))),
   #                                     					 linewidth=5.0,
   #                                     					 colors=np.array(((0,1,1)))))
   #      				lastconfig = currentconfig
			self.updateOpenSet()
			i += 1
			if i > 10:
				print "Time Cost Too Expensive"
				break

	def improveSolution(self, env, robot):
		while self.open_set.qsize():
			curr_node = self.open_set.get()[1]
			curr_g = self.close_set[tuple(curr_node.config)]

			if curr_node.e < self.E:
				self.E = curr_node.e

			if self.isGoal(curr_node.config):
				# print 'reach goal'
				self.G = curr_g
				self.findPath(curr_node)
				print "Suboptimal Solution Found!"
				return
			
			# Process all neighbors
			x = curr_node.config[0]
			y = curr_node.config[1]
			theta = curr_node.config[2]
			for dx in [-0.1, 0, 0.1]:
				for dy in [-0.1, 0, 0.1]:
					for dtheta in [-math.pi / 4, 0, math.pi / 4]:
						if (dx == 0) and (dy == 0) and (dtheta == 0):
							continue
						else:
							self.processPoint(x + dx, y + dy, theta + dtheta, curr_node, env, robot)

	def processPoint(self, x, y, theta, parent, env, robot):
		theta = self.calculate_theta(theta)
		config = [x, y, theta]
		node = Node(None, config)
		updateFlag = False
		new_cost = parent.g_cost + self.Step_Cost(parent.config, config)

		if self.collision(config, env, robot):
			if config not in self.collision_config:
				self.collision_config.append(config)
		else:
			if config not in self.collision_free_config:
				self.collision_free_config.append(config)

			if tuple(config) not in self.close_set:
				updateFlag = True
			else:
				if new_cost < self.close_set[tuple(config)]:
					updateFlag = True

		if updateFlag:
			self.close_set[tuple(config)] = new_cost
			node.g_cost = new_cost
			h_cost = self.Euclidean_heuristic(config)
			if new_cost + h_cost < self.G:
				node.e = self.compute_e(node.g_cost, h_cost)
				self.open_set.put((-node.e, node))
			node.parent = parent

	def updateOpenSet(self):
		updated_open_set = Queue.PriorityQueue()
		# print 'open set size:', self.open_set.qsize()
		while self.open_set.qsize():
			node = self.open_set.get()[1]
			g = self.close_set[tuple(node.config)]
			h = self.Euclidean_heuristic(node.config)
			if g + h < self.G:
				new_e = self.compute_e(g, h)
				node.e = new_e
				updated_open_set.put((-new_e, node))
		self.open_set = updated_open_set