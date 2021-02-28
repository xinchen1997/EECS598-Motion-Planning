import openravepy
import numpy as np
import Queue
import math

class Node:
	def __init__(self, parent, config):
		self.parent = parent       # node
		self.config = config     # [x, y, theta]
		self.g_cost = 0.0        # path cost to start
		self.f_cost = 0.0		 # path cost to goal

class Astar:
	def __init__(self, goalconfig):
		self.close_set = []
		self.open_set = []
		self.q = Queue.PriorityQueue()
		self.startconfig = [-3.4, -1.4, 0]
		self.goalconfig = goalconfig
		self.path = []
		self.path_cost = 0.0
		self.collision_free_config = []
		self.collision_config = []

	def collision(self, config, env, robot):
		with env:
			robot.SetActiveDOFValues(config)
    		return env.CheckCollision(robot)

	def isGoal(self, config):
		if (abs(config[0] - self.goalconfig[0]) < 1e-5) and (abs(config[1] - self.goalconfig[1]) < 1e-5) and (abs(config[2] - self.goalconfig[2]) < 1e-5):
			return True
		return False

	def Manhattan_heuristic(self, config):
		# w = 0.1 # weight for theta
		dis_x = abs(config[0] - self.goalconfig[0])
		dis_y = abs(config[1] - self.goalconfig[1])
		dis_theta = min(abs(config[2] - self.goalconfig[2]), 2 * math.pi - abs(config[2] - self.goalconfig[2]))# * w
		return dis_x + dis_y + dis_theta

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
		self.path_cost = node.f_cost
		while (node.parent != None):
			self.path.append(node.config)
			node = node.parent
		self.path.append(node.config)
		self.path.reverse()

	def fourConnectedAstar(self, heuristic, env, robot):
		start = Node(None, self.startconfig)
		self.q.put((0, start))
		self.open_set.append(self.startconfig)

		print 'planning begin'
		while(not self.q.empty()):
			curr_node = self.q.get()[1]
			# print 'current node config: ', curr_node.config
			if self.isGoal(curr_node.config):
				print 'reach goal'
				self.findPath(curr_node)
				# print 'path: ', self.path
				return self.path
			
			self.open_set.remove(curr_node.config)
			self.close_set.append(curr_node.config)

			# Process all neighbors
			x = curr_node.config[0]
			y = curr_node.config[1]
			theta = curr_node.config[2]
			for dx in [-0.1, 0, 0.1]:
				for dy in [-0.1, 0, 0.1]:
					for dtheta in [-math.pi / 2, 0, math.pi / 2]:
						if ((dx == 0) and (dy == 0) and (dtheta == 0)) or (abs(dx) == abs(dy)):
							continue
						else:
							self.processPoint(x + dx, y + dy, theta + dtheta, curr_node, heuristic, env, robot)

	def eightConnectedAstar(self, heuristic, env, robot):
		start = Node(None, self.startconfig)
		self.q.put((0, start))
		self.open_set.append(self.startconfig)

		print 'planning begin'
		while(not self.q.empty()):
			curr_node = self.q.get()[1]
			# print 'current node config: ', curr_node.config
			if self.isGoal(curr_node.config):
				print 'reach goal'
				self.findPath(curr_node)
				# print 'path: ', self.path
				return self.path
			
			self.open_set.remove(curr_node.config)
			self.close_set.append(curr_node.config)

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
							self.processPoint(x + dx, y + dy, theta + dtheta, curr_node, heuristic, env, robot)

	def processPoint(self, x, y, theta, parent, heuristic, env, robot):
		theta = self.calculate_theta(theta)
		config = [x, y, theta]
		if self.collision(config, env, robot):
			self.collision_config.append(config)
			# print 'collision:', config
			return
		else:
			self.collision_free_config.append(config)
			node = Node(None, config)
			g_cost = parent.g_cost + self.Step_Cost(parent.config, config)

	        if config in self.close_set:
	        	return
        	elif config not in self.open_set:
        		if heuristic == 'Manhattan':
        			f_cost = g_cost + self.Manhattan_heuristic(config)
        		if heuristic == 'Euclidean':
        			f_cost = g_cost + self.Euclidean_heuristic(config)
        		node.parent = parent
        		node.g_cost = g_cost
        		node.f_cost = f_cost
        		self.q.put((f_cost, node))
        		self.open_set.append(config)