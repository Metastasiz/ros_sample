#!/usr/bin/env python3
from functools import partial
import random
import math
import rclpy
from rclpy.node import Node

from turtlesim.srv import Spawn
from turtlesim.srv import Kill

from my_ros_interface.msg import Turtle
from my_ros_interface.msg import TurtleArray
from my_ros_interface.srv import CatchTurtle

class TurtleSpawner(Node): #modify name
	def __init__(self):
		super().__init__("turtle_spawner") #modify name
		#contants
		self.parm_spawner_frequency__ = "spawner_frequency"
		self.parm_turtle_name_prefix__ = "turtle_name_prefix"

		#parameters
		self.declare_parameter(self.parm_spawner_frequency__,1)
		self.declare_parameter(self.parm_turtle_name_prefix__,"turtle")

		self.spawner_frequency_ = self.get_parameter(
			self.parm_spawner_frequency__).value
		self.turtle_name_prefix_ = self.get_parameter(
			self.parm_turtle_name_prefix__).value
		self.turtle_counter_ = 0
		self.turtle_list_ = []
		
		self.publisher_turtle_list_ = self.create_publisher(
			TurtleArray, "turtle_list", 10
		)
		self.timer_spawn_turtle_ = self.create_timer(
			self.spawner_frequency_, self.spawn_new_turtle
		)
		self.service_catch_turtle = self.create_service(
			CatchTurtle, "catch_turtle", self.callback_catch_turtle
		)

	def callback_catch_turtle(self, request, response):
		self.call_kill_server(request.name)
		response.success = True
		return response

	def publish_turtle_list(self):
		msg = TurtleArray()
		msg.turtles = self.turtle_list_
		self.publisher_turtle_list_.publish(msg)

	def spawn_new_turtle(self):
		#contants
		x_min = 0
		x_max = 11
		y_min = 0
		y_max = 11

		self.turtle_counter_ += 1
		name = self.turtle_name_prefix_ + str(self.turtle_counter_)

		x = random.uniform(x_min,x_max)
		y = random.uniform(y_min,y_max)
		theta = random.uniform(0, 2*math.pi)

		self.call_spawn_server(name,x,y,theta)

	def call_spawn_server(self, turtle_name, x, y, theta):
		client = self.create_client(Spawn, "spawn")
		while not client.wait_for_service(1.0):
			self.get_logger().warn("Waiting for Server...")

		request = Spawn.Request()
		request.x = x
		request.y = y
		request.theta = theta
		request.name = turtle_name

		future = client.call_async(request)
		future.add_done_callback(
			partial(self.callback_call_spawn, turtle_name=turtle_name, x=x, y=y, theta=theta)
		)

	def callback_call_spawn(self,future, turtle_name,x,y,theta):
		try:
			response = future.result()
			
			if response.name != "":
				self.get_logger().info("Turtle " + response.name + " has been spawned")

				turtle_new = Turtle()
				turtle_new.name = response.name
				turtle_new.x = x
				turtle_new.y = y
				turtle_new.theta = theta

				self.turtle_list_.append(turtle_new)
				self.publish_turtle_list()
				
		except Exception as e:
			self.get_logger().error("Service called failed %r" % (e,))

	def call_kill_server(self, turtle_name):
		client = self.create_client(Kill, "kill")
		while not client.wait_for_service(1.0):
			self.get_logger().warn("Waiting for Server...")

		request = Kill.Request()
		request.name = turtle_name

		future = client.call_async(request)
		future.add_done_callback(
			partial(self.callback_call_kill, turtle_name=turtle_name)
		)

	def callback_call_kill(self,future, turtle_name):
		try:
			#response has nothing
			future.result()

			for (i, turtle) in enumerate(self.turtle_list_):
				if turtle.name == turtle_name:
					del self.turtle_list_[i]
					self.publish_turtle_list()
					break

		except Exception as e:
			self.get_logger().error("Service called failed %r" % (e,))


def main(args=None):
	rclpy.init(args=args)
	node = TurtleSpawner() #modify name
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()