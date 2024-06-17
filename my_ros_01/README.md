
# Topic
A topic is a named communication channel that allows nodes to exchange messages. Nodes can publish messages to a topic, and other nodes can subscribe to that topic to receive the messages.
## Usages
A common usage of Topic is data stream where the publisher and subscriber do not care who subscribed or published data.
## Example
From

    my_ros_01/my_ros_01/turtle_controller.py

This is how we create Topic publisher and subscriber

    self.publisher_cmd_vel_ = self.create_publisher(
			Twist, "turtle1/cmd_vel", 10
		)
  
    self.subscriber_pose_ = self.create_subscription(
			Pose, "turtle1/pose", self.callback_turtle_pose, 10
		)

The publisher topic is "turtle1/cmd_vel" and the message type being Twist
The subscriber topic is "turtle1/pose" and the message type being Pose

# Service/Client
The Service/Client model enables synchronous communication between nodes:
- Service: A node that provides a specific function and can respond to requests. It's defined by a name and a pair of message types (request and response).
- Client: A node that requests a service from another node. It sends a request message and waits for the response.
## Usages
A common usage of Service/Client is an action requested by client, for example, requesting to switch on an LED.
## Example
From 

    my_ros_01/my_ros_01/turtle_spawner.py

This is how we create service

    self.service_catch_turtle = self.create_service(
		    CatchTurtle, "catch_turtle", self.callback_catch_turtle
		)
  
which takes request of "catch_turtle" of message type CatchTurtle

From 

    my_ros_01/my_ros_01/turtle_controller.py

This is how we create client

    client = self.create_client(CatchTurtle, "catch_turtle")

which requests "catch_turtle" of message type CatchTurtle
# Custom Interface
A custom interface allows you to define your own message, service, or action types tailored to the specific needs of your application. This is useful when the built-in types do not meet your requirements.
## Usages
It is sometimes easier to use Custom Interface and it offers more flexibility.
## Example
From 

    my_ros_interface/srv/CatchTurtle.srv

The custom service has 2 data type inside, String and Bool, "---" is a seperator between request and response between client and server

    string name
    ---
    bool success

# Parameters
Parameters are key-value pairs that nodes can use to configure their behavior at runtime without changing the code. They provide flexibility and allow for dynamic adjustments.
## Usages
Like any parameters, it offers dynamic adjustments in the code.
## Example
This is the declaration of parameters and how to assign to a variable

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
   
# Launch file
A launch file in ROS2 is an XML or Python script used to start multiple nodes and set their configurations in a single command. Launch files simplify the process of managing and running complex robotic systems by allowing developers to specify node parameters, remappings, environment variables, and other startup options in a single place.
## Usages
They simplify complex setups by allowing you to configure nodes, set parameters, remap topics, and control execution order from a single file.
## Example
This launch file launches Node turtlesim_node from turtlesim, turtle_spawner from my_ros_01, and turtle_controller from my_ros_01 in this order.
Basically, it does this to the terminal, 

`ros2 run turtlesim turtlesim_node`
`ros2 run my_ros_01 turtle_spawner`
`ros2 run my_ros_01 turtle_controller`

    from launch import LaunchDescription
    from launch_ros.actions import Node


    def generate_launch_description():
    ld = LaunchDescription()

    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )

    turtle_spawner_node = Node(
        package="my_ros_01",
        executable="turtle_spawner",
        parameters=[
            {"spawn_frequency": 1.0},
            {"turtle_name_prefix": "my_turtle"}
        ]
    )

    turtle_controller_node = Node(
        package="my_ros_01",
        executable="turtle_controller",
        parameters=[
            {"catch_closest_turtle_first": True}
        ]
    )

    ld.add_action(turtlesim_node)
    ld.add_action(turtle_spawner_node)
    ld.add_action(turtle_controller_node)
    return ld
