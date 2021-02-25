import numpy as np
import rospy
from autocycle.msg import Object, ObjectList

height = 10000              # vertical dimension in millimeters
width = 20000               # horizontal dimension in millimeters
object_length = 1000		# object length

objects = []
heading = 0
time = 0

def new_object(obj):
	objects.append(obj)

def static_object_tracking():
	global objects, heading, time
	# Registers the Node with the Master
	rospy.init_node("static_object_tracking")

	# Waits for data getter service
	rospy.wait_for_service('get_data')

	# Creates the service client that will collect data
	data_getter = rospy.ServiceProxy("get_data", GetData)

	# Subscribes to the object topic
	rospy.Subscriber("cycle/objects", Object, new_object)

	# Publishes objects to path topic
	pub = rospy.Publisher("cycle/path", ObejctList, queue_size=1)

	heading = data_getter("heading").data
	time = data_getter("met").data

	while rospy.ok():
		# Picks up new objects
		rospy.spinOnce()

		# Collects data
		new_heading = data_getter("heading").data
		delta_angle = new_heading - heading
		heading = new_heading

		new_time = data_getter("met").data
		delta_time = new_time - time
		time = new_time
		
		speed = data_getter("vel").data

		theta = delta_angle
		c, s = np.cos(theta), np.sin(theta)
		rotation_matrix = np.array([[c, -s], [s, c]])
		distance = speed * delta_time
		new = []
		for obj in objects:
			rotated = rotation_matrix @ np.array([[obj.x1, obj.x2], [obj.z1, obj.z2]])
			new.append(Object(rotated[0,0], rotated[1,0] - distance, rotated[0,1], rotated[1,1] - distance))
		objects = list(filter(lambda x: x.z1 >= 0 or x.z2 >= 0, new))

	pub.publish(objects)

