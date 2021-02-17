import numpy as np

height = 10000              # vertical dimension in millimeters
width = 20000               # horizontal dimension in millimeters
object_length = 1000		# object length


def static_object_tracking(objects, delta_angle, speed, delta_time):
	theta = np.radians(delta_angle)
	c, s = np.cos(theta), np.sin(theta)
	rotation_matrix = np.array([[c, -s], [s, c]])
	distance = speed * delta_time
	[(x1, z1, x2, z2) for x1, z1, x2, z2 in objects]
	new = []
	for obj in objects:
		x1, z1, x2, z2 = obj
		rotated = rotation_matrix @ np.array([[x1, x2], [z1, z2]])
		new.add((rotated[0,0], rotated[1,0] - distance, rotated[0,1], rotated[1,1] - distance))
	return [(x1, z1, x2, z2) for x1, z1, x2, z2 in new if z1 >= 0 or z2 >= 0]

