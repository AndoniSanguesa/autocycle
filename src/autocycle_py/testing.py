import numpy as np
from matplotlib import pyplot as plt

box_dist = 10000  # distance in each dimesion surrounding line segment
trans_mat = np.eye(3)  # translation matrix
rot_mat = np.zeros((3, 3))  # rotation matrix
rot_mat[2, 2] = 1
point_mat = np.zeros((3, 4))  # matrix with point 1 in col 0 and point 2 in col 1
point_mat[2, :] = [1, 1, 1, 1]

def diff(p1, p2):
    return (p1[0] - p2[0],
            p1[1] - p2[1],
            p1[0] * p2[1] - p2[0] * p1[1])

def box_verts(x1, x2, z1, z2):
    global point_mat
    point_mat[0, 0] = x1
    point_mat[1, 0] = z1
    point_mat[0, 1] = x1
    point_mat[1, 1] = z1
    point_mat[0, 2] = x2
    point_mat[1, 2] = z2
    point_mat[0, 3] = x2
    point_mat[1, 3] = z2

    # Translation matrix translates line segment so (x1, z1) is at the origin
    trans_mat[0, 2] = -x1
    trans_mat[1, 2] = -z1

    # Find angle, theta, between x axis and segment. Rotate counterclockwise -theta.
    slope = (z2 - z1) / (x2 - x1)
    theta = np.arctan(slope)
    rot_mat[0, 0] = np.cos(-theta)
    rot_mat[0, 1] = np.sin(-theta)
    rot_mat[1, 0] = -rot_mat[0, 1]
    rot_mat[1, 1] = rot_mat[0, 0]

    point_mat = rot_mat @ trans_mat @ point_mat
    # bot left
    point_mat[0, 0] -= box_dist
    point_mat[1, 0] -= box_dist

    # top left
    point_mat[0, 1] -= box_dist
    point_mat[1, 1] += box_dist

    # top right
    point_mat[0, 2] += box_dist
    point_mat[1, 2] += box_dist

    # bottom right
    point_mat[0, 3] += box_dist
    point_mat[1, 3] -= box_dist

    rot_mat[0, 0] = np.cos(theta)
    rot_mat[0, 1] = np.sin(theta)
    rot_mat[1, 0] = -rot_mat[0, 1]
    rot_mat[1, 1] = rot_mat[0, 0]

    trans_mat[0, 2] = x1
    trans_mat[1, 2] = z1

    point_mat = trans_mat @ rot_mat @ point_mat
    return ((point_mat[0, 0], point_mat[1, 0]),
            (point_mat[0, 1], point_mat[1, 1]),
            (point_mat[0, 2], point_mat[1, 2]),
            (point_mat[0, 3], point_mat[1, 3]))


def intersection(x1, x2, z1, z2, objects):
    objects = map(lambda o: (o.x1, o.x2, o.z1, o.z2), objects)
    if x1 < x2:
        points = box_verts(x1, x2, z1, z2)
    elif x1 > x2:
        points = box_verts(x2, x1, z2, z1)
    else:
        if z1 < z2:
            points = ((x1 - box_dist, z1 - box_dist),
                      (x2 - box_dist, z2 + box_dist),
                      (x2 + box_dist, z2 + box_dist),
                      (x1 + box_dist, z1 - box_dist))
        else:
            points = ((x2 - box_dist, z2 - box_dist),
                      (x1 - box_dist, z1 + box_dist),
                      (x1 + box_dist, z1 + box_dist),
                      (x2 + box_dist, z2 - box_dist))
    p2p3 = (points[2][0] - points[1][0], points[2][1] - points[1][1])
    p2p1 = (points[0][0] - points[1][0], points[0][1] - points[1][1])

    point_diffs = (
    diff(points[0], points[1]), diff(points[0], points[2]), diff(points[1], points[3]), diff(points[2], points[3]))

    for x3, x4, z3, z4 in objects:
        # intersection = []
        if x3 > x4:
            temp = (x3, z3)
            x3, z3 = (x4, z4)
            x4, z4 = temp
        diffs_obj = (x3 - x4, z3 - z4, x3 * z4 - x4 * z3)

        for doni in range(4):
            if (D := point_diffs[doni][0] * diffs_obj[1] - point_diffs[doni][1] * diffs_obj[0]) != 0:
                x = (point_diffs[doni][2] * diffs_obj[1] - point_diffs[doni][1] * diffs_obj[2]) / D
                if x3 <= x <= x4:
                    y = (point_diffs[doni][0] * diffs_obj[2] - point_diffs[doni][2] * diffs_obj[0]) / D
                    p2m = (x - points[0][0], y - points[0][1])
                    if 0 <= np.dot(p2m, p2p3) < np.dot(p2p3, p2p3) and 0 <= np.dot(p2m, p2p1) < np.dot(p2p1, p2p1):
                        return True
    return False

X = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

plt.plot([])