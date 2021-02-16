import numpy as np

height = 10000              # vertical dimension in millimeters
width = 20000               # horizontal dimension in millimeters
cell_dim = 20               # dimension of cells in millimeters (cells are squares)

cell_col = np.ceil(height/cell_dim)
cell_row = np.ceil(width/cell_dim)

# Tunable parameters to determine if something is an object.
col_diff = 20           # Expected max difference between two adjacent cells in a column.
counter_reps = 3        # Number of reps required to dictate it is an object.
prev_diff = 1           # Amount of error needed to exceed for previous to be flags
same_obj_diff = 20      # maximum diff between horizontal cells to be considered the same object


def object_detection(points):

    cells = np.zeros((cell_row, cell_col))

    for tup in points:
        # creates list with x, y, and z coordinate
        x, y, z = tup
        x = x//cell_col
        y = y//cell_row

        # Dictating the z value for the cell. Currently only finds the minimum value of the cell.
        if x >= 0 and x < cell_col and y >= 0 and y < cell_row:
            cells[x, y] = z if cells[x, y] == 0 else min(z, cells[x, y])

    close_arr = np.zeros((1, cell_w))

    for col in range(cell_col):
        prev = 0        # Previous cell.
        closest = 0     # Minimum z value for an object in the column.
        counter = 0     # Counts the instances in which adjacent cells do not exceed diff.
        min_obj = 0     # Minimum dist to detected object
        for row in range(cell_row):
            min_obj = cells[row, col] if counter = 0 else min(min_obj, cells[row, col])
            if abs(cells[row, col] - prev) < col_diff:
                counter = 0
                min_obj = 0
            else:
                counter += 1
            if prev > cells[row, col] + prev_diff:
                closest = min(cells[row, col], closest)
            if counter > counter_reps:
                closest = min(min_obj, closest)
            prev = cells[row, col]
        close_arr[0, col] = closest

    left_bound = 0      # left most cord of object
    right_bound = 0     # right most cord of object
    prev = 0            # previous cell's z value
    objects = []        # array of detected object tuples (left bound, right bound, distance)

    for col in range(cell_w):
        if close_arr[0, col] != 0:
            if prev == 0:
                left_bound = col
                right_bound = col
                prev = close_arr[0, col]
            elif same_obj_diff > abs(prev - close_arr[0, col]):
                right_bound += 1
                prev = close_arr[0, col]
            else:
                objects.add((left_bound*cell_dim - 10000, (left_bound - right_bound) * cell_dim, close_arr[0, left_bound]))
                prev = 0
        elif prev != 0:
            objects.add((left_bound*cell_dim - 10000, (left_bound - right_bound) * cell_dim, close_arr[0, left_bound]))
            prev = 0
    return objects
