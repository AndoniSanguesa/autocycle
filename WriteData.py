dataFile = None

## INSTRUCTIONS ##

# 1. Each array below is a piece of input data for each obstacle.
# 2. Every array below should be a 2D array. Each subarray is a different frame.
#    Within each frame, the data of each object should be inputted starting from object 1 to object n
# 3. Each frame should have the same number of data from data type to data type
#    (i.e if the first frame in gamma has two pieces of data,
#    the first frame in edge_len should have two pieces of data)
# 4. Run this file and it will create the data in TestData
# 5. Run the bezier_visualization.py file

# You don't need to worry about anything past line 28

# The furthest distance from origin that the path is rendered to
resolution = 10
# The angle to the control point related to the object
gamma = [[35, 15], [35, 15]]
# Distances from the bike to the objects
dist_to_edge = [[4, 6], [4, 6]]
# The distance from the global path to the closest obstacle end-point
edge_to_path = [[1, 4.4], [1, 4.4]]
# A 1 indicates that the the above measurement is made above the global path line
# A -1 indicates that the measurement is made below the global path line
side_small_edge = [[1, 1], [1, 1]]
# Lengths of the objects
edge_len = [[6, 4.5], [6, 4]]

for frame in range(len(gamma)):
    if not (len(gamma[frame]) == len(dist_to_edge[frame]) == len(edge_to_path[frame])
            == len(side_small_edge[frame]) == len(edge_len[frame])):
        print("The number of items in all arrays must be the same!")
        exit()

try:
    dataFile = open("TestData", "w+")
except FileNotFoundError:
    print("The file could not be found, check the name.")
    exit()
except:
    print("It is likely that the file is open somewhere else. Close it and try again.")
    exit()

dataFile.write(str(resolution) + "\n")
for frame in range(len(gamma)):
    dataFile.write(str(len(gamma[frame])) + " ")
    for datum in range(len(gamma[frame])):
        dataFile.write(str(gamma[frame][datum]) + " " +
                       str(dist_to_edge[frame][datum]) + " " +
                       str(edge_to_path[frame][datum]) + " " +
                       str(side_small_edge[frame][datum]) + " " +
                       str(edge_len[frame][datum]) + " ")
    dataFile.write("\n")

print("Data Recorded!")