import matplotlib.pyplot as plt

bruh = open("bet.txt", "r")
lst = eval(bruh.read())

a = [(ind_x, ind_y, lst[ind_y][ind_x]) for ind_y in range(len(lst)) for ind_x in range(len(lst[ind_y]))]
xs = []
ys = []
zs = []
for x, y, z in a:
    if z != 0 and z < 8000:
        xs.append(x*50 - 10000)
        ys.append(y*50 - 5000)
        zs.append(z)

fig = plt.figure()
ax = fig.add_subplot()
ax.scatter(xs, zs, c=ys)
plt.show()