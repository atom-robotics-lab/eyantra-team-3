import numpy as np
import matplotlib.pyplot as plt
import sys

#Matplot stuff
def make_grid():
    # Define the grid
    plt.figure(figsize=(8, 8))  # Adjust figure size
    plt.xlim(0, 500)  # Set x-axis limit
    plt.ylim(0, 500)  # Set y-axis limit
    plt.gca().set_aspect('equal', adjustable='box')  # Equal aspect ratio
    plt.gca().invert_yaxis()  # Invert y-axis
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Lissajous Curve')
    plt.grid(True)
    plt.gca().set_xticks(np.arange(0, 501, 100))  # Adjust x-axis ticks
    plt.gca().set_yticks(np.arange(0, 501, 100))  # Adjust y-axis ticks
    plt.gca().set_xticks(np.arange(0, 501, 50), minor=True)  # Adjust minor x-axis ticks
    plt.gca().set_yticks(np.arange(0, 501, 50), minor=True)  # Adjust minor y-axis ticks
    plt.grid(which='minor', alpha=0.2)  # Minor grid lines
    plt.grid(which='major', alpha=0.5)  # Major grid lines

# Define xd(t) and yd(t) functions
def xd(t):
    return 200 * np.cos(t)

def yd(t):
    return 150 * np.sin(4*t)

T = 2*np.pi  # 6.28
T1 = T/3     # 2.09

step = 2.1 / 30.0

# Plot function
def plotter(start, end, color):
    for i in np.arange(start, end + step, step):
        x = xd(i)
        y = yd(i)
        norm_x, norm_y = int(x + 250), int(-y + 250)



        print(f"({norm_x}, {norm_y})",end=",")
        sys.stdout.flush()
        plt.scatter(norm_x, norm_y, color=color, s=10)

make_grid()

# Plotting with adjusted coordinates
# plotter(0, T1, "red")
# plotter(T/3, T/3 + T1, "green")
plotter(2*T/3, 2*T/3 + T1, "blue")

plt.show()


red = [(450, 250),(449, 208),(448, 170),(445, 138),(442, 114),(437, 102),(432, 100),(426, 111),(419, 132),(411, 162),(402, 199),(393, 240),(383, 282),(372, 321),(361, 355),(349, 380),(337, 395),(324, 399),(311, 392),(297, 373),(283, 344),(270, 308),(256, 268),(242, 226),(228, 186),(214, 151),(200, 124),(187, 106),(174, 100),(161, 105),(149, 121)]
green = [(150, 120),(138, 145),(126, 179),(116, 219),(106, 260),(96, 301),(88, 338),(80, 368),(73, 389),(67, 399),(61, 397),(57, 384),(54, 360),(51, 328),(50, 289),(50, 248),(50, 206),(52, 168),(54, 137),(57, 114),(62, 101),(67, 101),(73, 111),(80, 133),(88, 164),(97, 201),(106, 242),(116, 284),(127, 323),(139, 356),(150, 381)]
blue = [(149, 379),(162, 395),(175, 399),(188, 392),(201, 374),(215, 345),(229, 310),(243, 270),(257, 228),(271, 188),(285, 152),(298, 124),(312, 106),(325, 100),(338, 104),(350, 120),(362, 147),(373, 181),(384, 220),(394, 262),(403, 303),(412, 340),(420, 369),(426, 390),(433, 399),(438, 397),(442, 383),(445, 359),(448, 326),(449, 288),(449, 246)]

print(len(red),len(green),len(blue))
