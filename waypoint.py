import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches

# initialise values
t = np.linspace(0, 30, 2001)
x_goal = [0, 1, 1, 0, -1]
y_goal = [0, 0.2, 1, 1.5, 0.6]
phi_goal = np.zeros(len(x_goal))

for i in range(1, len(x_goal)):
    phi_goal[i] = np.arctan2(y_goal[i]-y_goal[i-1], x_goal[i]-x_goal[i-1])
goal_index = 0

x = np.zeros(len(t))
y = np.zeros(len(t))
phi = np.zeros(len(t))
x[0] = -2
y[0] = 0
phi[0] = 0
phi_goal[0]=np.arctan2(y_goal[0]-y[0],x_goal[0]-x[0])
theta = 0
alpha = 0
e = np.sqrt((x_goal[0] - x[0])**2 + (y_goal[0] - y[0])**2)

A = 1
dsdt = 0
V = 0

gamma = 3
k = 4
h = 1
lambda_ = 1
epsilon = 0.02

u = 0
omega = 0

for i in range(1, len(t)):
    V = 1/2 * (lambda_*e*e + alpha*alpha + h*theta*theta)
    if V < epsilon:
        goal_index += 1
        print("goal reached")
        if goal_index >= len(x_goal):
            break
    e_x = x_goal[goal_index] - x[i-1]
    e_y = y_goal[goal_index] - y[i-1]
    e = np.sqrt(e_x**2 + e_y**2)
    theta = np.arctan2(e_y, e_x) - phi_goal[goal_index]
    alpha = theta - phi[i-1] + phi_goal[goal_index]
    u = gamma * e * np.cos(alpha)
    if alpha != 0:
        omega = (gamma * np.sin(alpha) * np.cos(alpha) * (alpha + h*theta)) / alpha + \
            k * alpha 
    else:
        omega = gamma * np.cos(alpha) * h * theta
    print("alpha",alpha,"theta:",theta,"phi:",phi[i-1],"phi_goal:",phi_goal[goal_index])
    dt = t[i] - t[i-1]
    phi[i] = (phi[i-1] + omega * (dt))
    phi_avg = (phi[i] + phi[i-1]) / 2
    x[i] = x[i-1] + u * np.cos(phi_avg) * (dt)
    y[i] = y[i-1] + u * np.sin(phi_avg) * (dt)

fig, ax = plt.subplots()
scat = ax.scatter(x_goal, y_goal, c="r", s=1, facecolors='none')
line2 = ax.plot(x[0], y[0], label=f'Initial Position: {x[0]}, {y[0]}')[0]
arrow = ax.arrow(x[0], y[0], 0.1*np.cos(phi[0]), 0.1*np.sin(phi[0]), width=0.01)
circles = [patches.Circle((x_goal[i], y_goal[i]), np.sqrt(epsilon/lambda_), color='r', fill=False) for i in range(len(x_goal))]
bounds = [ax.add_patch(circle) for circle in circles]
ax.set(xlim=[-5, 5], ylim=[-5, 5], xlabel='x', ylabel='y')
ax.legend()


def update(frame):
    # for each frame, update the data stored on each artist.
    # print(frame)
    x_plot = x[:frame]
    y_plot = y[:frame]
    # update the line plot:
    line2.set_xdata(x_plot[:frame])
    line2.set_ydata(y_plot[:frame])
    # update the scatter plot:
    data = np.stack([x_goal[:frame], y_goal[:frame]]).T
    scat.set_offsets(data)
    if frame != 0:
        # update the arrow
        phi_plot = phi[frame-1]
        arrow.set_data(x=x_plot[frame-1], y=y_plot[frame-1], dx=0.1*np.cos(phi_plot), dy=0.1*np.sin(phi_plot))
        # # update the circle:
        # bound.set_center((x_goal[frame], y_goal[frame]))
    return scat, line2, arrow, bounds


ani = animation.FuncAnimation(fig=fig, func=update, frames=2000, interval=20)
plt.show()