import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches

# initialise values
t = np.linspace(0, 30, 2001)
x_goal = np.zeros(len(t))
y_goal = np.zeros(len(t))
phi_goal = np.zeros(len(t))

x = np.zeros(len(t))
y = np.zeros(len(t))
phi = np.zeros(len(t))
x[0] = -2
y[0] = 0
phi[0] = 0

theta = 0
alpha = 0
e = np.sqrt((x_goal[0] - x[0])**2 + (y_goal[0] - y[0])**2)

A = 1
dsdt = 0
V = 0

gamma = 3
k = 6
h = 1
lambda_ = 1
epsilon = 0.02

u = 0
omega = 0

for i in range(1, len(t)):
    V1 = 1/2 * (lambda_*e*e)
    V2 = 1/2 * (alpha*alpha + h*theta*theta)
    V = V1 + V2
    dsdt = max(0, 1 - V/epsilon)
    print("i: ", i)
    print("V1: ", V1)
    print("V2: ", V2)
    dt = t[i] - t[i-1]
    ds = dsdt * dt
    beta = np.arctan2(np.cos(x_goal[i-1]), 1) # trajectory slope
    x_goal[i] = x_goal[i-1] + ds * np.cos(beta)
    y_goal[i] = y_goal[i-1] + ds * np.sin(beta)
    phi_goal[i] = beta
    x_goal_avg = (x_goal[i] + x_goal[i-1]) / 2
    y_goal_avg = (y_goal[i] + y_goal[i-1]) / 2
    phi_goal_avg = (phi_goal[i] + phi_goal[i-1]) / 2

    e_x = x_goal_avg - x[i-1]
    e_y = y_goal_avg - y[i-1]
    e = np.sqrt(e_x**2 + e_y**2)
    theta = np.arctan2(e_y, e_x) - phi_goal_avg
    alpha = theta - phi[i-1] + phi_goal_avg
    u = gamma * e * np.cos(alpha)
    if alpha != 0:
        omega = (gamma * np.sin(alpha) * np.cos(alpha) * (alpha + h*theta)) / alpha + \
            k * alpha 
    else:
        omega = gamma * np.cos(alpha) * h * theta

    phi[i] = phi[i-1] + omega * (dt)
    phi_avg = (phi[i] + phi[i-1]) / 2
    x[i] = x[i-1] + u * np.cos(phi_avg) * (dt)
    y[i] = y[i-1] + u * np.sin(phi_avg) * (dt)

fig, ax = plt.subplots()
scat = ax.scatter(x_goal, y_goal, c="r", s=1, facecolors='none')
line2 = ax.plot(x[0], y[0], label=f'Initial Position: {x[0]}, {y[0]}')[0]
arrow = ax.arrow(x[0], y[0], 0.1*np.cos(phi[0]), 0.1*np.sin(phi[0]), width=0.01)
circle = patches.Circle((x_goal[0], y_goal[0]), np.sqrt(epsilon/lambda_), color='r', fill=False)
bound = ax.add_patch(circle)
ax.set(xlim=[-10, 10], ylim=[-10, 10], xlabel='x', ylabel='y')
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
        # update the circle:
        bound.set_center((x_goal[frame], y_goal[frame]))
    return scat, line2, arrow, bound


ani = animation.FuncAnimation(fig=fig, func=update, frames=2000, interval=1)
ani.save('animation.gif', writer='pillow')
plt.show()