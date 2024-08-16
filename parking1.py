import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# initialise values
x_goal = -2
y_goal = -2
phi_goal = -0.4

t = np.linspace(0, 15, 1501)
x = np.zeros(len(t))
y = np.zeros(len(t))
phi = np.zeros(len(t))
x[0] = 0
y[0] = 0
phi[0] = 2.5

theta = 0
alpha = 0
e = 0

gamma = 3
k = 6
h = 1

u = 0
omega = 0

for i in range(1, len(t)):
    e_x = x_goal - x[i-1]
    e_y = y_goal - y[i-1]
    e = np.sqrt(e_x**2 + e_y**2)
    theta = np.arctan2(e_y, e_x) - phi_goal
    alpha = theta - (phi[i-1]-phi_goal)
    u = gamma * e * np.cos(alpha)
    if alpha != 0:
        omega = (gamma * np.sin(alpha) * np.cos(alpha) * (alpha + h*theta)) / alpha + \
            k * alpha 
    else:
        omega = gamma * np.cos(alpha) * h * theta
    dt = t[i] - t[i-1]
    phi[i] = phi[i-1] + omega * (dt)
    phi_avg = (phi[i] + phi[i-1]) / 2
    x[i] = x[i-1] + u * np.cos(phi_avg) * (dt)
    y[i] = y[i-1] + u * np.sin(phi_avg) * (dt)

fig, ax = plt.subplots()
scat = ax.scatter(x_goal, y_goal, c="r", s=20)
line2 = ax.plot(x[0], y[0], label=f'Initial Position: {x[0]}, {y[0]}')[0]
arrow = ax.arrow(x[0], y[0], 0.1*np.cos(phi[0]), 0.1*np.sin(phi[0]), width=0.01)
ax.set(xlim=[-2.5, 2.5], ylim=[-2.5, 2.5], xlabel='x', ylabel='y')
ax.legend()


def update(frame):
    # for each frame, update the data stored on each artist.
    # print(frame)
    x_plot = x[:frame]
    y_plot = y[:frame]
    # # update the scatter plot:
    # data = np.stack([x_plot, y_plot]).T
    # scat.set_offsets(data)
    # update the line plot:
    line2.set_xdata(x_plot[:frame])
    line2.set_ydata(y_plot[:frame])
    # update the arrow:
    if frame != 0:
        phi_plot = phi[frame-1]
        arrow.set_data(x=x_plot[frame-1], y=y_plot[frame-1], dx=0.1*np.cos(phi_plot), dy=0.1*np.sin(phi_plot))
    return scat, line2, arrow


ani = animation.FuncAnimation(fig=fig, func=update, frames=170, interval=20)
plt.show()