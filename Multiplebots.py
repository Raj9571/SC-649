import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Initialize values for 8 robots
# initial_conditions = [
#     (-2, 0, 0),
#     (-np.sqrt(2), np.sqrt(2), 0),
#     (-np.sqrt(2), -np.sqrt(2), 0),
#     (0, 2, 0),
#     (0, -2, 0),
#     (1.5, -0.3, 0),
#     (1.5, 0.3, 0)
# ]

initial_conditions = [[-1.414,1.414,0],[0,2,0],[1.414,1.414,0],[2,0,0],[1.414,-1.414,0],[0,-2,0],[-1.414,-1.414,0],[-2,0,0]]

t = np.linspace(0, 30, 2001)

# Create arrays to store the states of each robot
x = np.zeros((len(initial_conditions), len(t)))
y = np.zeros((len(initial_conditions), len(t)))
phi = np.zeros((len(initial_conditions), len(t)))

# Set initial conditions for each robot
for i, (x0, y0, phi0) in enumerate(initial_conditions):
    x[i, 0] = x0
    y[i, 0] = y0
    phi[i, 0] = phi0

x_goal = 0
y_goal = 0
phi_goal = 0

theta = 0
alpha = 0
e = 0

gamma = 3
k = 4
h = 1

u = 0
omega = 0

# Move the update_robot_data segment outside the update function
for i in range(1, len(t)):
    for j in range(len(initial_conditions)):
        e_x = x_goal - x[j, i - 1]
        e_y = y_goal - y[j, i - 1]
        e = np.sqrt(e_x**2 + e_y**2)
        theta = np.arctan2(e_y, e_x) - phi_goal
        alpha = theta - phi[j, i - 1] + phi_goal
        u = gamma * e * np.cos(alpha)
        if alpha != 0:
            omega = (gamma * np.sin(alpha) * np.cos(alpha) * (alpha + h * theta)) / alpha + k * alpha
        else:
            omega = gamma * np.cos(alpha) * h * theta
        dt = t[i] - t[i - 1]
        phi[j, i] = phi[j, i - 1] + omega * (dt)
        phi_avg = (phi[j, i] + phi[j, i - 1]) / 2
        x[j, i] = x[j, i - 1] + u * np.cos(phi_avg) * (dt)
        y[j, i] = y[j, i - 1] + u * np.sin(phi_avg) * (dt)

fig, ax = plt.subplots()
scat = ax.scatter([], [], c="r", s=20)

# Create separate line artists for each robot
lines = [ax.plot(x[k, 0], y[k, 0], label=f'Robot {k}')[0] for k in range(len(initial_conditions))]
arrows = [ax.arrow(x[k, 0], y[k, 0], 0.1 * np.cos(phi[k, 0]), 0.1 * np.sin(phi[k, 0]), width=0.01) for k in range(len(initial_conditions))]

ax.set(xlim=[-3, 3], ylim=[-3, 3], xlabel='x', ylabel='y')
ax.legend()


def update(frame):
    # Update the data for each robot
    for i in range(len(initial_conditions)):
        # Update the line plot for each robot
        lines[i].set_xdata(x[i, :frame])
        lines[i].set_ydata(y[i, :frame])

        # Update the arrow for each robot
        if frame != 0:
            phi_plot = phi[i, frame - 1]
            arrows[i].set_data(x=x[i, frame - 1], y=y[i, frame - 1],
                               dx=0.1 * np.cos(phi_plot), dy=0.1 * np.sin(phi_plot))

    # Update the scatter plot
    # data = np.vstack([x[:, frame], y[:, frame]]).T
    # scat.set_offsets(data)

    return scat, *lines, *arrows


ani = animation.FuncAnimation(fig=fig, func=update, frames=len(t), interval=20)
ani.save('animation.gif', writer='pillow')
plt.show()