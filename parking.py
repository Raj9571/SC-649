import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Initialise values
x_goal = 0
y_goal = 0
phi_goal = 3.14

# Define initial positions
initial_positions = [[-2, -2], [0, -2], [2, -2]]

t = np.linspace(0, 15, 1501)
x = np.zeros((len(initial_positions), len(t)))
y = np.zeros((len(initial_positions), len(t)))
phi = np.zeros((len(initial_positions), len(t)))

theta = 0
alpha = 0
e = 0

gamma = 3
k = 6
h = 1

u = 0
omega = 0


fig, ax = plt.subplots()
scat = ax.scatter(x_goal, y_goal, c="r", s=20)
lines = []
arrows = []

for i, initial_position in enumerate(initial_positions):
    x[i][0] = initial_position[0]
    y[i][0] = initial_position[1]
    phi[i][0] = 0.5

    line, = ax.plot([], [], label=f'Initial Position: {x[i][0]}, {y[i][0]}')
    arrow = ax.arrow(x[i][0], y[i][0], 0.1*np.cos(phi[i][0]), 0.1*np.sin(phi[i][0]), width=0.01)

    lines.append(line)
    arrows.append(arrow)

ax.set(xlim=[-2.5, 2.5], ylim=[-2.5, 2.5], xlabel='x', ylabel='y')
ax.legend()


def update(frame):
    for i in range(len(initial_positions)):
        x_plot = x[i][:frame+1]
        y_plot = y[i][:frame+1]

        lines[i].set_xdata(x_plot)
        lines[i].set_ydata(y_plot)

        if frame != 0:
            phi_plot = phi[i][frame]
            arrows[i].set_data(x=x_plot[frame], y=y_plot[frame], dx=0.1*np.cos(phi_plot), dy=0.1*np.sin(phi_plot))
    return [scat] + lines + arrows


ani = animation.FuncAnimation(fig=fig, func=update, frames=len(t), interval=20)
plt.show()
