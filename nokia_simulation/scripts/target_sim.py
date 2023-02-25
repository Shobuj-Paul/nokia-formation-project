# importing numpy to work with arrays
import numpy as np

# importing matplotlib to plot the animation
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import random

class Drone:
    def __init__(self, T) -> None:
        # define the agent (x, y, z) position
        rand = lambda window : [window*random.random(), window*random.random(), window*random.random()]
        self.r = np.asarray(rand(10))
        # set the agent linear velocity
        self.s = 0.0025
        # define a target (x, y, z) position
        self.T = T 
        
        # First set up the figure, the axis, and the plot element we want to animate
        self.fig, ax = plt.subplots()

        ax.set_xlim([-2,10])
        ax.set_ylim([-2,10])
        ax.set_xlabel('X(m)')
        ax.set_ylabel('Y(m)')
        ax.grid()
        ax.set_aspect('equal')
        ax.set_title('Target behavior')

        ax.plot(T[0], T[1], marker='x', color='red', lw=0)
        self.agent, = ax.plot([], [], marker='o', lw=0)

    # initialization function: plot the background of each frame
    def init(self):
        self.agent.set_data([], [])
        return (self.agent,)
    
    # animation function. This is called sequentially
    def animate(self, i):
        self.r += self.s*(self.T-self.r)
        self.agent.set_data(self.r[0], self.r[1])
        return (self.agent,)

def main():
    drone = Drone(np.asarray([8., 8., 0.]))

    # call the animator. blit=True means only re-draw the parts that
    # have changed.
    anm = animation.FuncAnimation(drone.fig, drone.animate, init_func=drone.init,
                                   frames=480, interval=1, blit=True)
    plt.show()


if __name__=='__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass