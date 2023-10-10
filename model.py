import numpy as np
import matplotlib
from RobotSheep import *

from time import sleep

matplotlib.use('TkAgg')  # Replace 'TkAgg' with the backend of your choice
import matplotlib.pyplot as plt


class HerdModel:
    """
    A model for simulation of the evolution.
    """

    def __init__(self, n_robots: int, n_steps, fixed_target=True, CR=1000):
        """
        HerdModel init function

        """
        self.robots = np.empty(n_robots, dtype=object)
        self.target = None
        self.fixed_target = fixed_target

        # Target kinematic model
        # Define Ah as a 2x2 identity matrix
        self.At = np.eye(2)
        # Define Bh as a 2x2 identity matrix (assuming you have 2 inputs)
        self.Bt = np.eye(2)

        self.num_agents = n_robots
        self.current_id = 0
        self.CR = CR
        # self.x_Store = [np.empty((2)) for _ in range(n_robots)]

        self.n_robots = n_robots

        # Create a figure and axis for the plot
        #self.fig, self.ax = plt.subplots(figsize=(10, 5))
        self.Store = np.empty(n_steps, dtype='object')
        self.add_agents(n_robots)
        self.add_target()

        # Initialize the plot with initial positions
        # self.plot_positions()

        self.n_steps = n_steps
        self.xStore = np.empty(n_steps, dtype='object')
        self.tStore = np.empty((n_steps, 2, 1))
        self.uStore = np.empty(n_steps, dtype='object')

        self.A_tilda = self.get_A_tilda()
        self.CMP = False
        #print("TARGET POS: ", self.target.pos)

    def reset_robots(self):
        if self.fixed_target:
            for robot in self.robots:
                robot.Fi = None
                robot.ai = None

                robot.FiStore = None
                robot.aiStore = None
                robot.p_est_distr = np.zeros((2, 1))
                return

        for robot in self.robots:
            robot.Fi = None
            robot.ai = None

            robot.FiStore = None
            robot.aiStore = None



    def add_target(self):
        self.target = Target(At=self.At, Bt=self.Bt, fixed=self.fixed_target)

    def add_agents(self, n_robots):
        """
        Add agents to the model.
        :type n_robots: int
        """

        for i in range(n_robots):
            # Sensor Covariance
            Ri = 1 * (np.random.rand(2, 2) - 0.5)
            # Calculate the covariance matrix by multiplying the matrix with its transpose
            Ri = np.dot(Ri, Ri.T)

            # Sensor model
            Hi = (np.random.rand(2, 2) - 0.5)
            while np.linalg.matrix_rank(Hi) < 2:
                # print("SONO QUI")
                Hi = (np.random.rand(2, 2) - 0.5)

            # Hi = np.identity(2)
            # Input uncertainty
            Qi = 1 * (np.random.rand(2, 2) - 0.5)
            # Calculate the covariance matrix by multiplying the matrix with its transpose
            Qi = np.dot(Qi, Qi.T)

            # GPS uncertainty
            scale_GPS = 1
            R_GPS = scale_GPS * (np.random.rand(2, 2) - 0.5)
            # Calculate the covariance matrix by multiplying the matrix with its transpose
            R_GPS = np.dot(R_GPS, R_GPS.T)

            # random_pos = np.random.uniform(-10,10, (2, 1))
            random_x = 2 + np.random.uniform(0, 3)
            random_y = 2 + np.random.uniform(0, 3)
            random_pos = np.array([[random_x], [random_y]])
            robot = Robot_sheep(unique_id=i, pos=random_pos, Ri=Ri, Hi=Hi, Qi=Qi, R_GPS=R_GPS, CR=self.CR, At=self.At)

            # print(robot.pos)
            self.robots[i] = robot
            # self.x_Store[i][:] = robot.pos

    def step(self, t):
        """
        Model step
        """
        if not self.target.fixed:
            self.target.move(self.CMP)
        """
        0-100 GP 
        101-200 CMP
        201-300 GP
        301-400 CMP
        401-500 GP
        501-600 CMP 
        """
        positions = self.get_positions()
        velocities = self.get_velocities()
        self.xStore[t] = positions
        self.uStore[t] = velocities
        self.tStore[t] = self.target.pos
        # self.CMP = False
        # if (100 <= t <= 200) or (300 <= t <= 400) or (500 <= t <= 600):
        #     self.CMP = True

        if t == 0:  # or t == 100 or t == 300 or t == 500:
            # Assign rank
            # Create an array of unique numbers from 0 to 3
            unique_numbers = np.arange(self.n_robots)
            # Shuffle the array randomly
            np.random.shuffle(unique_numbers)

            for i, robot in enumerate(self.robots):
                robot.rank = unique_numbers[i]

        # Sort the robots based on their ranks
        sorted_robots = sorted(self.robots, key=lambda robot: robot.rank)
        #if self.CMP:
            #print("Target pos: ")
            #print(self.target.pos)
        for robot in sorted_robots:
            change = robot.move(self.CMP, self.robots, self.A_tilda)
            if change:
                if self.CMP:
                    self.CMP = False
                    self.reset_robots()
                    return
                self.CMP = True
                self.assign_rank(leader=robot)
                return

        if not self.CMP or self.CMP:  # They communicate only during GP
            if not self.fixed_target:
                for robot in self.robots:
                    robot.predict()

            for robot in self.robots:
                robot.measure(self.target.pos)

            # Number of consensus protocol msg exchanges:
            m = 5
            for k in range(m):
                A = self.get_topology_matrix()

                for i in range(self.n_robots):

                    # Compute robot neighborhood
                    robot = self.robots[i]
                    #self.plot_positions()
                    robot_neighborhood = []
                    for j in range(self.n_robots):
                        if A[i, j]:
                            robot_neighborhood.append(self.robots[j])

                    D = np.sum(A, axis=1)
                    #print("t= ", t)
                    #if(k==m-1):
                    #    print(np.linalg.norm(robot.p_est_distr-self.target.pos))

                    robot.share(robot_neighborhood, D)


        if t == self.n_steps - 1:
            # print(self.xStore)
            self.print_data()
            self.plot_history_pos()
            self.plot_traj()

    def plot_traj(self):
        # Create a list of colors for each robot's trajectory
        colors = ['c', 'tab:orange', 'g', 'y']

        legends = []  # List to store legend labels

        # Create a figure and axis
        fig, ax = plt.subplots(figsize=(10, 5))

        for i in range(self.n_robots):
            robot_pos = [positions[i] for positions in self.xStore]
            # Extract x and y coordinates from the points
            x = [point[0] for point in robot_pos]
            y = [point[1] for point in robot_pos]
            # print(robot_pos)
            legends.append(f'Robot {i}')
            plt.plot(x, y, linestyle='-', color=colors[i])

        # Extract x and y coordinates from the points of target

        if self.fixed_target:
            x = [point[0] for point in np.unique(self.tStore, axis=0)]
            y = [point[1] for point in np.unique(self.tStore, axis=0)]
            # legends.append('Target ')
            plt.scatter(x, y, s=2500, marker='o', facecolors='none', edgecolors='r')  # , label='Target')
            # plt.plot(x, y, 'o',  markersize=10, color='r', label='Target')
        else:
            x = [point[0] for point in self.tStore]
            y = [point[1] for point in self.tStore]
            legends.append('Target ')
            plt.plot(x, y, linestyle='-', color='r', label='Target')

        # Add labels and legend
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('Robot Trajectories')
        ax.set_xlim(0, 120)  # Set X-axis limits
        ax.set_ylim(0, 60)  # Set Y-axis limits
        ax.legend(legends)  # Add legend using the labels specified above
        # Show the plot
        plt.show()

    def get_topology_matrix(self):
        # Topology matrix
        A = np.zeros([self.n_robots, self.n_robots])
        for i in range(self.n_robots):
            for j in range(i + 1, self.n_robots):  # (it's simmetric so we compute only the upper part)
                d = np.linalg.norm(self.robots[i].pos - self.robots[j].pos)
                A[i, j] = (d <= self.CR)
        A = A + A.T

        return A

    def get_positions(self):
        return np.array([robot.pos for robot in self.robots])

    def get_velocities(self):
        return np.array([robot.u for robot in self.robots])

    def print_data(self):
        positions = self.get_positions()
        # Define the reference point
        reference_point = self.target.pos
        # Calculate the distances from the reference point to all robot positions
        distances = np.linalg.norm(positions - reference_point, axis=1)

        # Calculate the mean distance
        mean_distance = np.mean(distances)

        print("Mean Distance from (9, 5):", mean_distance)

    def plot_history_pos(self):

        # colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']  # You can add more colors as needed
        colors = ['c', 'tab:orange', 'g', 'y']
        legends = []  # List to store legend labels

        # Create a figure and axis
        fig, ax = plt.subplots(figsize=(10, 5))

        for i in range(self.n_steps):
            ax.clear()
            for j, robot in enumerate(self.robots):
                color = colors[j%len(colors)]
                robot_pos = self.xStore[i][j]
                x = robot_pos[0]
                y = robot_pos[1]
                plt.plot(x, y, 'o', markersize=5, label=f'Robot {i}', color=color)
                #plt.quiver(x, y, self.uStore[i][j][0], self.uStore[i][j][1], angles='xy', scale_units='xy', scale=0.4,
                           #color=color, label='Velocity Vector', headlength=1,  # Length of the arrowhead
                           #headwidth=1)
                # Add labels and legend

            #plt.plot(self.tStore[i][0], self.tStore[i][1], 'o', markersize=10, label='Target', color='r')
            plt.scatter(self.tStore[i][0], self.tStore[i][1], s=2500, label='Target', marker='o', facecolors='none',
                            edgecolors='r')
            ax.set_xlabel('X[m]')
            ax.set_ylabel('Y[m]')

            ax.set_xlim(0, 120)  # Set X-axis limits
            ax.set_ylim(0, 60)  # Set Y-axis limits

            title = f'Robot Positions  |  t = {i}'
            ax.set_title(title)
            ax.legend(legends)  # Add legend using the labels specified above
            plt.pause(0.001)

        # # Show the plot
        # plt.show()

    def plot_positions(self):
        self.ax.clear()  # Clear the previous plot
        # colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']  # You can add more colors as needed
        colors = ['c', 'tab:orange', 'g', 'y']
        legends = []  # List to store legend labels
        self.ax.set_xlim(0, 120)  # Set X-axis limits
        self.ax.set_ylim(0, 60)  # Set Y-axis limits

        for i, robot in enumerate(self.robots):
            color = colors[i % len(colors)]  # Cycle through colors if there are more robots than colors
            self.ax.plot(robot.pos[0], robot.pos[1], 'o', markersize=5, label=f'Robot {i}', color=color)
            self.ax.plot(robot.p_est_distr[0], robot.p_est_distr[1], 'x', markersize=5, color=color)
            legends.append(f'Robot {i}')

        # self.ax.plot(self.target.pos[0], self.target.pos[1], linestyle='o', markersize=10, label='Target', color='r')
        self.ax.scatter(self.target.pos[0], self.target.pos[1], s=1000, marker='o', facecolors='none', edgecolors='r')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_title('Robot Positions')


        self.ax.legend(legends)  # Add legend using the labels specified above
        plt.pause(0.01)  # Add a small pause (adjust as needed)

    def get_A_tilda(self):
        A_tilda = np.zeros([self.n_robots, self.n_robots])
        for i in range(self.n_robots-1):
            A_tilda[i+1, i] = 1

        # A_tilda[2,0] = 1
        # A_tilda[3,0] = 1
        # if self.n_robots == 4:
        #     # A_tilda[1, 0] = 5.9597
        #     # A_tilda[2, 1] = 5.3958
        #     # A_tilda[3, 2] = 6.2602
        #     A_tilda[1, 0] = 1
        #     A_tilda[2, 1] = 1
        #     A_tilda[3, 2] = 1
        #     return A_tilda
        #
        # if self.n_robots == 2:
        #     A_tilda[1, 0] = 1

        return A_tilda

    # def assign_rank(self, leader):
    #     # Assign rank
    #     # Create an array of unique numbers from 0 to 3
    #     unique_numbers = np.arange(1, self.n_robots)
    #
    #     # Shuffle the array randomly
    #     np.random.shuffle(unique_numbers)
    #
    #     for i, robot in enumerate(self.robots):
    #         if robot.unique_id == leader:
    #             robot.rank = 0
    #         else:
    #             robot.rank, unique_numbers = unique_numbers[-1], unique_numbers[:-1]

    def assign_rank(self, leader):
        # Assign rank
        # Create an array of unique numbers from 0 to n_robots-1
        unique_numbers = np.arange(self.n_robots - 1, -1, -1)
        leader_pos = leader.pos

        # Sort robots by distance to the leader
        self.robots = sorted(self.robots, key=lambda robot: np.linalg.norm(robot.pos - leader_pos))

        # Assign ranks
        for i, robot in enumerate(self.robots):
            robot.rank = unique_numbers[i]



class Target:
    def __init__(self, At=np.eye(2), Bt=np.eye(2), initial_pos=np.array([[55.], [30.]]), fixed=True):
        self.pos = initial_pos
        self.u = np.array([[0.4], [0.]])
        self.At = At
        self.Bt = Bt
        self.fixed = fixed
        self.changed_pos = False

    def move(self, CMP):
        dt = 1
        if self.fixed:
            if not CMP and not self.changed_pos:
                # Randomly assign a number from 0 to 50 to y
                y = np.random.randint(5, 55)
                # Randomly assign a number from 0 to 100 to x
                x = np.random.randint(5, 115)
                self.pos[0] = x
                self.pos[1] = y
                self.changed_pos = True
                return False

            if CMP and self.changed_pos:
                self.changed_pos = False
                return False

            return False

        # else (not fixed) :

        # Human being dynamics
        ProbChangeDir = 0.1
        #if self.pos[0] > 95 or self.pos[0] < 5 or self.pos[1] > 45 or self.pos[1] < 5:
            #ProbChangeDir = 0.3

        if np.random.rand() < ProbChangeDir:
            uTarget = (np.random.rand(2) - 0.5) * 2
            uTarget = uTarget.reshape(-1, 1)
            # uHuman will be a NumPy array containing 2 random values between -0.15 and 0.15

            self.u += uTarget
            self.u = np.clip(self.u, -1, 1)
        self.pos += self.u * dt

        # Enforce boundaries
        self.pos[0] = np.clip(self.pos[0], 10, 100)  # Bound x within 0-100
        self.pos[1] = np.clip(self.pos[1], 10, 50)
