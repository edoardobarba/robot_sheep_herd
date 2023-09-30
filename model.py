import numpy as np
import matplotlib

from time import sleep

matplotlib.use('TkAgg')  # Replace 'TkAgg' with the backend of your choice
import matplotlib.pyplot as plt


def polar_angle(vector):
    # Compute the angle in radians using numpy.arctan2
    angle_rad = np.arctan2(vector[1], vector[0])
    # Ensure the angle is in the range [0, 2*pi]
    # angle_rad = np.mod(angle_rad, 2 * np.pi)
    return angle_rad


class HerdModel:
    """
    A model for simulation of the evolution.
    """

    def __init__(self, n_robots: int, n_steps, CR=1000):
        """
        HerdModel init function

        """
        self.robots = np.empty(n_robots, dtype=object)
        self.target = None

        # Target kinemetic model
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
        self.fig, self.ax = plt.subplots()

        self.add_agents(n_robots)
        self.add_target()

        # Initialize the plot with initial positions
        self.plot_positions()

        self.n_steps = n_steps

        self.xStore = np.empty(n_steps, dtype='object')
        self.tStore = np.empty(n_steps, dtype='object')

        self.A_tilda = self.get_A_tilda()



    def add_target(self):
        self.target = Target(At=self.At, Bt=self.Bt)

    def add_agents(self, n_robots):
        """
        Add agents to the model.
        :type n_robots: int
        """

        for i in range(n_robots):
            Ri = 1 * (np.random.rand(2, 2) - 0.5)
            # Calculate the covariance matrix by multiplying the matrix with its transpose
            Ri = np.dot(Ri, Ri.T)

            Hi = (np.random.rand(2, 2) - 0.5)
            while np.linalg.matrix_rank(Hi) < 2:
                print("SONO QUI")
                Hi = (np.random.rand(2, 2) - 0.5)

            # Hi = np.identity(2)
            Qi = 1 * (np.random.rand(2, 2) - 0.5)
            # Calculate the covariance matrix by multiplying the matrix with its transpose
            Qi = np.dot(Qi, Qi.T)

            R_GPS = 1 * (np.random.rand(2, 2) - 0.5)
            # Calculate the covariance matrix by multiplying the matrix with its transpose
            R_GPS = np.dot(R_GPS, R_GPS.T)

            # random_pos = np.random.uniform(-10,10, (2, 1))
            random_x = np.random.uniform(0, )
            random_y = 5 + np.random.uniform(0, 1)
            random_pos = np.array([[random_x], [random_y]])
            robot = Robot_sheep(unique_id=i, pos=random_pos, Ri=Ri, Hi=Hi, Qi=Qi, R_GPS=R_GPS, CR=self.CR)
            # print(robot.pos)
            self.robots[i] = robot
            # self.x_Store[i][:] = robot.pos

    def step(self, t):
        """
        Model step
        """
        self.target.move()
        """
        0-100 GP 
        101-200 CMP
        201-300 GP
        301-400 CMP
        401-500 GP
        501-600 CMP 
        """
        CMP = False

        if (100 <= t <= 200) or (300 <= t <= 400) or (500 <= t <= 600):
            CMP = True

        if t == 100 or t == 300 or t == 500:
            # Assign rank
            # Create an array of unique numbers from 0 to 3
            unique_numbers = np.arange(4)
            # Shuffle the array randomly
            np.random.shuffle(unique_numbers)

            for i, robot in enumerate(self.robots):
                robot.rank = unique_numbers[i]

        # if t == 100:
        #     # random_pos = np.random.uniform(-10,10, (2, 1))
        #     random_x = np.random.uniform(0, 10)
        #     random_y = np.random.uniform(0, 10)
        #     random_pos = np.array([[random_x], [random_y]])
        #     Target.pos = random_pos
        #     print("Target.pos = ", random_pos)

        if CMP:
            # Sort the robots based on their ranks
            sorted_robots = sorted(self.robots, key=lambda robot: robot.rank)

            for robot in sorted_robots:
                robot.move(CMP, self.robots, self.A_tilda)

        else:
            for robot in self.robots:
                robot.move(CMP, self.robots, self.A_tilda)

        self.plot_positions()

        positions = self.get_positions()
        self.xStore[t] = positions
        self.tStore[t] = self.target.pos

        if not CMP:

            for robot in self.robots:
                robot.measure(self.target.pos)

            # Number of consensus protocol msg exchanges:
            m = 5
            for k in range(m):
                A = self.get_topology_matrix()

                for i in range(self.n_robots):
                    # Compute robot neighborhood
                    robot = self.robots[i]
                    robot_neighborhood = []
                    for j in range(self.n_robots):
                        if A[i, j]:
                            robot_neighborhood.append(self.robots[j])

                    D = np.sum(A, axis=1)
                    robot.share(robot_neighborhood, D)

        if t == self.n_steps - 1:
            # print(self.xStore)
            self.print_data()
            self.plot_traj()

    def plot_traj(self):
        # Create a list of colors for each robot's trajectory
        colors = ['r', 'b', 'g', 'k', 'm', 'y', 'c', 'tab:blue', 'tab:orange', 'tab:purple']

        legends = []  # List to store legend labels

        # Create a figure and axis
        fig, ax = plt.subplots()

        for i in range(self.n_robots):
            robot_pos = [positions[i] for positions in self.xStore]
            # Extract x and y coordinates from the points
            x = [point[0] for point in robot_pos]
            y = [point[1] for point in robot_pos]
            # print(robot_pos)
            legends.append(f'Robot {i}')
            plt.plot(x, y, linestyle='-', color=colors[i])

        # Extract x and y coordinates from the points of target
        x = [point[0] for point in self.tStore]
        y = [point[1] for point in self.tStore]

        legends.append('Target ')
        plt.plot(x, y, linestyle='-', color='r')

        # Add labels and legend
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('Robot Trajectories')
        ax.set_xlim(0, 20)  # Set X-axis limits
        ax.set_ylim(0, 20)  # Set Y-axis limits
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

    def print_data(self):
        positions = self.get_positions()
        # Define the reference point
        reference_point = self.target.pos
        # Calculate the distances from the reference point to all robot positions
        distances = np.linalg.norm(positions - reference_point, axis=1)

        # Calculate the mean distance
        mean_distance = np.mean(distances)

        print("Mean Distance from (9, 5):", mean_distance)

    def plot_positions(self):
        self.ax.clear()  # Clear the previous plot
        # colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']  # You can add more colors as needed
        colors = ['r', 'b', 'g', 'k', 'm', 'y', 'c', 'tab:blue', 'tab:orange', 'tab:purple']
        legends = []  # List to store legend labels

        for i, robot in enumerate(self.robots):
            color = colors[i % len(colors)]  # Cycle through colors if there are more robots than colors
            self.ax.plot(robot.pos[0], robot.pos[1], 'o', markersize=5, label=f'Robot {i}', color=color)
            legends.append(f'Robot {i}')

        self.ax.plot(self.target.pos[0], self.target.pos[1], 'x', markersize=5, label='Target', color='r')

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_title('Robot Positions')
        self.ax.set_xlim(0, 20)  # Set X-axis limits
        self.ax.set_ylim(0, 20)  # Set Y-axis limits

        self.ax.legend(legends)  # Add legend using the labels specified above
        plt.pause(0.001)  # Add a small pause (adjust as needed)

    def get_A_tilda(self):
        A_tilda = np.zeros([self.n_robots, self.n_robots])
        if self.n_robots == 4:
            # A_tilda[1, 0] = 5.9597
            # A_tilda[2, 1] = 5.3958
            # A_tilda[3, 2] = 6.2602
            A_tilda[1, 0] = 1
            A_tilda[2, 1] = 1
            A_tilda[3, 2] = 1
            return A_tilda

        if self.n_robots == 2:
            A_tilda[1, 0] = 1

        return A_tilda


class Robot_sheep:
    def __init__(self, unique_id, pos, Ri, Hi, Qi, R_GPS, CR, RobGain=0.01):
        """
        Robot_sheep Agent init function

        :param CR: maximum distance radio of interaction with other agents
        :param Ri: Uncertainty
        :param Qi: Input uncertainty
        :param Hi: Sensor model
        """
        self.unique_id = unique_id
        self.CR = CR

        self.Ri = Ri
        self.Hi = Hi
        self.Qi = Qi
        self.R_GPS = R_GPS
        self.RobGain = RobGain

        self.Fi = None
        self.ai = None

        self.FiStore = None
        self.aiStore = None

        self.pos = pos

        # Suppose we know exactly the initial position
        self.x_est = pos
        self.P_est = np.zeros((2, 2))
        self.p_est_distr = np.zeros((2, 1))
        self.rank = None
        self.u = np.array([[0], [0]])

    # def move(self):
    #     u = self.RobGain * (self.p_est_distr - self.x_est)
    #     u = np.clip(u, -0.5, 0.5)
    #     self.pos = self.pos + u
    #     self.pos_kf(u)

    def move(self, CMP, robots, A_tilda):
        if CMP:
            theta = polar_angle(self.u)
            v = 0.05
            if self.rank == 0:
                u = self.RobGain * (self.p_est_distr - self.x_est)
                self.u = np.clip(u, -0.5, 0.5)
                self.pos = self.pos + self.u
                self.pos_kf(u)
                return

            counter = 0
            for robot in robots:
                if robot == self:
                    continue

                # diff = self.x_est - robot.x_est
                diff = self.pos - robot.pos

                diff_norm = np.linalg.norm(diff)

                e_ij = diff / diff_norm

                # Compute the angle in polar coordinates
                aij = polar_angle(e_ij)
                gij = -np.sin(aij - polar_angle(self.u))
                prova1 = polar_angle(robot.u) - polar_angle(self.u)
                influence = A_tilda[self.rank, robot.rank]

                # gij = np.sin(polar_angle(robot.u) - polar_angle(self.u))

                if gij != 0 and influence != 0:
                    if diff_norm > 0.3:
                        v = 1
                    counter += 1

                theta += A_tilda[self.rank, robot.rank] * gij

            if counter != 0:
                # new_velocity = np.array([np.cos(theta) * self.pos[0], np.sin(theta) * self.pos[1]])
                new_velocity = v * np.array([np.cos(theta), np.sin(theta)])
                self.u = np.clip(new_velocity, -0.5, 0.5)
                dt = 0.1
                self.pos += self.u * dt

            self.pos_kf(self.u)

        else:  # GP
            mean = 0
            std_dev = 0.1
            u_random_vector = np.random.normal(mean, std_dev, size=(2, 1))
            self.u = u_random_vector
            dt = 0.1
            self.pos += self.u * dt

            self.pos_kf(self.u)

    def measure(self, target_pos):
        s_World = self.get_target_pos(target_pos)
        self.measure_target_pos(s_World)

    def share(self, robot_neighborhood, D):
        # for j in range(len(robot_neighborhood)):
        #     self.Fi = self.Fi + 1 / ((1 + max(D)) * (robot_neighborhood[j].FiStore - self.FiStore))
        #     self.ai = self.ai + 1 / ((1 + max(D)) * (robot_neighborhood[j].aiStore - self.aiStore))

        self.FiStore = self.Fi
        self.aiStore = self.ai
        for j in range(len(robot_neighborhood)):
            self.Fi = self.Fi + (1 / (1 + max(D)) * (robot_neighborhood[j].FiStore - self.FiStore))
            self.ai = self.ai + (1 / (1 + max(D)) * (robot_neighborhood[j].aiStore - self.aiStore))

        self.update_target_est()
        return

    def get_target_pos(self, target_pos):
        # Target in robot reference frame
        t_pos_robot_frame = target_pos - self.pos

        # Sensor (measure of the position + uncertainty(0 mean, Ri covariance))

        # prova = self.Hi @ t_pos_robot_frame
        # prova2 = np.random.multivariate_normal([0, 0], self.Ri).reshape((-1, 1))
        s = self.Hi @ t_pos_robot_frame + np.random.multivariate_normal([0, 0], self.Ri).reshape((-1, 1))

        # Transform the robot measurements into world measurements (the common reference frame for all the robots)
        s_World = s + self.Hi @ self.x_est
        return s_World

    def measure_target_pos(self, s_World):
        # Consensus
        zi = s_World
        # prova0 = self.Hi.T
        # prova1 = self.Hi @ self.P_est @ self.Hi.T
        # prova2 = np.linalg.inv(self.Ri + self.Hi @ self.P_est @ self.Hi.T)
        # prova3 = self.Hi
        self.Fi = self.Hi.T @ np.linalg.inv(self.Ri + self.Hi @ self.P_est @ self.Hi.T) @ self.Hi
        self.ai = self.Hi.T @ np.linalg.inv(self.Ri + self.Hi @ self.P_est @ self.Hi.T) @ zi
        self.FiStore = self.Fi
        self.aiStore = self.ai

    def update_target_est(self):
        self.p_est_distr = np.linalg.inv(self.Fi) @ self.ai

        # if np.any(self.p_est_distr < -100) or np.any(self.p_est_distr >200) :
        #     print(self.unique_id)
        #     print(self.p_est_distr)
        #
        #     sleep(2)

        return

    def pos_kf(self, u):
        # Update estimated position
        random_vector = np.random.multivariate_normal([0, 0], self.Qi).reshape((-1, 1))
        uUnc = u + random_vector
        uUnc = np.clip(uUnc, -0.5, 0.5)
        self.x_est = self.x_est + uUnc
        self.P_est = self.P_est + self.Qi

        # Measurements update
        # assume we have a gps that gives to each robot its own locations

        z_GPS = self.pos + np.random.multivariate_normal([0, 0], self.R_GPS).reshape((-1, 1))

        Innovation = z_GPS - self.x_est  # difference between measurement minus the predicted location of robot
        H_GPS = np.identity(2)

        # Updated Kalman estimates

        S_Inno = H_GPS @ self.P_est @ H_GPS.T + self.R_GPS  # covariance matrix of innovation

        W = self.P_est @ H_GPS.T @ np.linalg.inv(S_Inno)  # KF gain

        prova = W @ Innovation

        self.x_est = self.x_est + W @ Innovation  # updated state estimate

        self.P_est = (np.identity(2) - W @ H_GPS) @ self.P_est  # updated covariance matrix
        return


class Target:
    def __init__(self, At, Bt, pos=np.array([[1.], [5.]])):
        self.pos = pos
        self.u = np.array([[0.3], [0.1]])
        self.At = At
        self.Bt = Bt

    def move(self):
        dt = 0.1
        # Human being dynamics
        ProbChangeDir = 0.01
        if np.random.rand() < ProbChangeDir:
            uHuman = (np.random.rand(2) - 0.5) * 0.3
            uHuman = uHuman.reshape(-1, 1)
            print(self.pos)
            print("uHuman ", uHuman)
            # uHuman will be a NumPy array containing 2 random values between -0.15 and 0.15

            self.u += uHuman
        self.pos += self.u * dt
