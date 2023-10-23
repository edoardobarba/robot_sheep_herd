import numpy as np
import random


def polar_angle(vector):
    # Compute the angle in radians using numpy.arctan2
    angle_rad = np.arctan2(vector[1], vector[0])
    # Ensure the angle is in the range [0, 2*pi]
    angle_rad = np.mod(angle_rad, 2 * np.pi)
    return angle_rad


class RobotSheep:
    def __init__(self, unique_id, pos, Ri, Hi, Qi, R_GPS, At, Bt, fixed_target, n_robots, RobGain=0.1, target_radius = 5):
        """
        Initialize a RobotSheep agent.

        Parameters:
            - unique_id (int): A unique identifier for the robot.
            - pos (tuple): Initial position (x, y) of the robot.
            - Ri (float): Uncertainty associated with robot position measurements.
            - Hi (function): Sensor model for the robot's position measurements.
            - Qi (float): Input control uncertainty.
            - R_GPS (float): Uncertainty in the GPS measurements.
            - At (numpy array): Transition matrix for the Kalman filter.
            - Bt (numpy array): Control input matrix for the Kalman filter.
            - fixed_target (bool): True if the target is fixed, False if it's dynamic.
            - n_robots (int): Total number of robots in the simulation.
            - RobGain (float, optional): Gain for robot motion control (default is 0.1).
        """

        self.unique_id = unique_id

        self.fixed_target = fixed_target
        self.n_robots = n_robots

        self.Ri = Ri
        self.Hi = Hi
        self.Qi = Qi
        self.R_GPS = R_GPS
        self.RobGain = RobGain
        self.target_radius = target_radius

        self.Fi = None
        self.ai = None

        self.FiStore = None
        self.aiStore = None

        self.pos = pos

        self.At = At
        self.Bt = Bt

        self.Ut = np.array([[0.3], [0.]])

        # Suppose we know exactly the initial position
        self.x_est = pos
        self.P_est = np.zeros((2, 2))
        # Uncertainty on the initial location of the target
        self.p_est_distr = np.array([[10.], [30.]])
        # self.p_est_distr = np.zeros((2, 1))
        # Uncertainty on the initial location of the target
        # self.Th_KF = np.eye(2)
        self.Th_KF = np.eye(2)
        # np.fill_diagonal(self.Th_KF, [(120 / 3) ** 2, (60 / 3) ** 2]) # initiallly 120x60m region
        np.fill_diagonal(self.Th_KF, [0, 0])

        # Uncertainty in target motion
        self.Qt = np.eye(2)
        # np.fill_diagonal(self.Qt, [0, (0.5 / 3) ** 2])
        np.fill_diagonal(self.Qt, [0, (0.5 / 3) ** 2])
        # self.Qt = (0.5 / 3) ** 2 * np.eye(2)

        self.rank = None
        self.u = np.array([[0.], [0.]])

        self.GP_step_count = 0

    def move(self, CMP, robots, A_tilda, target_pos):
        dt = 1
        if CMP:
            return self.move_as_CMP(robots, A_tilda, dt, target_pos)
        else:  # GP
            return self.move_as_GP()

    def move_as_CMP(self, robots, A_tilda, dt, target_pos):
        self.GP_step_count = 0
        theta = polar_angle(self.u)

        if self.rank == 0:
            v = 1
            diff = self.x_est - self.p_est_distr
            diff_norm = np.linalg.norm(diff)
            e_ij = diff / diff_norm
            aij = polar_angle(e_ij)
            gij = -np.sin(aij - polar_angle(self.u))
            theta += gij

            new_u = v * np.array([np.cos(theta), np.sin(theta)])
            self.u = new_u
            self.pos += self.u * dt

            self.pos_kf(self.u)
            if np.linalg.norm(self.pos - target_pos) < 5:
                # print("sono qui")
                return True
            return False
        # Define noise parameters
        D_theta = 0.1  # Adjust this value as needed for your specific noise strength

        counter = 0
        theta_dot = 0
        theta_dot_vel = 0

        for robot in robots:
            if robot == self:
                continue

            diff = self.x_est - robot.x_est
            diff_norm = np.linalg.norm(diff)
            e_ij = diff / diff_norm
            aij = polar_angle(e_ij)
            g_vel = np.sin(polar_angle(robot.u) - polar_angle(self.u))
            gij = -np.sin(aij - polar_angle(self.u))
            # e_theta = np.array([np.cos(self.u[0]) * self.pos[0], np.sin(self.u[1]) * self.pos[1]])

            A_ij = 1
            influence = A_tilda[self.rank, robot.rank] * A_ij

            if influence != 0:
                counter += 1

            # Add delta-correlated noise at each time step

            theta_dot_vel += influence * g_vel
            theta_dot += influence * gij

        noise = np.random.normal(0, 1)  # You can adjust the noise parameters as needed
        # Add the accumulated delta-correlated noise to theta_dot
        theta_dot += D_theta * noise

        theta += theta_dot

        if counter != 0:
            new_u = 0.7 * np.array([np.cos(theta), np.sin(theta)]) + 0.3 * np.array(
                [np.cos(theta_dot_vel), np.sin(theta_dot_vel)])
            # self.u = np.clip(new_u, -1, 1)
            self.u = new_u
            self.pos += self.u * dt

        self.pos_kf(self.u)
        return False

    def move_as_GP(self):
        self.GP_step_count += 0.000001
        probCMP = np.sqrt(self.GP_step_count)
        rand = random.uniform(0, 1)

        if rand < probCMP:
            # print("HERE")
            return False

        mean = 0
        std_dev = 0.2
        ProbMove = 0.1
        u_random_vector = np.zeros((2, 1))

        if np.random.rand() < ProbMove:
            u_random_vector = np.random.normal(mean, std_dev, size=(2, 1))

        self.u = 0.5 * self.u + u_random_vector
        dt = 1
        self.pos += self.u * dt
        self.pos_kf(self.u)
        return False

    def measure(self, target_pos):
        s_World = self.get_target_pos(target_pos)
        self.measure_target_pos(s_World)

    def predict(self):
        self.p_est_distr = self.At @ self.p_est_distr + self.Bt @ self.Ut
        self.Th_KF = self.At @ self.Th_KF @ self.At.T + self.Qt
        return

    def share(self, robot_neighborhood, D, target_pos):
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
        s = self.Hi @ t_pos_robot_frame + np.random.multivariate_normal([0, 0], self.Ri).reshape((-1, 1))
        # Transform the robot measurements into world measurements (the common reference frame for all the robots)
        s_World = s + self.Hi @ self.x_est
        return s_World

    def measure_target_pos(self, s_World):
        # Consensus
        zi = s_World
        self.Fi = self.Hi.T @ np.linalg.inv(self.Ri + self.Hi @ self.P_est @ self.Hi.T) @ self.Hi
        self.ai = self.Hi.T @ np.linalg.inv(self.Ri + self.Hi @ self.P_est @ self.Hi.T) @ zi

        self.FiStore = self.Fi
        self.aiStore = self.ai

    def update_target_est(self):
        if self.fixed_target:
            self.p_est_distr = np.linalg.inv(self.Fi) @ self.ai
            return

        PredP = self.Th_KF
        self.Th_KF = np.linalg.inv(self.Th_KF + self.n_robots * self.Fi)
        self.p_est_distr = self.Th_KF @ (PredP @ self.p_est_distr + self.n_robots * self.ai)

        return

    def pos_kf(self, u):
        # Update estimated position
        random_vector = np.random.multivariate_normal([0, 0], self.Qi).reshape((-1, 1))
        uUnc = u + random_vector
        # uUnc = np.clip(uUnc, -1, 1)
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

        self.x_est = self.x_est + W @ Innovation  # updated state estimate

        self.P_est = (np.identity(2) - W @ H_GPS) @ self.P_est  # updated covariance matrix

        return
