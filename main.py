import numpy as np
import matplotlib.pyplot as plt

from model import HerdModel  # Import the HerdModel from the 'model' module


# Define a function to run experiments with different parameters
def batch_run(n_robots, connections, max_steps, scale_R, n_run, fixed_target, show_sim, exp_number):
    if n_run == 0:
        return

    if exp_number == 0:
        # Initialize a list to store average time data for different scale_R values
        avg_time_data = []
        for s in scale_R:
            avg_time = []
            for k in range(n_run):
                # Create an instance of the HerdModel
                model = HerdModel(n_robots=n_robots[0], n_steps=max_steps, scale_R=s, fixed_target=fixed_target,
                                  connections=connections[0], show_sim=show_sim)
                for i in range(max_steps):
                    flag = model.step(i)

                    if fixed_target and (flag or i == max_steps - 1):
                        avg_time.append(i)
                        break

                if show_sim:
                    model.plot_history_pos()
                    model.plot_traj()

            # Store the average time data for the current scale_R
            avg_time_data.append(np.mean(np.array(avg_time)))

            print("scale_R: ", s)
            print("avg time: ", np.mean(np.array(avg_time)))

        # Display the average time data using a plot
        plt.plot(scale_R, avg_time_data)

        plt.xlabel('scale_R')
        plt.ylabel('Average Time')
        plt.show()

        return

    if exp_number == 1:
        # Initialize a dictionary to store average time data for different scale_R and connection values
        avg_time_data = {}
        for n in n_robots:
            for s in scale_R:
                for n_conn in connections:
                    avg_time = []
                    for k in range(n_run):
                        # Create an instance of the HerdModel
                        model = HerdModel(n_robots=n, n_steps=max_steps, scale_R=s, fixed_target=fixed_target,
                                          connections=n_conn, show_sim=show_sim)
                        for i in range(max_steps):
                            flag = model.step(i)

                            if fixed_target and (flag or i == max_steps - 1):
                                avg_time.append(i)
                                break

                        if show_sim:
                            model.plot_history_pos()
                            model.plot_traj()

                    # Store the average time data using a tuple (scale_R, n_conn) as the key
                    key = (s, n_conn)
                    avg_time_data[key] = np.mean(np.array(avg_time))

                    print("n_connections: ", n_conn, "scale_R: ", s)
                    print("avg time: ", np.mean(np.array(avg_time)))

        # Display the average time data using separate lines for each scale_R value
        for scale_R_value in scale_R:
            avg_times = [avg_time_data[(scale_R_value, n_conn)] for n_conn in connections]
            plt.plot(connections, avg_times, label=f'scale_R={scale_R_value}')

        plt.xlabel('Connections')
        plt.ylabel('Average Time')
        plt.legend()
        plt.show()

        return

    if exp_number == 2:
        # Initialize a list to store average distance data for different scale_R values
        avg_dist_data = []
        for s in scale_R:
            avg_dist = []
            for k in range(n_run):
                # Create an instance of the HerdModel
                model = HerdModel(n_robots=n_robots[0], n_steps=max_steps, scale_R=s, fixed_target=fixed_target,
                                  connections=connections[0], show_sim=show_sim)
                for i in range(max_steps):
                    model.step(i)
                    avg_dist.append(model.get_avg_dist())
                if show_sim:
                    model.plot_history_pos()
                    model.plot_traj()

            # Store the average distance data for the current scale_R
            avg_dist_data.append(np.mean(np.array(avg_dist)))
            print("scale_R: ", s)
            print("avg dist: ", np.mean(np.array(avg_dist)))

        # Display the average distance data using a plot
        plt.plot(scale_R, avg_dist_data)

        plt.xlabel('scale_R')
        plt.ylabel('Average Dist')
        plt.show()

        return

    if exp_number == 3:
        # Initialize a list to store average time data for a specific scale_R and connection value
        avg_time_data = []

        avg_time = []
        for k in range(n_run):
            # Create an instance of the HerdModel
            model = HerdModel(n_robots=n_robots[0], n_steps=max_steps, scale_R=scale_R[0], fixed_target=fixed_target,
                              connections=connections[0], show_sim=show_sim)
            for i in range(max_steps):
                flag = model.step(i)

                if fixed_target and (flag or i == max_steps - 1):
                    avg_time.append(i)
                    print("time: ", i)
                    break

            if show_sim:
                model.plot_history_pos()
                model.plot_traj()

        # Store the average time data for the specific case
        avg_time_data.append(np.mean(np.array(avg_time)))
        print("avg time: ", np.mean(np.array(avg_time_data)), " std_dev: ", np.std(np.array(avg_time_data)))

        return


if __name__ == '__main__':






    # PUT N_RUN = 0 TO NOT RUN BATCH RUN
    n_run = 10
    show_sim = False

    exp_number = 2  # Specifies the experiment to run (Experiment 3: Alternating Leadership Roles).
    """
    Experiment 0: Uncertainty of Target Position Sensor
        In this preliminary experiment, we investigate the impact of sensor uncertainty on reaching a fixed target.
            -Four robots are used with the maximum number of connections.
            -The 'scale_R' parameter is varied to control sensor precision.
            -A fixed target is employed with a maximum number of steps (max_steps).
            -Results are plotted to visualize how sensor uncertainty affects robot trajectories.
    
    
    Experiment 1: Influence of Number of Connections
        This experiment explores the influence of the number of connections on the time it takes for robots to reach a fixed target.
            -Four robots are utilized with different connection configurations (0, 2, 4, or 6 connections).
            -The 'scale_R' parameter is varied to control sensor precision (1, 10, and 100).
            -The results are visualized to show the relationship between the number of connections and the average time required to reach the target.
        
        
    Experiment 2: Influence of Prediction Accuracy
    
        This experiment delves into the effect of prediction accuracy within the Kalman filter on tracking a moving target.
            -Four robots are employed with varying prediction accuracy, and the average distance from the target is measured as a metric.
            -The robots are divided into two scenarios: high prediction accuracy and low prediction accuracy.
            -Results are presented in both table and plot formats to compare the impact of prediction accuracy on tracking accuracy.
    
    
    Experiment 3: Alternating Leadership Roles
    
        In this experiment, we explore the importance of alternating the role of leader during Collective Motion Phases (CMP).
            -Four robots are used, with one robot having precise sensor information (scale_R = 10) while the others have high noise (scale_R = 1000).
            -Leadership roles are alternated randomly during CMP phases, and the results are compared with a scenario where leadership remains constant.
            -The experiment highlights the benefits of alternating leadership roles for effective information sharing and faster convergence.
    
    """
    if exp_number == 0:
        # Define the parameters for the experiment
        fixed_target = True
        max_steps = 2000
        n_robots = [4]
        scale_R = [1, 10, 100]
        n_connections = [6]

    elif exp_number == 1:
        # Define the parameters for the experiment
        fixed_target = True
        max_steps = 2000
        n_robots = [4]
        scale_R = [1, 10, 100]
        n_connections = [0, 2, 4, 6]

    elif exp_number == 3:
        # Define the parameters for the experiment
        fixed_target = True
        max_steps = 2000
        n_robots = [4]
        scale_R = [100]
        n_connections = [6]


    else:  # exp_number == 2
        # Define the parameters for the experiment
        fixed_target = False
        max_steps = 300
        n_robots = [4]
        scale_R = [10, 100, 1000]
        n_connections = [6]

    # Run the experiment with the specified parameters
    batch_run(n_robots, connections=n_connections, scale_R=scale_R, fixed_target=fixed_target,
              max_steps=max_steps, n_run=n_run, show_sim=show_sim, exp_number=exp_number)
