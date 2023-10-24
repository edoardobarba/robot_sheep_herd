import numpy as np
import matplotlib.pyplot as plt

from model import HerdModel  # Import the HerdModel from the 'model' module


# Define a function to run experiments with different parameters
def batch_run(n_robots, connections, max_steps, scale_R, n_run, fixed_target, show_sim, exp_number):
    """
    Run experiments with different parameters using the HerdModel simulation.

    Parameters:
        - n_robots (list): List of the number of robots to use in experiments.
        - connections (list): List of the number of connections to use in experiments.
        - max_steps (int): Maximum number of simulation steps.
        - scale_R (list): List of scale_R values for experiments.
        - n_run (int): Number of runs for each combination of parameters.
        - fixed_target (bool): True if the target is fixed, False if it's dynamic.
        - show_sim (bool): True to display the simulation, False to run in the background.
        - exp_number (int): Experiment number to choose the experiment type.

    Returns:
        None
    """
    if n_run == 0:
        return

    if exp_number == -1:
        # Create an instance of the HerdModel
        model = HerdModel(n_robots=n_robots[0], max_steps=max_steps, scale_R=scale_R[0], fixed_target=fixed_target,
                          connections=connections[0], show_sim=show_sim)
        last_step = -1
        print("Running...")
        for i in range(max_steps):
            flag = model.step(i)

            if fixed_target and (flag or i == max_steps - 1):
                last_step = i
                break

        if show_sim:
            model.plot_history_pos(last_step)
            model.plot_traj()

        return

    if exp_number == 0:
        # Initialize a list to store average time data for different scale_R values
        avg_time_data = []
        for s in scale_R:
            avg_time = []
            for k in range(n_run):
                # Create an instance of the HerdModel
                model = HerdModel(n_robots=n_robots[0], max_steps=max_steps, scale_R=s, fixed_target=fixed_target,
                                  connections=connections[0], show_sim=show_sim)
                for i in range(max_steps):
                    flag = model.step(i)

                    if fixed_target and (flag or i == max_steps - 1):
                        last_step = i
                        avg_time.append(i)
                        break

                if show_sim:
                    model.plot_history_pos(last_step)
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
                        model = HerdModel(n_robots=n, max_steps=max_steps, scale_R=s, fixed_target=fixed_target,
                                          connections=n_conn, show_sim=show_sim)
                        for i in range(max_steps):
                            flag = model.step(i)

                            if fixed_target and (flag or i == max_steps - 1):
                                last_step = i
                                avg_time.append(i)
                                break

                        if show_sim:
                            model.plot_history_pos(last_step)
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
                model = HerdModel(n_robots=n_robots[0], max_steps=max_steps, scale_R=s, fixed_target=fixed_target,
                                  connections=connections[0], show_sim=show_sim)
                for i in range(max_steps):
                    model.step(i)
                    avg_dist.append(model.get_avg_dist())
                if show_sim:
                    model.plot_history_pos(-1)
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
            model = HerdModel(n_robots=n_robots[0], max_steps=max_steps, scale_R=scale_R[0], fixed_target=fixed_target,
                              connections=connections[0], show_sim=show_sim)
            for i in range(max_steps):
                flag = model.step(i)

                if fixed_target and (flag or i == max_steps - 1):
                    last_step = i
                    avg_time.append(i)
                    print("time: ", i)
                    break

            if show_sim:
                model.plot_history_pos(last_step)
                model.plot_traj()

        # Store the average time data for the specific case
        avg_time_data.append(np.mean(np.array(avg_time)))
        print("avg time: ", np.mean(np.array(avg_time_data)), " std_dev: ", np.std(np.array(avg_time_data)))

        return

