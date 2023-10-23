import numpy as np

from model import *


def batch_run(n_robots, connections, max_steps, scale_R, n_run, fixed_target, show_sim, exp_number):
    if exp_number == 0:
        avg_time_data = []
        for s in scale_R:
            avg_time = []
            for k in range(n_run):
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

            # N_CONNECTION
            # key = s  # Use a tuple (scale_R, n_conn) as the key
            avg_time_data.append(np.mean(np.array(avg_time)))

            print("scale_R: ", s)
            print("avg time: ", np.mean(np.array(avg_time)))

        print(avg_time_data)
        print(scale_R)
        plt.plot(scale_R, avg_time_data)

        plt.xlabel('scale_R')
        plt.ylabel('Average Time')
        plt.show()

        return avg_time_data  # Return the data for plotting

    if exp_number == 1:
        avg_time_data = {}  # Create a dictionary to store the data for each scale_R
        for n in n_robots:
            for s in scale_R:
                for n_conn in connections:
                    avg_time = []
                    # avg_dist = []
                    for k in range(n_run):
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

                    # N_CONNECTION
                    key = (s, n_conn)  # Use a tuple (scale_R, n_conn) as the key
                    avg_time_data[key] = np.mean(np.array(avg_time))

                    print("n_connections: ", n_conn, "scale_R: ", s)
                    print("avg time: ", np.mean(np.array(avg_time)))

        for scale_R_value in scale_R:
            avg_times = [avg_time_data[(scale_R_value, n_conn)] for n_conn in n_connections]
            plt.plot(n_connections, avg_times, label=f'scale_R={scale_R_value}')

        plt.xlabel('Connections')
        plt.ylabel('Average Time')
        plt.legend()
        plt.show()

        return avg_time_data  # Return the data for plotting

    if exp_number == 2:
        avg_dist_data = []
        for s in scale_R:
            avg_dist = []
            for k in range(n_run):
                model = HerdModel(n_robots=n_robots[0], n_steps=max_steps, scale_R=s, fixed_target=fixed_target,
                                  connections=connections[0], show_sim=show_sim)
                for i in range(max_steps):
                    model.step(i)
                    avg_dist.append(model.get_avg_dist())
                if show_sim:
                    model.plot_history_pos()
                    model.plot_traj()

            avg_dist_data.append(np.mean(np.array(avg_dist)))
            print("scale_R: ", s)
            print("avg dist: ", np.mean(np.array(avg_dist)))

        print(avg_dist_data)
        print(scale_R)
        plt.plot(scale_R, avg_dist_data)

        plt.xlabel('scale_R')
        plt.ylabel('Average Dist')
        plt.show()

        return avg_dist_data

    if exp_number == 3:
        avg_time_data = []

        avg_time = []
        for k in range(n_run):
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


        avg_time_data.append(np.mean(np.array(avg_time)))
        print("avg time: ", np.mean(np.array(avg_time)), " std_dev: ", np.std(np.array(avg_time)))

        return avg_time_data  # Return the data for plotting


if __name__ == '__main__':

    n_robots = [4]
    scale_R = [1, 10]
    n_connections = [0]
    show_sim = True
    fixed_target = True
    n_run = 10  # number of runs per combination of parameters
    if fixed_target:
        max_steps = 2000
    else:
        max_steps = 300

    exp_number = 0
    batch_run(n_robots, connections=n_connections, scale_R=scale_R, fixed_target=fixed_target,
              max_steps=max_steps, n_run=n_run, show_sim=show_sim, exp_number=exp_number)
