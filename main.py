from batch_run import *

if __name__ == '__main__':
    # PUT BATCH_RUN = False TO NOT RUN BATCH RUN
    BATCH_RUN = False
    SINGLE_RUN = True

    if SINGLE_RUN:
        # PARAMETERS FOR SINGLE RUN
        show_sim = True  # Enable the simulation display
        fixed_target = False  # Use a fixed target
        max_steps = 1000  # Maximum number of steps for the simulation
        n_robots = 4  # Number of robots in the simulation
        scale_R = 10  # Scaling factor for R

        """
        Number of connections between robots, Maximum values: 
            max 1 if n_robots = 2
            max 3 if n_robots = 3
            max 6 if n_robots = 4
            max 10 if n_robots = 5
            max 15 if n_robots = 6
            ...
        """
        n_connections = 6

        max_conn = (n_robots * (n_robots - 1)) // 2
        if n_connections > max_conn:
            n_connections = max_conn

        batch_run([n_robots], connections=[n_connections], scale_R=[scale_R], fixed_target=fixed_target,
                  max_steps=max_steps, n_run=1, show_sim=show_sim, exp_number=-1)

    if BATCH_RUN:
        # Set batch run parameters
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
