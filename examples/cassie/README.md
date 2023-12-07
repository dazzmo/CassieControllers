# Cassie

## Cassie Simulation
To run the simulation with the controller, build the code and navigate to `build/examples/cassie/` then run:

    ./cassie_sim

## Cassie Code Generation
To generate code for the model, build the code and navigate to `build/examples/cassie/` then run: 

    ./cassie_codegen_leg <code generation destination>

## Current Code Status (22/11/2023)

We have currently developed an OSC-based controller to move Cassie's feet to track desired trajectories while the robot is suspended on its stand.

Currently, we have several tasks that we can include in the controller, a list of them are as follows:

1. The ankle tracking task is your standard task-space task, a three-dimensional task that attempts to compute the necessary torques required to move the ankle (the toe joint of the leg) to a desired position with respect to the pelvis.

2. The joint tracking task is a higher-dimensional task that tries to regulate the joints of the robot to a desired position and/or velocity. 

    - Use this task to include joint-damping costs for smoother motion.
    - In older versions of this code, we've used IK to generate reference trajectories in joint-space to track.

3. The joint limit task is still a work in progress, it acts similarly to the joint tracking task, however it tries to act as a rigid-spring/damper system to repel the joints from their limits.

4. Lastly, we can add holonomic constraints to the problem (their contraint forces are solved for in the optimisation). We can also add projected constraints, which are similar to the holnomic constraints but the constraint forces are solved for before the optimisation is run (i.e. all other constraints are projected into the null space of these constraints).