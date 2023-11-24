# Cassie
## Cassie Simulation
To run the simulation with the controller, you must execute the following command:
`./cassie_simulator <path to "agility_cassie/scene.xml">`

## Cassie Code Generation
To generate code for the model, run the following:
`./cassie_codegen_leg <code generation destination>`

## Current Code Status (22/11/2023)

We're currently developing an OSC-based leg controller to move the leg of cassie to a desired position whilst respecting the constraints of the leg.

Currently, we have several tasks that we can include in the controller, a list of them are as follows:

1. The ankle tracking task is your standard task-space task, a three-dimensional task that attempts to compute the necessary torques required to move the ankle (the toe joint of the leg) to a desired position with respect to the pelvis.

2. The joint tracking task is a higher-dimensional task that tries to regulate the joints of the robot to a desired position and/or velocity. We can set an IK reference as well as include joint-damping for smoother motions.

3. The joint limit task is still a work in progress, it acts similarly to the joint tracking task, however tries to act as a rigid-spring/damper system to repel the joints from their limits.

4. Lastly, we can add holonomic constraints to the problem (their contraint forces are solved for in the optimisation). We can also add projected constraints, which are similar to the holnomic constraints but the constraint forces are solved for before the optimisation is run (i.e. all other constraints are projected into the null space of these constraints).