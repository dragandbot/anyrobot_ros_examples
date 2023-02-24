# ROS anyrobot_examples

Examples for the AnyRobot Interface with ROS infrastructure.

# Requirements

AnyRobot Interface must be running locally inside a drag&bot runtime instance. Examples cannot be run without the interface.
The examples must be placed in the overlay directory and can only be executed inside the drag&bot Docker runtime container. This is necessary due to the imported ROS message and service types.
drag&bot Robot Simulator must be running with a Simbot selected.

# Installation

1. Clone the repository into the drag&bot overlay, usually ~/dnb_catkin_ws/src
2. Restart the drag&bot Runtime

# Examples execution

Enter the drag&bot console in a linux terminal: dnb-console
Execute each example using rosrun command, e.g. rosrun anyrobot_ros_examples ros_example_ios.py

# Example / Tools list

1. ros_example_read_robot_positions.py

This example connects to the AnyRobot Interface and starts receiving joint and tool positions. These values are printed in the console.

Trace in console looks similar to:

Robot position as Euler Intrinsic ZYX frame (m, radians): {'x': 0.5015877485275269, 'y': 2.310114979309219e-07, 'z': 0.7040988206863403, 'alpha': -3.141592264175415, 'beta': 0.01039330754429102, 'gamma': 3.141591787338257}
Joint positions in radians: [0.0, 0.0908, 1.5708, 0.0, 1.4696, 0.0]

2. ros_example_move_robot.py

This example periodically moves the robot up and down one time while printing its joints in a similar way to the previous example.

3. ros_example_ios.py

This example reads value of digital output number one, sets its value to the inverted value, and reads the value again.

Expected trace:

rosrun anyrobot_ros_examples ros_example_ios.py
Digital Output original value: False
Digital Output current value: True