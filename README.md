# toro_orchard_navigation
Orchard navigation by a Toro electric utility vehicle using a spinning 2D Lidar.

This is a ROS catkin workspace that is used on a vehicle that contains a 2D Lidar on a motor at the front of a vehicle, and encoders and motors at the steering wheel and rear wheel axle. Commands are sent to power the driving and steering and to take the state of the encoders. The spinning 2D Lidar creates a point cloud (that then goes through the iterative closest point method) in which algorithms are run that extract lines of trees on each side of the vehicle. These algorithms include Random Sample Consensus, an Extended Kalman Filter, and a probability-based line offset. The navigation_controller_node then takes the generated lines and the post-icp point cloud and uses Pure Pursuit to navigate between the rows of trees. The node also develops driving commands to turn when the vehicle has reached the end of a row. The vehicle has the capability to then navigate through the rows of an orchard.

A video of the vehicle in action (sped up to shorten the video) is located here: https://www.dropbox.com/s/nkjxrsi4lje69aw/Nov4_Autonomous_Navigation.mp4?dl=0
