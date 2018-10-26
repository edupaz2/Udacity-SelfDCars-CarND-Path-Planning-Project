# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

#### Constraints we must satisfy

Velocity below and close to 22,352 m/s (50MPH).
Total acceleration below 10 m/s^2.
Jerk below 10 m/s^3.
Change lane time lower than 3 seconds.
Avoid collisions.

#### Steps and iterations:
Steps:
1. Get the car follow a straight line of waypoints in the middle lane. There are no checks for collisions, and speed/acc/jerk constraints .
2. Make the trajectory smooth using spline, while reusing previous trajectory points left. 
3. Obey to speed/acc/jerk constraints and avoid collisions.
4. Change lanes.

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

####