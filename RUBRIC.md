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
1. Create a list of waypoints and get the car follow it.
2. Make the trajectory smooth using spline, while reusing previous trajectory points left. 
3. Obey to speed/acc/jerk constraints and avoid collisions.
4. Change lanes.

#### Detailed description
## Step 1: Create a list of waypoints and get the car follow it.

The road is divided into 181 waypoints. I create a small list of the next 3 waypoints ahead of us plus our current position and a previous position based on our current yaw (to avoid sudden turns).
Check the code in main.cpp from 461-514.

## Step 2. Make the trajectory smooth using spline, while reusing previous trajectory points left. 
Once the previous list is created we use a Spline Tool (http://kluge.in-chemnitz.de/opensource/spline/) to create a smoother path for the car to drive through.
Check the code in main.cpp from 517-559.

## Step 3. Obey to speed/acc/jerk constraints and avoid collisions.
In order to avoid sudden turns, acceleration, we increase the speed by a certain amount to reach our target speed (At main.cpp the variable path_planned_speed help us with that).
Check the code in main.cpp from 535-542.
Check also main.cpph line 166, where we slow down 20% when changing lanes.

## Step 4. Change lanes.
I created the Planner class (lines 23-218) to help deciding what to do about lane and speed. It´s based on FSM with 4 states:
- State 0: Keep the current lane.
- State 1: Decide if we change lanes.
- State 2: Start the lane change.
- State 3: Finish the lane change.

On State 0: (main.cpp line 122-144)
If there is no car ahead of us (more than 30m ahead), our target speed is the max speed.
If there is a car in front of us at less than 30 meters, we match its speed and check if we want a lane change (State 1).

On State 1: (main.cpp line 145-170)
We check our left lane and right lane (if they are drivable) for the best speed, checking the following parameters for a safe maneuver:
- If a car is behind less than 10m, the lane is discarded.
- If a car is ahead less than 30m, the lane is discarded.
- If a car is ahead between 30m-40m, the lane max speed will be this car´s speed.
- If no car ahead, the lane max speed will be the max speed.
With left, current and right lanes speeds, the we choose the best lane. If we choose to change lane, we go to State 2.

On State 2: (main.cpp line 171-184)
We start the lane change reducing the speed by 20% and starting an smooth maneuver. When we crossed the road lane between the two lanes, we go to State 3.

On State 3: (main.cpp line 185-194)
We finishe the by arriving to the middle of the target lane. Once we arrive, we change to State 0, and keep the lane again.

#### Improvements:
- We are not checking cars coming from behind at higher speeds than us.
- We are not canceling the lane change if there is change of the events: car braking, car coming from behind.


