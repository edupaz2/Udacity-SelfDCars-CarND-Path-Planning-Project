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

The road is divided into 181 waypoints. We create a small list of the next 3 waypoints ahead of us plus our current position and a previous position based on our current yaw (to avoid sudden turns).
Check the code in main.cpp from 265-318.

## Step 2. Make the trajectory smooth using spline, while reusing previous trajectory points left. 
Once the previous list is created we use a Spline Tool (http://kluge.in-chemnitz.de/opensource/spline/) to create a smoother path for the car to drive through.
Check the code in main.cpp from 320-363.

## Step 3. Obey to speed/acc/jerk constraints and avoid collisions.
In order to avoid sudden turns, acceleration, we increase the speed by a certain amount to reach our target speed (At main.cpp the variable path_planned_speed help us with that).
Check the code in main.cpp from 339-343.
Check also planner.h line 114, where we slow down a little when changing lanes.

## Step 4. Change lanes.
I created a the Planner class to help deciding what to do: lane and speed. It´s based on FSM with 3 states:
- State 1: Keep the current lane.
- State 2: Decide if we change lanes.
- State 3: Perform the lane change.

On State 1: (planner.h line 84-98)
If there is no car ahead of us (more than 50m ahead), our target speed is the max speed.
If there is a car in front of us at less than 50 meters, we target its speed and change to State 2.

On State 2: (planner.h line 100-116)
We check our left lane, current lane and right lane for a better speed. That means, checking if there is a car in front of us, its speed, and if there is enough (safe) space to change lane (if we decide to).
Once we choose the best option, we target our lane and speed and change to State 3.

On State 3: (planner.h line 118-127)
We are performing the lane change. Once we arrive, we change to state 0, and keep the lane again.

#### Improvements:
- We are not checking cars coming from behind at higher speeds than us.
- We are not canceling the lane change if there is change of the events: car braking, car coming from behind.


