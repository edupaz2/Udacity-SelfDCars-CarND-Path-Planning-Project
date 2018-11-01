#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

class Planner
{
public:
  void    init                 (int totalLanes, double maxSpeed);

  void    tick                 (double tickTime, double car_s, double car_d, const std::vector< std::vector<double> >& sensor_fusion);

  double  getTargetSpeed       () const { return m_targetSpeed; }
  int     getCurrentLane       () const { return 4*m_currentLane+m_currentLaneOffset; }

private:

  int      m_totalLanes;
  double   m_maxSpeed;

  int      m_currentLane;
  double   m_targetSpeed; // The speed we want to achieve. It can be higher or lower than our current speed.

  std::vector< std::pair<double, double> > m_carsAheadDistanceAndSpeed;
  std::vector< std::pair<double, double> > m_carsBehindDistanceAndSpeed;

  // 0 - KEEP LANE
  // 1 - PREPARING FOR LANE CHANGE
  // 2 - CHANGING LANE
  int     m_state;
  int     m_targetLane;
  int     m_currentLaneOffset;
  double  checkSpeedInLane     (int lane) const;
};


void Planner::init(int totalLanes, double maxSpeed)
{
  m_totalLanes = totalLanes;
  m_maxSpeed = maxSpeed;
  m_targetSpeed = m_maxSpeed;

  m_carsAheadDistanceAndSpeed = std::vector< std::pair<double, double> >(m_totalLanes);
  m_carsBehindDistanceAndSpeed = std::vector< std::pair<double, double> >(m_totalLanes);
  std::cout << "**** PLANNER - KEEP LANE " << m_currentLane << " ****" << std::endl;
  m_state = 0;
  m_currentLane = 1;
  m_currentLaneOffset = 2;
  m_targetLane = m_currentLane;
}

void Planner::tick(double tickTime, double car_s, double car_d, const std::vector< std::vector<double> >& sensor_fusion)
{
  // BEHAVIOR PLANNER
  if (m_targetSpeed == 0.0)
  {
    m_targetSpeed = m_maxSpeed;
  }
  else
  {
    // The data format in sensor_fusion is: [ id, x, y, vx, vy, s, d].
    for(int i=0; i<m_totalLanes; ++i)
    {
      m_carsAheadDistanceAndSpeed[i].first = 10000; // Some large number for the distance
      m_carsAheadDistanceAndSpeed[i].second = m_maxSpeed;

      m_carsBehindDistanceAndSpeed[i].first = 10000; // Some large number for the distance
      m_carsBehindDistanceAndSpeed[i].second = m_maxSpeed;
    }

    const int size = sensor_fusion.size();
    for(int i=0; i < size; ++i)
    {
      int car_lane = (sensor_fusion[i][6] / 4.0);
      if(car_lane >= 0 && car_lane < m_totalLanes)
      {
        double other_car_s = sensor_fusion[i][5];
        double distance = other_car_s - car_s;
        // Check the closest car in ahead of us in each lane
        if(distance > 0.0)
        {
          if(distance < m_carsAheadDistanceAndSpeed[car_lane].first)
          {
            m_carsAheadDistanceAndSpeed[car_lane].first = distance;
            double vx = sensor_fusion[car_lane][3];
            double vy = sensor_fusion[car_lane][4];
            m_carsAheadDistanceAndSpeed[car_lane].second = sqrt(vx*vx + vy*vy);
          }
        }
        else
        {
          if(abs(distance) < m_carsBehindDistanceAndSpeed[car_lane].first)
          {
            m_carsBehindDistanceAndSpeed[car_lane].first = abs(distance);
            double vx = sensor_fusion[car_lane][3];
            double vy = sensor_fusion[car_lane][4];
            m_carsBehindDistanceAndSpeed[car_lane].second = sqrt(vx*vx + vy*vy);
          }
        }
      }
    }
    // Decide the lane and speed
    switch(m_state)
    {
      case 0: // KEEP LANE
      {
        if (m_carsAheadDistanceAndSpeed[m_currentLane].first >= 30.0)
        {
          m_targetSpeed = m_maxSpeed;
        }
        else
        {
          // Keep the lane
          m_targetSpeed = m_carsAheadDistanceAndSpeed[m_currentLane].second;
          // And plan to change lane
          if(checkSpeedInLane(m_currentLane-1) > m_targetSpeed || checkSpeedInLane(m_currentLane+1) > m_targetSpeed)
          {
            std::cout << "**** PLANNER - CAR AHEAD. CHECK LANE CHANGING ****" << std::endl;
            m_state = 1;
          }
          else
          {
            std::cout << "**** PLANNER - CAR AHEAD. SLOW DOWN.(" << m_targetSpeed << "m/s) ****" << std::endl;
          }
        }
        break;
      }
      case 1:
      {
        // Decide which lane. Need to check space availability and car ahead's speed
        const double speedArray[3] = {checkSpeedInLane(m_currentLane-1), m_targetSpeed, checkSpeedInLane(m_currentLane+1)};
        double best_speed = 0.0;
        for(int i=0; i<3; ++i)
        {
          if(speedArray[i] > best_speed)
          {
            m_targetLane = m_currentLane+i-1;
            best_speed = speedArray[i];
          }
        }
        if(m_targetLane == m_currentLane)
        {
          std::cout << "**** PLANNER - KEEP LANE " << m_currentLane << " ****" << std::endl;
          m_state = 0;
        }
        else
        {
          std::cout << "**** PLANNER - CHANGING TO LANE " << m_targetLane << " ****" << std::endl;
          m_targetSpeed = m_targetSpeed*0.8; // Slow down 20% to avoid changing lanes too fast.
          m_state = 2;
        }
        break;
      }
      case 2:
      {
        if(m_targetLane < m_currentLane)
        {
          m_currentLaneOffset = 4;
        }
        else
        {
          m_currentLaneOffset = 0;
        }
        m_currentLane = m_targetLane;
        m_state = 3;
        break;
      }
      case 3:
      {
        if( 4*m_currentLane+m_currentLaneOffset-1 < car_d && car_d < 4*m_currentLane+m_currentLaneOffset+1 )
        {
          m_currentLane = m_targetLane;
          m_currentLaneOffset = 2;
          std::cout << "**** PLANNER - KEEP LANE " << m_currentLane << " ****" << std::endl;
          m_state = 0;
        }
        break;
      }
    }
  }
}

double Planner::checkSpeedInLane(int lane) const
{
  if(lane >= 0)
  {
    // Check safety first: a car behind or ahead too close
    if(m_carsBehindDistanceAndSpeed[lane].first <= 10.0 || m_carsAheadDistanceAndSpeed[lane].first <= 30.0)
    {
      return 0.0;
    }

    if(m_carsAheadDistanceAndSpeed[lane].first <= 40.0)
      return m_carsAheadDistanceAndSpeed[lane].second;

    if(m_carsAheadDistanceAndSpeed[lane].first > 40.0)
      return m_maxSpeed; // Go as fast as you can!
  }
  return 0.0;
}
/// END PLANNER

// CONSTANTS
const double time_cycle = 0.02;
const double max_speed = 21.5;// 50MPH = 22.352; // m/s

// VARIABLES
double path_planned_speed = 0.0; // The speed on the last point of the trajectory path.
Planner planner;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1; 
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  planner.init(3, max_speed);

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

            // Reference variables
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            // BEHAVIOR PLANNER
            // 4. Change lanes.
            planner.tick(time_cycle, car_s, car_d, sensor_fusion);
            const double target_speed = planner.getTargetSpeed();
            const int car_lane = planner.getCurrentLane();

            // 1. Create a list of waypoints and get the car follow it.
            // 1.1 First obtain 2 points to concanete smoothly previous trajectory, if exists, and new.
            vector<double> anchor_x;
            vector<double> anchor_y;
            const int prev_path_size = previous_path_x.size();
            if(prev_path_size < 2)
            {
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              anchor_x.push_back(prev_car_x);
              anchor_x.push_back(car_x);

              anchor_y.push_back(prev_car_y);
              anchor_y.push_back(car_y);
            } else {
              ref_x = previous_path_x[prev_path_size-1];
              ref_y = previous_path_y[prev_path_size-1];

              double prev_ref_x = previous_path_x[prev_path_size-2];
              double prev_ref_y = previous_path_y[prev_path_size-2];
              ref_yaw = atan2(ref_y-prev_ref_y, ref_x-prev_ref_x);

              anchor_x.push_back(prev_ref_x);
              anchor_x.push_back(ref_x);

              anchor_y.push_back(prev_ref_y);
              anchor_y.push_back(ref_y);
            }

            //std::cout << "---- ITER -----" << std::endl;
            //std::cout << "X: " << car_x << ", Y:" << car_y << ", S:" << car_s << ", D:" << car_d << ", YAW:" << car_yaw << ", SPEED:" << car_speed << ", TARGET: " << target_speed*2.23 <<  std::endl;

            // 1.2 Create the new anchor points for the spline
            // The road has 181 waypoints and 6945.554 meters length
            // Each waypoint covers around 38.373 meters (close to 40 meters).
            // Let´s visit the next 3
            const int waypoint_distance = 40; // 38.4 meters
            const int waypoints_to_visit = 3;
            for(int i=1; i<=waypoints_to_visit; ++i)
            {
              vector<double> anchor_xy = getXY(car_s+(i*waypoint_distance), car_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              anchor_x.push_back(anchor_xy[0]);
              anchor_y.push_back(anchor_xy[1]);
            }
            // 1.3, shift and rotate the new path to car´s origin coordinate system. 
            for(int i=0; i< anchor_x.size(); ++i)
            {
              double shift_x = anchor_x[i]-ref_x; 
              double shift_y = anchor_y[i]-ref_y;

              anchor_x[i] = (shift_x*cos(0-ref_yaw))-shift_y*sin(0-ref_yaw);
              anchor_y[i] = (shift_x*sin(0-ref_yaw))+shift_y*cos(0-ref_yaw);
            }

            // 2. Make the trajectory smooth using spline, while reusing previous trajectory points left. 
            tk::spline s;
            s.set_points(anchor_x, anchor_y);

            // 2.1 Define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            vector<double> next_x_vals;
            vector<double> next_y_vals;

            // 2.2. Reuse add the previous path points.
            for(int i=0; i<prev_path_size; ++i)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            // 2.3 Create the new points. Obey to the acceleration and jerk constraints
            const int remaining_points = 50-prev_path_size;
            for(int i=1; i<=remaining_points; ++i)
            {
              if(path_planned_speed < target_speed)
              {
                path_planned_speed += 0.08;
              }
              else if(path_planned_speed > target_speed)
              {
                path_planned_speed -= 0.08;
              }
              double step = i*(path_planned_speed * time_cycle);

              double new_x = step;
              double new_y = s(new_x);

              double shift_x = new_x; 
              double shift_y = new_y;

              new_x = (shift_x*cos(ref_yaw))-shift_y*sin(ref_yaw);
              new_y = (shift_x*sin(ref_yaw))+shift_y*cos(ref_yaw);

              new_x += ref_x;
              new_y += ref_y;

              next_x_vals.push_back(new_x);
              next_y_vals.push_back(new_y);
            }

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
