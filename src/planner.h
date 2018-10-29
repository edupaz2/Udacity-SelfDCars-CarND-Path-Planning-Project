#ifndef _PLANNER_H_
#define _PLANNER_H_

class Planner
{
public:
  void    init                (int totalLanes, double maxSpeed);

  void    tick                (double tickTime, double car_s, double car_d, const std::vector< std::vector<double> >& sensor_fusion);

  double  getTargetSpeed      () const { return m_targetSpeed; }
  int     getCurrentLane      () const { return 4*m_currentLane+2; }

private:

  int      m_totalLanes;
  double   m_maxSpeed;

  int      m_currentLane;
  double   m_targetSpeed; // The speed we want to achieve. It can be higher or lower than our current speed.

  std::vector< std::pair<double, double> > m_laneDistancesAndSpeeds;

  // 0 - KEEP LANE
  // 1 - PREPARING FOR LANE CHANGE
  // 2 - CHANGING LANE
  unsigned int  m_state;
  unsigned int  m_targetLane;
  double    carSpeedAtLeftLane    () const;
  double    carSpeedAtRightLane   () const;
};


void Planner::init(int totalLanes, double maxSpeed)
{
  m_totalLanes = totalLanes;
  m_maxSpeed = maxSpeed;
  m_targetSpeed = m_maxSpeed;

  m_laneDistancesAndSpeeds = std::vector< std::pair<double, double> >(m_totalLanes);
  std::cout << "**** PLANNER - KEEP LANE " << m_currentLane << " ****" << std::endl;
  m_state = 0;
  m_currentLane = 1;
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
      m_laneDistancesAndSpeeds[i].first = 10000; // Some large number for the distance
      m_laneDistancesAndSpeeds[i].second = m_maxSpeed;
    }

    const int size = sensor_fusion.size();
    for(int i=0; i < size; ++i)
    {
      int car_lane = (sensor_fusion[i][6] / 4.0);
      if(car_lane >= 0 && car_lane < m_totalLanes)
      {
        double s = sensor_fusion[i][5];
        double distance = s - car_s;
        // Check the closest car in ahead of us in each lane
        if(distance < m_laneDistancesAndSpeeds[car_lane].first && distance > 0.0)
        {
          m_laneDistancesAndSpeeds[car_lane].first = distance;
          double vx = sensor_fusion[car_lane][3];
          double vy = sensor_fusion[car_lane][4];
          m_laneDistancesAndSpeeds[car_lane].second = sqrt(vx*vx + vy*vy);
        }
      }
    }
    // Decide the lane and speed
    switch(m_state)
    {
      case 0: // KEEP LANE
      {
        if (m_laneDistancesAndSpeeds[m_currentLane].first >= 50.0)
        {
          m_targetSpeed = m_maxSpeed;
        }
        else
        {
          std::cout << "**** PLANNER - CAR AHEAD ****" << std::endl;
          // Keep the lane
          m_targetSpeed = m_laneDistancesAndSpeeds[m_currentLane].second;
          // And plan to change lane
          m_state = 1;
        }
        break;
      }
      case 1:
      {
        std::cout << "**** PLANNER - PREPARING TO CHANGE ****" << std::endl;
        // Decide which lane. Need to check space availability and car ahead's speed
        double speedArray[3] = {carSpeedAtLeftLane(), m_targetSpeed, carSpeedAtRightLane()};
        double best_speed = 0.0;
        for(int i=0; i<3; ++i)
        {
          if(speedArray[i] > best_speed)
          {
            m_targetLane = m_currentLane+i-1;
            best_speed = speedArray[i];
          }
        }
        m_targetSpeed = best_speed - 3.0; // Slow down a little to avoid Changing lanes too fast.
        m_state = 2;
        break;
      }
      case 2:
      {
        std::cout << "**** PLANNER - CHANGING TO LANE " << m_targetLane << " ****" << std::endl;
        m_currentLane = m_targetLane;
        if( 4*m_currentLane+1 < car_d && car_d < 4*m_currentLane+3 )
        {
          std::cout << "**** PLANNER - KEEP LANE " << m_currentLane << " ****" << std::endl;
          m_state = 0;
        }
        break;
      }
    }
  }
}

double Planner::carSpeedAtLeftLane() const
{
  if(m_currentLane-1 >= 0)
  {
    if(m_laneDistancesAndSpeeds[m_currentLane-1].first >= 50.0)
      return m_maxSpeed;
    else if(m_laneDistancesAndSpeeds[m_currentLane-1].first <= 30.0) // For safety
      return 0.0;
    return m_laneDistancesAndSpeeds[m_currentLane-1].second;
  }
  return 0.0;
}

double Planner::carSpeedAtRightLane() const
{
  if(m_currentLane+1 < m_totalLanes)
  {
    if(m_laneDistancesAndSpeeds[m_currentLane+1].first >= 50.0)
      return m_maxSpeed;
    else if(m_laneDistancesAndSpeeds[m_currentLane+1].first <= 30.0) // For safety
      return 0.0;
    return m_laneDistancesAndSpeeds[m_currentLane+1].second;
  }
  return 0.0;
}

#endif
