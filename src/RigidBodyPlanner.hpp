#ifndef RIGID_BODY_PLANNER_HPP_
#define RIGID_BODY_PLANNER_HPP_

#include "RigidBodySimulator.hpp"
#include <vector>

struct RigidBodyMove
{
    double m_dx;
    double m_dy;
    double m_dtheta;
};

class RigidBodyPlanner
{
public:
    RigidBodyPlanner(RigidBodySimulator * const simulator);

    ~RigidBodyPlanner(void);

    /*
     * This is the function that you should implement.
     * This function needs to compute by how much the position (dx, dy)
     * and orientation (dtheta) should change so that the robot makes a small
     * move toward the goal while avoiding obstacles,
     * as guided by the potential field.
     *
     * You have access to the simulator.
     * You can use the methods available in simulator to get all the information
     * you need to correctly implement this function
     *
     */
    RigidBodyMove ConfigurationMove(void);
    std::vector<double> RepulsiveForce(double, double);
    std::vector<double> AttractiveForce(double, double);
    std::vector<double> AttractiveForceToEscapeGoal(double, double,double, double);
    std::vector<double> TranspoedJacobianAB(double, double);
	double DistanceToObs(double, double, double, double);
	double step;
  double tStep;
	double threshold;
    bool escape;
    bool reachedTempGoal;
    int multiplier; //remove?
    int escapecntr; //remove?
    double escapeX;
    double escapeY;
    double escapeThreshold;

protected:
    RigidBodySimulator *m_simulator;
};

#endif
