#include "RigidBodyPlanner.hpp"

RigidBodyPlanner::RigidBodyPlanner(RigidBodySimulator * const simulator)
{
    m_simulator = simulator;   
}

RigidBodyPlanner::~RigidBodyPlanner(void)
{
    //do not delete m_simulator  
}


RigidBodyMove RigidBodyPlanner::ConfigurationMove(void)
    
{
    RigidBodyMove move;

	//start by taking all the vertices of our robot (this is a vector)
		//calculate attractive force for each vertices (basically have much the goal is pulling on it)
		//we'll make a helper function that returns a vector? of size 2 (really just a 2 by 1 matrix)that calculates for a specific point and then loop that over all the control pints
		//calculate the repulsive force of each obstacle on each control point (remember that there is a threshold you must account for - force is 0 outside of that threshold)
		//helper function that claculates force on a point (vecotr?) then a nested for loop to iterate over all obstacles and for every control point.
		//make helper function to create a seperate transposed jacobian for each vertex (they are all control points i think one long vector of size 9 will work fine for our purposes.) 
		//sum the attractive and repulsive forces for each control point seperately
		//matrix multiply the transposed jacobian (3 by 2) with the full force (2 by 1) to make a final vector (3 by 1) containg the movement of the control point in the x , y , and theta
		//loop over all the cotrol points and sum up the movements for each to recieve the final movement and return it as our move
		//
		//
		//add some functionality to get out of local minimums
		//
	//add your implementation

	return move;
}
//takes a control point (Cx, Cy) and returns the goals force on it. --------------JIMMY
std::vector AttractiveForce(double cx, double cy) {

	return 0.0



}
//takes a control point (Cx, Cy) and returns a sum of the repulsive forces that are performed on it by each obstacle in a (2 by 1) matrix 
//we can morph this into a function that also takes in the coordinates of the closest point of an obstacle e.g. double ox, double oy to make it a simple helper 
//function with a more complex structure in the configurationmove ----------------JACOB
std::vector RepulsiveForce(double cx, double cy) {


}
//returns a vecto of size 9 of the jacobian of the control point transposed ---------------PABLO
std::vector TranspoedJacobian(double cx, double cy) {

}


