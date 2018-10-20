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
        double currentX = m_simulator->GetRobotX();
        double currentY = m_simulator->GetRobotY();
        double RF = RepulsiveForce(currentX,currentY);

        printf("RF:%lf\n",RF );
        //add your implementation

        return move;
}
//takes a control point (Cx, Cy) and returns the goals force on it. --------------JIMMY
double RigidBodyPlanner::AttractiveForce(double cx, double cy) {

        return 0.0;



}
//takes a control point (Cx, Cy) and returns a sum of the repulsive forces that
// are performed on it by each obstacle in a (2 by 1) matrix
//we can morph this into a function that also takes in the coordinates of
// the closest point of an obstacle e.g. double ox, double oy to make it a simple helper
//function with a more complex structure in the configurationmove ----------------JACOB
double RigidBodyPlanner::RepulsiveForce(double cx, double cy) {
        int numberOfObstacles = m_simulator->GetNrObstacles(); //includes goal as obstacle so one less
        int numberOfVertices = m_simulator->GetNrRobotVertices();
        /*
         * Vertices are stored consecutively in the vector
         * So the x-coord of the i-th vertex is at index 2 * i
         * and the y-coord of the i-th vertex is at index 2 * i + 1
         */
        const double *vertices = m_simulator->GetRobotVertices();
        double forceTotal = 0.0;
        /**
         *@brief Returns closest point on the i-th circle obstacle to point [x, y]
           Point ClosestPointOnObstacle(const int i, const double x, const double y);
         */
        double scaling_factor = 2.0; //can be configed
        double d_obst = 4.5; //can be configed - threshold to allow the robot to ignore obstacles far away from it


        if(numberOfObstacles>0) {
                for (size_t i = 0; i < numberOfObstacles; i++) { //terates over obstacles -- Urep(q)
                        for (size_t j = 0; j < sizeof(vertices); j++) { //iterates over points on robot
                                if(i%2) {
                                        //adds the local points to the workspace points to get global points
                                        double x = vertices[2*j] + cx;
                                        double y = vertices[2*j+1] + cy;

                                        Point o_i = m_simulator->ClosestPointOnObstacle(i,x,y); //gets point of closest obstacle to global
                                        double Dq = distanceBetweenPoints(x,y,o_i.m_x,o_i.m_y); //distance from robots point to closest obstacle
                                        if(Dq>=d_obst) { //if the obstacle is too far to add to force
                                                forceTotal+=0;
                                        }
                                        else{ // adds force to total
                                                double dist = (1/Dq) - (1/d_obst);
                                                dist *= dist;
                                                forceTotal += (.5*scaling_factor) * dist;
                                        }
                                }
                        }

                }
        }
        return forceTotal;
}
//returns a vecto of size 9 of the jacobian of the control point transposed ---------------PABLO
double RigidBodyPlanner::TranspoedJacobian(double cx, double cy) {

}

double RigidBodyPlanner::distanceBetweenPoints(double x1,double y1,double x2,double y2) {
        double x = x1 - x2;
        double y = y1 - y2;
        double dist;

        dist = (x*x) + (y*y);
        dist = sqrt(dist);
        return dist;
}
