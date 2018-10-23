#include "RigidBodyPlanner.hpp"

RigidBodyPlanner::RigidBodyPlanner(RigidBodySimulator * const simulator)
{
        m_simulator = simulator;
        step = 0.03; //TODO:figure out correct step size
        threshold = 1.5; //TODO: figure out threshold
        tStep = 0.01;

}

RigidBodyPlanner::~RigidBodyPlanner(void)
{
        //do not delete m_simulator

}


RigidBodyMove RigidBodyPlanner::ConfigurationMove(void)

{
        RigidBodyMove move;

        //start by taking all the vertices of our robot (this is a vector)
        const double* vertices = m_simulator->GetRobotVertices();
        printf("numvert %lu\n", sizeof(vertices)/2);
        //calculate attractive force for each vertices (basically have much the goal is pulling on it)
        std::vector<double> totalForces(sizeof(vertices), 0);
        std::vector<double> Frep(2, 0);
        std::vector<double> Fatt(2, 0);
        int i;
        for (i = 0; i < sizeof(vertices); i += 2) {

                Fatt = AttractiveForce(vertices[i], vertices[i+1]);
                Frep = RepulsiveForce(vertices[i], vertices[i+1]);
                totalForces[i] = Fatt[0] + Frep[0];
                totalForces[i + 1] = Fatt[1] + Frep[1];

                // printf("rx:%lf ry:%lf\n", Frep[0] + Frep[1]);
                // printf("ax:%lf ay:%lf\n", Fatt[0] + Fatt[1]);

        }

        // printf("tx:%lf ty:%lf\n", totalForces[0] + totalForces[1]);

        //make helper function to create a seperate transposed jacobian for each vertex (they are all control points i think one long vector of size 9 will work fine for our purposes.)
        std::vector<double> jacobians(sizeof(vertices));
        std::vector<double> temp(2, 0);
        for (i = 0; i < sizeof(vertices); i+=2) {
                temp = TranspoedJacobianAB(vertices[i], vertices[i + 1]);
                jacobians[i] = temp[0];
                jacobians[i + 1] = temp[1];
        }
        double dx = 0;
        double dy = 0;
        double dtheta = 0;
        for (i = 0; i < sizeof(vertices); i+=2) {
                dx += totalForces[i];
                dy += totalForces[i + 1];
                dtheta += jacobians[i] * totalForces[i] + jacobians[i+1] * totalForces[i+1];
        }

        double mag = sqrt(dx*dx + dy * dy + dtheta * dtheta);
        dx = dx / mag;
        dy = dy / mag;
        dtheta = dtheta / mag;


        move.m_dx = dx*step;
        move.m_dy = dy*step;
        move.m_dtheta = dtheta*tStep;

        // printf("dx:%lf dy:%lf\n",move.m_dx,move.m_dy );
        //we'll make a helper function that returns a vector? of size 2 (really just a 2 by 1 matrix)that calculates for a specific point and then loop that over all the control pints
        //calculate the repulsive force of each obstacle on each control point (remember that there is a threshold you must account for - force is 0 outside of that threshold)
        //helper function that claculates force on a point (vecotr?) then a nested for loop to iterate over all obstacles and for every control point.

        //sum the attractive and repulsive forces for each control point seperately
        //matrix multiply the transposed jacobian (3 by 2) with the full force (2 by 1) to make a final vector (3 by 1) containing the movement of the control point in the x , y , and theta
        //loop over all the cotrol points and sum up the movements for each to recieve the final movement and return it as our move
        //
        //
        //add some functionality to get out of local minimums
        //
        //add your implementation

        return move;
}
//takes a control point (Cx, Cy) and returns the goals force on it. --------------JIMMY
std::vector<double> RigidBodyPlanner::AttractiveForce(double cx, double cy) {

        std::vector<double> returnVector(2, 0);

        returnVector[0] = m_simulator->GetGoalCenterX() - cx;
        returnVector[1] = m_simulator->GetGoalCenterY() - cy;

        double mag  = sqrt(returnVector[0]*returnVector[0] + returnVector[1]*returnVector[1]);
        returnVector[0] /= mag;
        returnVector[1] /= mag;

        printf("ax: %lf ay: %lf\n", returnVector[0], returnVector[1]);
        return returnVector;



}
//takes a control point (Cx, Cy) and returns a sum of the repulsive forces that
// are performed on it by each obstacle in a (2 by 1) matrix
//we can morph this into a function that also takes in the coordinates of
// the closest point of an obstacle e.g. double ox, double oy to make it a simple helper
//function with a more complex structure in the configurationmove ----------------JACOB
std::vector<double> RigidBodyPlanner::RepulsiveForce(double cx, double cy) {
        int numberOfObstacles = m_simulator->GetNrObstacles();
        printf("numobs %d\n", numberOfObstacles);
        std::vector<double> returnVector(2, 0);
        /*
         * Vertices are stored consecutively in the vector
         * So the x-coord of the i-th vertex is at index 2 * i
         * and the y-coord of the i-th vertex is at index 2 * i + 1
         */
        /**
         *@brief Returns closest point on the i-th circle obstacle to point [x, y]
           Point ClosestPointOnObstacle(const int i, const double x, const double y);
         */
        Point o_i;

        if(numberOfObstacles>0) {
          int i;
          double mag;
          double repy, repx;
                for (i = 0; i < numberOfObstacles; i++) {
                        o_i = m_simulator->ClosestPointOnObstacle(i, cx, cy);
                        double dist = DistanceToObs(cx,cy,o_i.m_x,o_i.m_y);
                        if ( dist <= threshold) {

                                repx = (cx - o_i.m_x);
                                repy = (cy - o_i.m_y);
                                mag = sqrt(repx*repx + repy*repy);

                                returnVector[0] += repx/mag;
                                returnVector[1] += repy/mag;

                        }
                        else {
                                returnVector[0] += 0;
                                returnVector[1] += 0;
                        }
                }
                if(returnVector[0]!=0 && returnVector[1]!=0){
                  printf("x:%lf, y:%lf\n", returnVector[0], returnVector[1]);

                }
        }
        //if(returnVector[0] != 0 && returnVector[1] != 0){
          //returnVector[0] /= mag;
          //returnVector[1] /= mag;
        //}
        return returnVector;
}
//returns a vector of size 2 of the jacobian of the control point transposed ---------------PABLO
std::vector<double> RigidBodyPlanner::TranspoedJacobianAB(double cx, double cy) {
        double theta = m_simulator->GetRobotTheta();
        std::vector<double> TJacobianAB(2, 0);
        TJacobianAB[0]  = (-cx * sin(theta) - cy * cos(theta));
        TJacobianAB[1] =  (cx * cos(theta) - cy * sin(theta));
        // printf("Jx%lf Jy:%lf\n", TJacobianAB[0],TJacobianAB[1]);
        return TJacobianAB;
}
double RigidBodyPlanner::DistanceToObs(double cx, double cy, double ox, double oy) {
        return sqrt((cx-ox)*(cx-ox) + (cy-oy)*(cy-oy));
}
