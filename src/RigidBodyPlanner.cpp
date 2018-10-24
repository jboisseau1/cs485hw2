#include "RigidBodyPlanner.hpp"

#define MIN 0.0019
#define MINTHETA 0.01

RigidBodyPlanner::RigidBodyPlanner(RigidBodySimulator * const simulator)
{
        m_simulator = simulator;
        step = 0.03; //robot step movement size
        threshold = 1.5; //threshold for how far away objects push the robot
        tStep = 0.01; //robot turn step movement size
        escape = false; //is the robot trying to leave local min
        escapecntr = 1501;
        //point where robot is going to leave local min
        escapeX = 0.0;
        escapeY = 0.0;
        //picks where robot leave point should be
        escapeThreshold = 2.0;

}

RigidBodyPlanner::~RigidBodyPlanner(void)
{
        //do not delete m_simulator

}


RigidBodyMove RigidBodyPlanner::ConfigurationMove(void)

{
        RigidBodyMove move;
        const double* vertices = m_simulator->GetRobotVertices();
        int numOfVertices = m_simulator->GetNrRobotVertices();
        //calculate attractive force for each vertices (basically have much the goal is pulling on it)
        std::vector<double> totalForces(numOfVertices, 0);
        std::vector<double> Frep(2, 0);
        std::vector<double> Fatt(2, 0);
        int i;
        for (i = 0; i < numOfVertices; i += 2) {
                if(escape == true) { //if the robot is trying to leave a local min it moves to a temp goal
                        Fatt = AttractiveForceToEscapeGoal(vertices[i], vertices[i+1], escapeX, escapeY);
                }else{ //moving towards the regular goal
                        Fatt = AttractiveForce(vertices[i], vertices[i+1]);
                }
                //adds repulsive force to attractive for total force on robot
                Frep = RepulsiveForce(vertices[i], vertices[i+1]);
                totalForces[i] = (Fatt[0] + Frep[0]);
                totalForces[i + 1] = (Fatt[1] + Frep[1]);


        }

        std::vector<double> jacobians(numOfVertices);
        std::vector<double> temp(2, 0);
        //jacobian A and B for every vertice
        for (i = 0; i < numOfVertices; i+=2) {
                temp = TranspoedJacobianAB(vertices[i], vertices[i + 1]);
                jacobians[i] = temp[0];
                jacobians[i + 1] = temp[1];
        }
        double dx = 0;
        double dy = 0;
        double dtheta = 0;
        //finds u
        for (i = 0; i < numOfVertices; i+=2) {
                dx += totalForces[i];
                dy += totalForces[i + 1];
                dtheta += jacobians[i] * totalForces[i] + jacobians[i+1] * totalForces[i+1];
        }
        //normalizes
        double mag = sqrt(dx*dx + dy * dy + dtheta * dtheta);
        dx = dx / mag;
        dy = dy / mag;
        dtheta = dtheta / mag;

        //make move
        move.m_dx = dx*step;
        move.m_dy = dy*step;
        move.m_dtheta = dtheta*tStep;
        //checks if the robot is in a local min
        if((move.m_dx >= -MIN && move.m_dx <= MIN) && (move.m_dy <= MIN && move.m_dy >= -MIN) && escapecntr > 1500 && (move.m_dtheta <= MINTHETA&&move.m_dtheta >= -MINTHETA)) { //&& move.m_dtheta <= MINTHETA){
                printf("\nGOING INTO ESCAPE MODE\n");
                escapecntr = 0;
                escape = true;
                double goalX =  m_simulator->GetGoalCenterX();
                double goalY = m_simulator->GetGoalCenterY();
                double robotX = m_simulator->GetRobotX();
                double robotY = m_simulator->GetRobotY();
                //sets escape point based on workspace config
                if((goalX<escapeThreshold&&goalX>-escapeThreshold)&&(goalY<escapeThreshold&&goalY>-escapeThreshold)) {
                        if((robotX < escapeThreshold && robotX > -escapeThreshold) || (robotY < escapeThreshold && robotY > -escapeThreshold)) {
                                escapeX = robotY;
                                escapeY = robotX;
                                printf("Goal in center - going to new goal\n");
                        }
                        else{
                                escapeX = -1*robotX;
                                escapeY = robotY;
                                printf("Robot is near zero - going to new goal\n");
                        }
                }
                else{
                        escapeX = -1*goalX;
                        escapeY = goalY;
                        printf("Goal not in center - going to new goal\n");
                }


        }
        //checks if the robot has reached the escape point
        if(escape == true) {
                double robotX = m_simulator->GetRobotX();
                double robotY = m_simulator->GetRobotY();
                double dis = DistanceToObs(robotX, robotY, escapeX, escapeY);
                if(dis < 5.0) {
                        escape = false;
                        printf("Going to original goal\n");
                        escapecntr = 0;
                }
        }
        else{
                escapecntr++;
        }
        return move;
}
//gathers attractive force to the goal
std::vector<double> RigidBodyPlanner::AttractiveForce(double cx, double cy) {
        std::vector<double> returnVector(2, 0);

        returnVector[0] = m_simulator->GetGoalCenterX() - cx;
        returnVector[1] = m_simulator->GetGoalCenterY() - cy;
        double mag  = sqrt(returnVector[0]*returnVector[0] + returnVector[1]*returnVector[1]);
        returnVector[0] /= mag;
        returnVector[1] /= mag;
        return returnVector;
}

//attractive force to the escape point
std::vector<double> RigidBodyPlanner::AttractiveForceToEscapeGoal(double cx, double cy, double destx, double desty) {
        std::vector<double> returnVector(2, 0);
        returnVector[0] = destx - cx;
        returnVector[1] = desty - cy;

        double mag  = sqrt(returnVector[0]*returnVector[0] + returnVector[1]*returnVector[1]);
        returnVector[0] /= mag;
        returnVector[1] /= mag;

        return returnVector;
}

//repulsive force to each obstacle within the threshold radius
std::vector<double> RigidBodyPlanner::RepulsiveForce(double cx, double cy) {
        int numberOfObstacles = m_simulator->GetNrObstacles();
        std::vector<double> returnVector(2, 0);
        Point o_i;
        //dont care if there are no obstacles
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
                                //normalizes
                                mag = sqrt(repx*repx + repy*repy);
                                returnVector[0] += repx/mag;
                                returnVector[1] += repy/mag;

                        }
                        //outside of threshold add nothing
                        else {
                                returnVector[0] += 0;
                                returnVector[1] += 0;
                        }
                }
        }
        return returnVector;
}

//finds A and B from shortcut method to not deal with matrix
std::vector<double> RigidBodyPlanner::TranspoedJacobianAB(double cx, double cy) {
        double theta = m_simulator->GetRobotTheta();
        std::vector<double> TJacobianAB(2, 0);
        TJacobianAB[0]  = (-cx * sin(theta) - cy * cos(theta));
        TJacobianAB[1] =  (cx * cos(theta) - cy * sin(theta));
        return TJacobianAB;
}
//helper to find distance between points
double RigidBodyPlanner::DistanceToObs(double cx, double cy, double ox, double oy) {
        return sqrt((cx-ox)*(cx-ox) + (cy-oy)*(cy-oy));
}
