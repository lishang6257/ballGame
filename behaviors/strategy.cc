#include "naobehavior.h"
#include "../rvdraw/rvdraw.h"

extern int agentBodyType;


// Example usage of the roboviz drawing system and RVSender in rvdraw.cc.
    // Draw agent positions and orientations
    /*
    worldModel->getRVSender()->clearStaticDrawings();
    VecPosition pos = worldModel->getMyPosition();
    VecPosition dir = VecPosition(1,0,0);
    dir = dir.rotateAboutZ(-worldModel->getMyAngDeg());
    worldModel->getRVSender()->drawPoint(pos.getX(), pos.getY(), 10);
    worldModel->getRVSender()->drawLine(pos.getX(), pos.getY(), pos.getX()+dir.getX(), pos.getY()+dir.getY());
    */



/*
 * Real game beaming.
 * Filling params x y angle
 */
void NaoBehavior::beam( double& beamX, double& beamY, double& beamAngle ) {
    beamX = -HALF_FIELD_X + worldModel->getUNum();
    beamY = 0;
    beamAngle = 0;
}


SkillType NaoBehavior::selectSkill() {
    // My position and angle
    //cout << worldModel->getUNum() << ": " << worldModel->getMyPosition() << ",\t" << worldModel->getMyAngDeg() << "\n";

    // Position of the ball
    //cout << worldModel->getBall() << "\n";

    // Example usage of the roboviz drawing system and RVSender in rvdraw.cc.
    // Agents draw the position of where they think the ball is
    // Also see example in naobahevior.cc for drawing agent position and
    // orientation.
    
    worldModel->getRVSender()->clear(); // erases drawings from previous cycle
    // worldModel->getRVSender()->drawPoint("ball", ball.getX(), ball.getY(), 10.0f, RVSender::MAGENTA);
    // worldModel->getRVSender()->drawPoint("dir " + worldModel->getUNum() ,pos.getX(), pos.getY(), 10);

    // VecPosition dir = VecPosition(1,0,0);
    // cout << worldModel->getMyAngDeg() << "myangle \n";
    // dir = dir.rotateAboutZ(-worldModel->getMyAngDeg() + 90);

    // worldModel->getRVSender()->drawLine("My1"
    //     ,me.getX(), me.getY(), (me+dir).getX(), (me+dir).getY(),RVSender::BLUE);
    // worldModel->getRVSender()->drawLine("X" 
    //     ,me.getX(), me.getY(), (me+myXDirection).getX(), (me+myXDirection).getY(),RVSender::PINK);
    //  worldModel->getRVSender()->drawLine("Y" 
    //     ,me.getX(), me.getY(), (me+myYDirection).getX(), (me+myYDirection).getY(),RVSender::ORANGE);
    

    // ### Demo Behaviors ###

    // Walk in different directions
    // return goToTargetRelative(VecPosition(1,0,0), 0); // Forward
    //return goToTargetRelative(VecPosition(-1,0,0), 0); // Backward
    //return goToTargetRelative(VecPosition(0,1,0), 0); // Left
    //return goToTargetRelative(VecPosition(0,-1,0), 0); // Right
    //return goToTargetRelative(VecPosition(1,1,0), 0); // Diagonal
    //return goToTargetRelative(VecPosition(0,1,0), 90); // Turn counter-clockwise
    //return goToTargetRelative(VecPosition(0,-1,0), -90); // Turn clockwise
    //return goToTargetRelative(VecPosition(1,0,0), 15); // Circle

    // Walk to the ball
    // return goToTarget(VecPosition(0,0, 0));

    // goToTarget

    // Turn in place to face ball
    // double distance, angle;
    // getTargetDistanceAndAngle(ball, distance, angle);
    // if (abs(angle) > 10) {
    //   return goToTargetRelative(VecPosition(), angle);
    // } else {
    //   return SKILL_STAND;
    // // }
    // if(worldModel->getUNum() == 1) 
    // return goToTargetRelative(VecPosition(),0);

    // if(worldModel->getUNum() == 2) 
    // return goToTarget(ball);
    

    // //test for rrt
    double searchR = 4,nearR = 0.5;
    if(worldModel->getUNum() == 1) {
    std::vector<VecPosition> pos;
    std::vector<int> num;
    std::vector<double> angle;

    

    double angleSearch = getLimitingAngleForward()*.9;
    getAgentForward(90 , 90,searchR , pos,num,angle);

    std::vector<VecPosition> paths;


    //test circle.haveIntersectionWithLine();
    // SIM::Circle c(me,searchR);
    // SIM::Point2D p1tmp(ball),p2tmp(VecPosition(0,0,0));
    // VecPosition p(0,0,0);
    // worldModel->getRVSender()->drawLine("tLine",ball,p);
    // cout << c.haveIntersectionWithLine(p1tmp,p2tmp) << "\n";




    // buildRRT(paths,me,ball,pos,nearR);
    // VecPosition target = collisionAvoidance(true /*Avoid teamate*/, true /*Avoid opponent*/, false /*Avoid ball*/, .5, .5, ball,
    //                                 false /*fKeepDistance*/);
    // return goToTarget(target);

        VecPosition res = buildDijkstraForLongDistanceAvoid(paths,me,ball,pos,num,searchR,nearR);
        worldModel->getRVSender()->drawPoint("nextPoint",res,15,RVSender::MAGENTA);
        for(unsigned int i = 1;i < paths.size();i ++){
            worldModel->getRVSender()->drawPoint("path",paths[i],15,RVSender::RED);
            worldModel->getRVSender()->drawLine("pathLine",paths[i],paths[i-1],RVSender::RED);
        }
        // if(paths.size() > 0)
        //     return goToTarget( collisionAvoidanceApproach(nearR,0.2, ball,paths[0]) );
        // return goToTarget(ball);
        return goToTarget(res);
    }else{
        worldModel->getRVSender()->drawCircle("safeR",me,nearR);
    }

    // VecPosition p1(0,0,0),p2(0,1,0),p3(1,0,0),p4(sqrt(2),-sqrt(2),0),p5(-sqrt(2),-sqrt(2),0),p6(-sqrt(2),sqrt(2),0),p7(0,-1,0);
    // cout << p1.getAngleBetweenPoints(p2,p6,true) << "\n";
    return SKILL_STAND;

    // Walk to ball while always facing forward
    //return goToTargetRelative(worldModel->g2l(ball), -worldModel->getMyAngDeg());

    // Dribble ball toward opponent's goal
    //return kickBall(KICK_DRIBBLE, VecPosition(HALF_FIELD_X, 0, 0));

    // Kick ball toward opponent's goal
    // return kickBall(KICK_FORWARD, VecPosition(HALF_FIELD_X, 0, 0)); // Basic kick
    //return kickBall(KICK_IK, VecPosition(HALF_FIELD_X, 0, 0)); // IK kick

    // Just stand in place
    // return SKILL_STAND;

    // Demo behavior where players form a rotating circle and kick the ball
    // back and forth
    // return demoKickingCircle();

    // test for split
    // setFallSkillTime(SKILL_FALL_SPLIT,3);
    //return SKILL_FALL_SPLIT;
      // return SKILL_TEXT;




}


/*
 * Demo behavior where players form a rotating circle and kick the ball
 * back and forth
 */
SkillType NaoBehavior::demoKickingCircle() {
    // Parameters for circle
    VecPosition center = VecPosition(-HALF_FIELD_X/2.0, 0, 0);
    double circleRadius = 5.0;
    double rotateRate = 2.5;

    // Find closest player to ball
    int playerClosestToBall = -1;
    double closestDistanceToBall = 10000;
    for(int i = WO_TEAMMATE1; i < WO_TEAMMATE1+NUM_AGENTS; ++i) {
        VecPosition temp;
        int playerNum = i - WO_TEAMMATE1 + 1;
        if (worldModel->getUNum() == playerNum) {
            // This is us
            temp = worldModel->getMyPosition();
        } else {
            WorldObject* teammate = worldModel->getWorldObject( i );
            if (teammate->validPosition) {
                temp = teammate->pos;
            } else {
                continue;
            }
        }
        temp.setZ(0);

        double distanceToBall = temp.getDistanceTo(ball);
        if (distanceToBall < closestDistanceToBall) {
            playerClosestToBall = playerNum;
            closestDistanceToBall = distanceToBall;
        }
    }

    if (playerClosestToBall == worldModel->getUNum()) {
        // Have closest player kick the ball toward the center
        return kickBall(KICK_FORWARD, center);
    } else {
        // Move to circle position around center and face the center
        VecPosition localCenter = worldModel->g2l(center);
        SIM::AngDeg localCenterAngle = atan2Deg(localCenter.getY(), localCenter.getX());

        // Our desired target position on the circle
        // Compute target based on uniform number, rotate rate, and time
        VecPosition target = center + VecPosition(circleRadius,0,0).rotateAboutZ(360.0/(NUM_AGENTS-1)*(worldModel->getUNum()-(worldModel->getUNum() > playerClosestToBall ? 1 : 0)) + worldModel->getTime()*rotateRate);

        // Adjust target to not be too close to teammates or the ball
        target = collisionAvoidance(true /*teammate*/, false/*opponent*/, true/*ball*/, 1/*proximity thresh*/, .5/*collision thresh*/, target, true/*keepDistance*/);

        if (me.getDistanceTo(target) < .25 && abs(localCenterAngle) <= 10) {
            // Close enough to desired position and orientation so just stand
            return SKILL_FALL_SPLIT;
        } else if (me.getDistanceTo(target) < .5) {
            // Close to desired position so start turning to face center
            
            worldModel->getRVSender()->clearStaticDrawings();
            VecPosition pos = worldModel->getMyPosition();
            VecPosition dir = VecPosition(1,0,0);
            dir = dir.rotateAboutZ(-worldModel->getMyAngDeg());
            

            return goToTargetRelative(worldModel->g2l(target), localCenterAngle);
        } else {
            // Move toward target location
            return goToTarget(target);
        }
    }
}


