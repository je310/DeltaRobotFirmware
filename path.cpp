#include "path.h"

using namespace Eigen;

PathManager::PathManager(octree *oct_ ){
    for(int i = 0; i < PATH_BUFFER_SIZE; i++){
        segment newSeg;
        newSeg.type = invalid;
        newSeg.ID = -1;
        newSeg.complete = 1;
        array[i] = newSeg;
    }
    errorCounter = 0;
    bezierDivs = 32;
    pathCount = 0;

    GunMarkerToBaseCentreM<<1,0,0,0,0,0.5,0,0,0,0,0.5,0,0,0,0,1;
    GunMarkerToBaseCentreInvM<<1,0,0,0,0,0.5,0,0,0,0,0.5,0,0,0,0,1;
    HeadCentreToPitchM<<1,0,0,0.01785,0,1,0,-0.0098,0,0,1,0.00174,0,0,0,1;
    HeadCentreToPitchInvM<<1,0,0,-0.01785,0,1,0,0.0098,0,0,1,-0.00174,0,0,0,1;
    PitchToYawM<<1,0,0,0.012,0,1,0,0,0,0,1,0,0,0,0,1;
    PitchToYawInvM<<1,0,0,-0.012,0,1,0,0,0,0,1,0,0,0,0,1;
    imuToOriginM<<1,0,0,0.085571,0,1,0,0.0048,0,0,1,-1.11022e-16,0,0,0,1;
    imuToOriginInvM<<1,0,0,-0.085571,0,1,0,-0.0048,0,0,1,1.11022e-16,0,0,0,1;
    boardMarkerToCentreM<<0.846575,-0.0875766,0.525016,0.0166051,0.514511,0.38731,-0.765029,-0.0734264,-0.136345,0.917781,0.372946,-0.173941,0,0,0,1;
    boardMarkerToCentreInvM<<0.846575,0.514511,-0.136345,5.23615e-06,-0.0875766,0.38731,0.917781,0.189533,0.525016,-0.765029,0.372946,-2.06365e-05,0,0,0,1;
    yawToTipM<<1,0,0,0.11334,0,1,0,0,0,0,1,0.02456,0,0,0,1;
    yawToTipInvM<<1,0,0,-0.11334,0,1,0,0,0,0,1,-0.02456,0,0,0,1;


    originToCentreM <<1,0,0,0.16,0,1,0,0,0,0,1,0,0,0,0,1;

    GunMarkerToBaseCentre = GunMarkerToBaseCentreM;
    GunMarkerToBaseCentreInv = GunMarkerToBaseCentreInvM;
    HeadCentreToPitch = HeadCentreToPitchM;
    HeadCentreToPitchInv = HeadCentreToPitchInvM;
    PitchToYaw = PitchToYawM;
    PitchToYawInv = PitchToYawInvM;
    imuToOrigin = imuToOriginM;
    imuToOriginInv = imuToOriginInvM;
    boardMarkerToCentre = boardMarkerToCentreM;
    boardMarkerToCentreInv = boardMarkerToCentreInvM;
    yawToTip = yawToTipM;
    yawToTipInv = yawToTipInvM;
    originToCentre = originToCentreM;



    DoriginToCentre = originToCentre * HeadCentreToPitch * PitchToYaw * yawToTip;
    baseToCentre = imuToOrigin * DoriginToCentre;
    oct = oct_;

    pathMode = octreeMode;
}

Eigen::Vector3f PathManager::cubicBezierDerivitive(Eigen::Vector3f start, Eigen::Vector3f CP1,Eigen::Vector3f CP2, Eigen::Vector3f end, float t){
    int order = 3;
    Vector3f sum(0,0,0);
    // i = 0;
    sum+= pow((1-t),2)*(CP1 - start);
    // i = 1;
    sum+= 2*t*(1-t)*(CP2 - CP1);
    // i = 2;
    sum+= pow(t,2)*(end - CP2);
    sum *= 3;

    return sum;
}

Eigen::Vector3f PathManager::quatraticBezier(Eigen::Vector3f start, Eigen::Vector3f CP, Eigen::Vector3f end, float t){
    Vector3f point = (1.0-t)*((1.0-t)*start + t*CP) + t*((1.0-t)*CP + t*end);
    return point;
}

Eigen::Vector3f PathManager::cubicBezier(Eigen::Vector3f start, Eigen::Vector3f CP1,Eigen::Vector3f CP2, Eigen::Vector3f end, float t){
    Vector3f point = (1.0-t)*quatraticBezier(start,CP1,CP2,t) + t*quatraticBezier(CP1,CP2,end,t);
    return point;
}

Eigen::Vector3f PathManager::cubicBezier(segment seg, float t){
    Vector3f start = seg.value1;
    Vector3f CP1 = seg.value2;
    Vector3f CP2 = seg.value3;
    Vector3f endPoint = seg.value4;
    Vector3f normal1 = seg.value5;
    Vector3f normal2 = seg.value6;
    return cubicBezier(start,CP1,CP2,endPoint,t);
}


// This function finds the approximate parametrized value of a particulart point near a bezier curve (needed due to the parametrisation not being linear in time )
float PathManager::lookupCubicBezierT(segment seg,Eigen::Vector3f newPosition, int end, int divs){


    int bestSegment = -1;
    float distance = std::numeric_limits<float>::max();
    int runnerUpSegment = -1;
    float runnerUpDistance = std::numeric_limits<float>::max();
    float smallDot = std::numeric_limits<float>::max();
    //we segment the bezier into line segments, and find the two end points that are both closest and roughly oposites ends of the point.
    if(end ==1){
        int startT = seg.fromStartT*(divs);
        int wasNeg = 0;
        for(int i = startT; i < divs; i++){
            Vector3f pointA = cubicBezier(seg,i* 1.0/divs);
            Vector3f pointB = cubicBezier(seg,(i+1)* 1.0/divs);
            Vector3f toA = pointA - newPosition;
            Vector3f toB = pointB - newPosition;
            float dist = toA.norm() + toB.norm();
            float dot = toA.normalized().dot(toB.normalized());
            if(dot < -0 && dist < distance){
                distance = dist;
                bestSegment = i;
                wasNeg = 1;
                //break;
            }
            //if(dot > 0 && wasNeg) break;
        }
    }
    else{
        int startT = (1.0-seg.fromEndT)*(divs) + 1;
        int wasNeg = 0;
        for(int i =startT-1; i >=  -1 ; i--){
            Vector3f pointA = cubicBezier(seg,i* 1.0/divs);
            Vector3f pointB = cubicBezier(seg,(i+1)* 1.0/divs);
            Vector3f toA = pointA - newPosition;
            Vector3f toB = pointB - newPosition;
            float dist = toA.norm() + toB.norm();
            float dot = toA.normalized().dot(toB.normalized());
            if(dot < -0 && dist < distance){
                distance = dist;
                bestSegment = i;
                wasNeg = 1;
                //break;
            }
            //if(dot > 0 && wasNeg) break;
        }
    }

    // we assume that the point is very close to the line, therfore the ratio of distances from A and B is the fraction traveled allong the line.
    if(bestSegment >= 0){
    Vector3f pointA = cubicBezier(seg,bestSegment* 1.0/divs);
    Vector3f pointB = cubicBezier(seg,(bestSegment+1)* 1.0/divs);
    Vector3f toA = pointA - newPosition;
    Vector3f toB = pointB - newPosition;
    Vector3f AtoB = pointB - pointA;
    Vector3f AtoPoint = newPosition -pointA;
    float dot = AtoB.dot(AtoPoint);
    float lineT =dot/pow(AtoB.norm(),2);
    //lineT = toA.norm()/(AtoB.norm());
    return ((float)bestSegment + lineT)*(1.0/divs);
    }
    else if(divs > 4){
        return lookupCubicBezierT(seg,newPosition, end,divs/2);
    }
//    if(runnerUpSegment >=0){
//        Vector3f pointA = cubicBezier(seg,runnerUpSegment* 1.0/divs);
//        Vector3f pointB = cubicBezier(seg,(runnerUpSegment+1)* 1.0/divs);
//        Vector3f toA = pointA - newPosition;
//        Vector3f toB = pointB - newPosition;
//        float lineT = toA.norm()/(distance);
//        return ((float)runnerUpSegment + lineT)*(1.0/divs);
//    }
    if(end == 1)
        return 1;  // something went wrong, say that we have finished the line !
    if(end ==2)
        return 0;
}
// gets position (from actual start) given a T from 0 to 1;
Vector3f PathManager::getTPos(segment seg, float tIn){
    if(seg.type == circle){
        Vector3f normal = seg.value4;
        Vector3f centre = seg.value1;
        Vector3f start = seg.value2;
        Vector3f end = seg.value3;
        float dot = start.dot(end);
        float det = start[0]*end[1]*normal[2] + end[0]*normal[1]*start[2] + normal[0]*start[1]*end[2] - start[2]*end[1]*normal[0] - end[2]*normal[1]*start[0] - normal[2]*start[1]*end[0];
        float angle = atan2(det, dot);
        angle = angle * tIn;
        AngleAxis<float> t = AngleAxis<float>(angle,normal);
        Vector3f rotated = t * start;
        rotated += centre;
        return rotated;
    }
    if(seg.type == line){
        Vector3f vec = seg.value2 - seg.value1;
        vec = vec * tIn;
        return seg.value1 + vec;
    }
    if(seg.type == bezier){
        Vector3f start = seg.value1;
        Vector3f CP1 = seg.value2;
        Vector3f CP2 = seg.value3;
        Vector3f endPoint = seg.value4;
        Vector3f normal1 = seg.value5;
        Vector3f normal2 = seg.value6;
        float t = tIn;
        Vector3f point = cubicBezier(start,CP1,CP2,endPoint,t);
        return point;
    }
}


Vector3f PathManager::getStartPos(segment seg){
    if(seg.type == circle){
        Vector3f normal = seg.value4;
        Vector3f centre = seg.value1;
        Vector3f start = seg.value2;
        Vector3f end = seg.value3;
        float dot = start.dot(end);
        float det = start[0]*end[1]*normal[2] + end[0]*normal[1]*start[2] + normal[0]*start[1]*end[2] - start[2]*end[1]*normal[0] - end[2]*normal[1]*start[0] - normal[2]*start[1]*end[0];
        float angle = atan2(det, dot);
        angle = angle * seg.fromStartT;
        AngleAxis<float> t = AngleAxis<float>(angle,normal);
        Vector3f rotated = t * start;
        rotated += centre;
        return rotated;
    }
    if(seg.type == line){
        Vector3f vec = seg.value2 - seg.value1;
        vec = vec * seg.fromStartT;
        return seg.value1 + vec;
    }
    if(seg.type == bezier){
        Vector3f start = seg.value1;
        Vector3f CP1 = seg.value2;
        Vector3f CP2 = seg.value3;
        Vector3f endPoint = seg.value4;
        Vector3f normal1 = seg.value5;
        Vector3f normal2 = seg.value6;
        float t = seg.fromStartT;
        Vector3f point = cubicBezier(start,CP1,CP2,endPoint,t);
        return point;
    }
}



Vector3f PathManager::getEndPos(segment seg){
    if(seg.type == circle){
        Vector3f normal = seg.value4;
        Vector3f centre = seg.value1;
        Vector3f start = seg.value2;
        Vector3f end = seg.value3;
        float dot = start.dot(end);
        float det = start[0]*end[1]*normal[2] + end[0]*normal[1]*start[2] + normal[0]*start[1]*end[2] - start[2]*end[1]*normal[0] - end[2]*normal[1]*start[0] - normal[2]*start[1]*end[0];
        float angle = atan2(det, dot);
        angle = angle * seg.fromEndT;
        AngleAxis<float> t = AngleAxis<float>(-angle,normal);
        Vector3f rotated = t * end;
        rotated += centre;
        return rotated;
    }
    if(seg.type == line){
        Vector3f vec = seg.value2 - seg.value1;
        vec = vec * seg.fromEndT;
        return seg.value2 - vec;
    }
    if(seg.type == bezier){
        Vector3f start = seg.value1;
        Vector3f CP1 = seg.value2;
        Vector3f CP2 = seg.value3;
        Vector3f endPoint = seg.value4;
        Vector3f normal1 = seg.value5;
        Vector3f normal2 = seg.value6;
        float t = seg.fromEndT;
        Vector3f point = cubicBezier(start,CP1,CP2,endPoint,1-t);
        return point;
    }
}

int PathManager::isClose(int ID, int end, Eigen::Affine3f currentPos, Eigen::Affine3f targetLocation, float margin){
    segment seg = hashLookupSegment(ID);

    Vector3f location;
    if(end ==1) location = targetLocation * getStartPos(seg);
    if(end ==2) location = targetLocation * getEndPos(seg);

    float dist = (currentPos.translation() - location).norm();
    if(dist < margin) return 1;
    else return 0;

}

Eigen::Vector3f PathManager::getNormal(int ID){
    segment seg = hashLookupSegment(ID);
    if(seg.type == line)   return seg.value3;
    if(seg.type == circle) return seg.value4;
}

Vector3f PathManager::getEndPos(int ID){
    segment seg = hashLookupSegment(ID);
    return getEndPos(seg);
}

Vector3f PathManager::getStartPos(int ID){
    segment seg = hashLookupSegment(ID);
    return getStartPos(seg);
}

Vector3f PathManager::getStartEndPos(segment seg,int end){
    if(end == 1) return getStartPos(seg);
    if(end == 2) return getEndPos(seg);
}

Eigen::Vector3f PathManager::getStartEndPos(int ID,int end){
    segment seg = hashLookupSegment(ID);
    return getStartEndPos(seg,end);
}

float PathManager::getT(segment seg, int end){
    if(end == 1) return seg.fromStartT;
    if(end == 2) return seg.fromEndT;
}

int PathManager::reachable(int ID ,Affine3f currentPos,Affine3f targetPos, Affine3f basePos, float envelopeRadius,Affine3f &pos, int end){
    segment seg = hashLookupSegment(ID);
    return reachable(seg ,currentPos,targetPos, basePos, envelopeRadius,pos, end);
}

//returns whether there is a reachable part of the segment (from either end)
int PathManager::reachable(segment seg ,Affine3f currentPos,Affine3f targetPos, Affine3f basePos, float envelopeRadius,Affine3f &pos, int &end){
    Affine3f startPos, endPos, testPos; // start and end after adding any existing progress.
    int ret = 0;
    int startReachable = 0, endReachable = 0;
    int error = 0;

    endPos = targetPos * Eigen::Translation3f(getEndPos(seg));
    startPos = targetPos * Eigen::Translation3f(getStartPos(seg));

    float startDist;
    float endDist;

    startReachable = getReachability(basePos, currentPos, startPos,testPos, startDist, envelopeRadius);
    //startReachable = getReachabilityFast(basePos, currentPos, startPos,  startDist, envelopeRadius, M_PI*(50.0/180.0));
    endReachable = getReachability(basePos, currentPos, endPos,testPos, endDist, envelopeRadius);
    //endReachable = getReachabilityFast(basePos, currentPos, endPos,  endDist, envelopeRadius, M_PI*(50.0/180.0));

    if( startReachable == 1 && endReachable == 1){
        if(startDist <= endDist){
            end = 1;
            ret = 1;
            pos = startPos;
        }
        else {
            end = 2;
            ret = 1;
            pos = endPos;
        }
    }
    else if( startReachable == 0 && endReachable == 0){
        if(startDist < endDist){
            ret = 0;
            end = 1;
            pos = startPos;
        }
        else {
            ret = 0;
            end = 2;
            pos = endPos;
        }
    }
    else if( startReachable == 1){
        end = 1;
        ret = 1;
        pos = startPos;
    }
    else if( endReachable == 1){
        end = 2;
        ret = 1;
        pos = endPos;
    }
    else{
        error++;
    }
    return ret;

}

//returns whether there is a reachable part of the segment (from either end)
int PathManager::thisEndReachable(int ID ,Eigen::Affine3f currentPos,Eigen::Affine3f targetPos, Eigen::Affine3f basePos, float envelopeRadius,Eigen::Affine3f &pos, int end){
    segment seg = hashLookupSegment(ID);
    return thisEndReachable(seg , currentPos,targetPos,basePos, envelopeRadius,pos,end);
}

int PathManager::thisEndReachable(segment seg ,Eigen::Affine3f currentPos,Eigen::Affine3f targetPos, Eigen::Affine3f basePos, float envelopeRadius,Eigen::Affine3f &pos, int end){
    Affine3f startPos, endPos, testPos; // start and end after adding any existing progress.
    int  reach = 0;

    if(end == 2){
        float endDist;
        pos = targetPos * Eigen::Translation3f(getEndPos(seg));
        reach = getReachability(basePos, currentPos, pos,testPos, endDist, envelopeRadius);

    }
    else if(end == 1){
        float startDist;
        pos = targetPos * Eigen::Translation3f(getStartPos(seg));
        reach = getReachability(basePos, currentPos, pos,testPos, startDist, envelopeRadius);
    }
    return reach;

}
void PathManager::getEulerYPREigen(Eigen::Matrix3f mat, float& yaw, float& pitch, float& roll)
{
     unsigned int solution_number = 1;
    Eigen::Vector3f m_el[3];
   m_el[0] =  mat.row(0);
   m_el[1] =  mat.row(1);
   m_el[2] =  mat.row(2);
    struct Euler
    {
        float yaw;
        float pitch;
        float roll;
    };

    Euler euler_out;
    Euler euler_out2; //second solution
    //get the pointer to the raw data

    // Check that pitch is not at a singularity
    // Check that pitch is not at a singularity
    if (fabs(m_el[2].x()) >= 1)
    {
        euler_out.yaw = 0;
        euler_out2.yaw = 0;

        // From difference of angles formula
        if (m_el[2].x() < 0)  //gimbal locked down
        {
          float delta = atan2(m_el[0].y(),m_el[0].z());
            euler_out.pitch = M_PI / 2.0;
            euler_out2.pitch = M_PI / 2.0;
            euler_out.roll = delta;
            euler_out2.roll = delta;
        }
        else // gimbal locked up
        {
          float delta = atan2(-m_el[0].y(),-m_el[0].z());
            euler_out.pitch = -M_PI / 2.0;
            euler_out2.pitch = -M_PI / 2.0;
            euler_out.roll = delta;
            euler_out2.roll = delta;
        }
    }
    else
    {
        euler_out.pitch = - asin(m_el[2].x());
        euler_out2.pitch = M_PI - euler_out.pitch;

        euler_out.roll = atan2(m_el[2].y()/cos(euler_out.pitch),
            m_el[2].z()/cos(euler_out.pitch));
        euler_out2.roll = atan2(m_el[2].y()/cos(euler_out2.pitch),
            m_el[2].z()/cos(euler_out2.pitch));

        euler_out.yaw = atan2(m_el[1].x()/cos(euler_out.pitch),
            m_el[0].x()/cos(euler_out.pitch));
        euler_out2.yaw = atan2(m_el[1].x()/cos(euler_out2.pitch),
            m_el[0].x()/cos(euler_out2.pitch));
    }

    if (solution_number == 1)
    {
        yaw = euler_out.yaw;
        pitch = euler_out.pitch;
        roll = euler_out.roll;
    }
    else
    {
        yaw = euler_out2.yaw;
        pitch = euler_out2.pitch;
        roll = euler_out2.roll;
    }
}

int PathManager::getReachability(Eigen::Affine3f currentBasePos, Eigen::Affine3f currentHeadPos, Eigen::Affine3f targetPos,Eigen::Affine3f &posOut,  float &distance, float evelope){


    int error = 0;
    int inside = 0;
         Eigen::Affine3f origin = currentBasePos* imuToOrigin;
    Eigen::Affine3f originToTarget = origin.inverse((Eigen::TransformTraits)1) * targetPos;
    float r , p , y;
    getEulerYPREigen(originToTarget.rotation(), y, p, r);
    //make separate rotation matrix
    float yawAng =  y;
    float pitchAng = p;

    Eigen::Affine3f yaw = Eigen::Translation3f(0,0,0) * Eigen::AngleAxisf(yawAng, Eigen::Vector3f::UnitZ());
    Eigen::Affine3f pitch = Eigen::Translation3f(0,0,0) * Eigen::AngleAxisf(pitchAng, Eigen::Vector3f::UnitY());
   Eigen::Affine3f roll = Eigen::Translation3f(0,0,0) * Eigen::AngleAxisf(r, Eigen::Vector3f::UnitX());

    Eigen::Affine3f kinOut = origin.inverse((Eigen::TransformTraits)1)
                            * targetPos
                            * roll.inverse((Eigen::TransformTraits)1)
                            * yawToTipInv
                            * yaw.inverse((Eigen::TransformTraits)1)
                            * PitchToYawInv
                            * pitch.inverse((Eigen::TransformTraits)1)
                            * HeadCentreToPitchInv;
    // Eigen::Matrix4f originM = currentPos.matrix() * imuToOriginM;
    // Eigen::Matrix4f targetM = targetPos.matrix();
    // Eigen::Matrix4f kinOutMat = origin.inverse()
    //                             * targetM
    //                             * yawToTipInvM
    //                             * yaw.matrix().inverse()
    //                             * PitchToYawInvM
    //                             * pitch.matrix().inverse()
    //                             * HeadCentreToPitchInvM;
    // kinOut = kinOutMat;
    Eigen::Vector3f kinTrans = kinOut.translation();

    distance = (targetPos.translation() - currentHeadPos.translation()).norm();

    Eigen::Vector3f centre(0.16,0,0);
    float range = 0.07;
    float dif = (kinTrans - centre).norm();
    if(dif < evelope && 180*fabs(yawAng)/M_PI < 60 && 180*fabs(pitchAng)/M_PI < 60){
        inside = 1;
        posOut = currentHeadPos;
    }
    else{
        inside = 0;
        Eigen::Vector3f centreToTarget;
                centreToTarget = kinTrans - centre;
                kinTrans = centreToTarget.normalized()*range + centre;
                kinOut.translation() = kinTrans;
        posOut = origin * kinOut * HeadCentreToPitch * pitch * PitchToYaw * yaw * yawToTip * roll;
    }
    return inside;
}

int PathManager::getReachabilityFast(Eigen::Affine3f currentBasePos, Eigen::Affine3f currentHeadPos, Eigen::Affine3f targetPos,  float &distance, float evelope, float minMaxAngle){


    int error = 0;
    int inside = 0;
         Eigen::Affine3f origin = currentBasePos* imuToOrigin;
    Eigen::Affine3f originToTarget = origin.inverse((Eigen::TransformTraits)1) * targetPos;
    Eigen::Affine3f centre = origin * DoriginToCentre;
    float r , p , y;
    getEulerYPREigen(originToTarget.rotation(), y, p, r);

    Eigen::Vector3f centreToTarget = targetPos.translation() - (centre).translation();

    distance = (currentHeadPos.translation() - targetPos.translation()).norm();

    float dFromCentre = (centre.translation()- targetPos.translation()).norm();

    if(fabs(r) < minMaxAngle && fabs(p) < minMaxAngle && dFromCentre < evelope) inside = 1;
    else inside = 0;

    return inside;

}

int PathManager::stepTime(int ID,float deltaT, int end){ // also returns 1 if the line is now complete.
    if(ID >= 0){
        int hID = hashLookup(ID);
        return stepTime(array[hID],deltaT,end);
    }
    if(ID == -2){
        return stepTime(travelSeg,deltaT,end);
    }
}
int PathManager::stepTime(segment &seg,float deltaT, int end){
    float fullTime = 0;
    if(seg.type ==line){
        Vector3f vec = seg.value2 - seg.value1;
        fullTime = vec.norm() / seg.speed;
    }
    if(seg.type ==circle){
        Vector3f normal = seg.value4;
        Vector3f centre = seg.value1;
        Vector3f start = seg.value2;
        Vector3f finish = seg.value3;
        float dot = start.dot(finish);
        float det = start[0]*finish[1]*normal[2] + finish[0]*normal[1]*start[2] + normal[0]*start[1]*finish[2] - start[2]*finish[1]*normal[0] - finish[2]*normal[1]*start[0] - normal[2]*start[1]*finish[0];
        float angle = atan2(det, dot);
        float radius = start.norm();
        float arc = angle * radius;
        fullTime = fabs(arc) / seg.speed;

    }
    if(seg.type == bezier){
        //bezier is different due to the parameterisation not being proportional to time.
        //we must approximate the bezier such that we can work in a value representing time.
        Vector3f start = seg.value1;
        Vector3f CP1 = seg.value2;
        Vector3f CP2 = seg.value3;
        Vector3f endPoint = seg.value4;
        Vector3f normal1 = seg.value5;
        Vector3f normal2 = seg.value6;
        Vector3f currentPosition = getStartEndPos(seg,end);
        Vector3f currentGradient;
        if(end ==1){
            currentGradient = cubicBezierDerivitive(start,CP1,CP2,endPoint,seg.fromStartT);
        }
        if(end ==2){
            currentGradient = -cubicBezierDerivitive(start,CP1,CP2,endPoint,1 - seg.fromEndT);
        }
        Vector3f deltaMove = seg.speed*deltaT* currentGradient.normalized();
        Vector3f newPosition = currentPosition + deltaMove;

        float newT = lookupCubicBezierT(seg,newPosition, end,bezierDivs);
        if(end ==1){
            seg.fromStartT = newT;
        }
        if(end ==2){
            seg.fromEndT = 1 - newT;
        }

    }

    if(seg.type ==line || seg.type == circle){
        float timeRatio ;
        if(fullTime > 0){
            timeRatio = deltaT / fullTime ;
        }
        else{
            seg.complete = 1;
            seg.fromStartT = 1;
            return 1;
        }
        if(end == 1){
           seg.fromStartT += timeRatio;
        }
        if(end == 2){
            seg.fromEndT += timeRatio;
        }

    }

    if(seg.fromEndT + seg.fromStartT  >= 1.0){
        seg.complete = 1;
        return 1;
    }
    return 0;
}

Eigen::Vector3f PathManager::getPos(int ID, int end, Eigen::Affine3f targetLocation){
    segment seg = hashLookupSegment(ID);
    if(ID == -1) return targetLocation.translation();
    if(end == 1){
        return targetLocation * getStartPos(seg);
    }
    if(end == 2){
        return targetLocation * getEndPos(seg);
    }
}
int PathManager::getClosestPathFast(Eigen::Affine3f currentPos,Eigen::Affine3f baseLocation,Eigen::Affine3f targetLocation, float envelopeRadius, int &end, bool incBounds, std::vector<octree::dataPtr> &list){
    if(!incBounds){
        list = oct->getNnearest(targetLocation.inverse() * currentPos.translation(),1,10000000);
    }
    else{

        std::vector<octree::dataPtr> listValid;
        listValid = oct->getNnearest((targetLocation.inverse() * baseLocation* baseToCentre).translation(),pathCount,envelopeRadius);
        list = oct->getNnearest(targetLocation.inverse() * currentPos.translation(),pathCount,10000000);
        if(list.size() >0 && listValid.size() > 0){
            int i  = 0;
        for(i = 0; i < list.size(); i++){
            bool escape = false;
            for(int j = 0; j < listValid.size(); j++){
                if(list[i]==listValid[j]){
                    escape = true;
                    break;
                }
            }
            if(escape) break;
        }

            segment seg  = hashLookupSegment(list[i].data);
            if(getStartPos(seg) == list[0].point) end = 1;
            if(getEndPos(seg) == list[0].point) end = 2;
            return list[i].data;

        }


    }
    if(list.size() >0){
        segment seg  = hashLookupSegment(list[0].data);
        if(getStartPos(seg) == list[0].point) end = 1;
        if(getEndPos(seg) == list[0].point) end = 2;
        return list[0].data;
    }
    return -1;
}

int PathManager::getClosestPathBiased(Eigen::Affine3f currentPos,Eigen::Affine3f baseLocation,Eigen::Affine3f targetLocation, float envelopeRadius, int &end, bool incBounds,float biasGain,std::vector<octree::dataPtr> &list){
    Eigen::Vector3f baseInOct =  (targetLocation.inverse() * baseLocation* baseToCentre).translation();
    if(!incBounds){
        list = oct->getNnearestBiased(targetLocation.inverse() * currentPos.translation(),baseInOct,1,10000000,biasGain);
    }
    else{

        std::vector<octree::dataPtr> listValid;
        listValid = oct->getNnearest(baseInOct,pathCount,envelopeRadius);
        list = oct->getNnearestBiased(targetLocation.inverse() * currentPos.translation(),baseInOct,pathCount,10000000,biasGain);
        if(list.size() >0 && listValid.size() > 0){
            int i  = 0;
        for(i = 0; i < list.size(); i++){
            bool escape = false;
            for(int j = 0; j < listValid.size(); j++){
                if(list[i]==listValid[j]){
                    escape = true;
                    break;
                }
            }
            if(escape) break;
        }

            segment seg  = hashLookupSegment(list[i].data);
            if(getStartPos(seg) == list[0].point) end = 1;
            if(getEndPos(seg) == list[0].point) end = 2;
            return list[i].data;

        }


    }
    if(list.size() >0){
        segment seg  = hashLookupSegment(list[0].data);
        if(getStartPos(seg) == list[0].point) end = 1;
        if(getEndPos(seg) == list[0].point) end = 2;
        return list[0].data;
    }
    return -1;
}

int PathManager::getClosestPathBiasedWithGlobal(Eigen::Affine3f currentPos,Eigen::Affine3f baseLocation,Eigen::Affine3f targetLocation,std::vector<Eigen::Vector3f> globalPoints, float envelopeRadius, int &end, bool incBounds,float biasGain, float GlobalGainTotal,std::vector<octree::dataPtr> &list){

    Eigen::Vector3f baseInOct =  (targetLocation.inverse() * baseLocation* baseToCentre).translation();
    std::vector<float> biasGainVec(globalPoints.size()+1);
    std::vector<Eigen::Vector3f> biasPoints(globalPoints.size()+1);
    biasGainVec[0] = biasGain;
    biasPoints[0]  = baseInOct;
    float singleGain = GlobalGainTotal/ globalPoints.size();
    for(int i = 1; i < biasGainVec.size(); i++){
        biasGainVec[i] = singleGain;
        biasPoints[i] = globalPoints[i-1];
    }
    if(!incBounds){
        list = oct->getNnearestBiasedVector(targetLocation.inverse() * currentPos.translation(),biasPoints,pathCount,10000000,biasGainVec);
    }
    else{

        std::vector<octree::dataPtr> listValid;
        listValid = oct->getNnearest(baseInOct,pathCount,envelopeRadius);
        list = oct->getNnearestBiasedVector(targetLocation.inverse() * currentPos.translation(),biasPoints,pathCount,10000000,biasGainVec);
        if(list.size() >0 && listValid.size() > 0){
            int i  = 0;
        for(i = 0; i < list.size(); i++){
            bool escape = false;
            for(int j = 0; j < listValid.size(); j++){
                if(list[i]==listValid[j]){
                    escape = true;
                    break;
                }
            }
            if(escape) break;
        }

            segment seg  = hashLookupSegment(list[i].data);
            if(getStartPos(seg) == list[0].point) end = 1;
            if(getEndPos(seg) == list[0].point) end = 2;
            return list[i].data;

        }


    }
    if(list.size() >0){
        segment seg  = hashLookupSegment(list[0].data);
        if(getStartPos(seg) == list[0].point) end = 1;
        if(getEndPos(seg) == list[0].point) end = 2;
        return list[0].data;
    }
    return -1;
}

int PathManager::getClosestPath(Eigen::Affine3f currentPos,Eigen::Affine3f baseLocation,Eigen::Affine3f targetLocation, float envelopeRadius, int &end){
    int closestIn = -1;
    int closestOut = -1;
    int closestInEnd = -1;
    int closestOutEnd = -1;
    float distanceIn = std::numeric_limits<float>::max();

    float distanceOut = std::numeric_limits<float>::max();
    for(int i = 0 ; i < PATH_BUFFER_SIZE; i++){
        Affine3f pos;
        int end = -1;
        if(array[i].complete == 0 && array[i].type != invalid){
            int isIn = reachable(array[i],currentPos,targetLocation, baseLocation,envelopeRadius,pos, end);
            if(((currentPos.translation()-pos.translation()).norm() < distanceIn) || ((currentPos.translation()-pos.translation()).norm() < distanceOut)){
            switch(isIn){
            case 1:
                if( (currentPos.translation()-pos.translation()).norm() < distanceIn){
                    distanceIn = (currentPos.translation()-pos.translation()).norm();
                    closestIn = i;
                    closestInEnd = end;
                }
                break;

            case 0:
                if( (currentPos.translation()-pos.translation()).norm() < distanceOut){
                    distanceOut = (currentPos.translation()-pos.translation()).norm();
                    closestOut = i;
                    closestOutEnd = end;

                }
                break;

            case -1:
                // this indicates that this array element is empty and shouldl not be considered.
                break;

            default:
                break;
                // this indicates an error
            }
            }
        }
    }
    if(closestIn != -1){
        end = closestInEnd;
        return array[closestIn].ID;
    }
    if(closestOut != -1){
        end = closestOutEnd;
        return array[closestOut].ID;
    }
    return -1;
}

void PathManager::constrainHead(Eigen::Affine3f &currentPos, Eigen::Affine3f basePos, float envelope){
    Eigen::Affine3f posOut;
    float distance;
    int reachable  =  getReachabilityFast(basePos, currentPos, currentPos,  distance, envelope,M_PI*(50.0/180.0));
    if(reachable) return;
    getReachability(basePos,  currentPos, currentPos,posOut,distance,envelope);
    currentPos = posOut;


}

int PathManager::pathType(int ID){
    segment seg = hashLookupSegment(ID);
    return seg.type;
}

int PathManager::setupTravel(Affine3f start, Eigen::Vector3f end, Eigen::Affine3f targetLocation , Eigen::Vector3f normal, float speed, int toID, int toEnd){
    travelSeg.ID = -2;
    travelSeg.complete = 0;
    travelSeg.fromEndT = 0;
    travelSeg.fromStartT = 0;
    travelSeg.speed = speed;
    travelSeg.type = line;
    for(int i = 1; i < 4; i++){
        travelSeg.startHints[i] = std::numeric_limits<uint16_t>::max();
        travelSeg.endHints[i] = std::numeric_limits<uint16_t>::max();
    }
    travelSeg.endHintToEnd = 0;
    travelSeg.startHintToEnd = 0 ;
    travelSeg.startHints[0] = toID;
    travelSeg.endHints[0]  = toID;
    if(toEnd == 2){
        setBit(travelSeg.endHintToEnd,0,1);
    }
    else{
        setBit(travelSeg.endHintToEnd,0,0);
    }
    //travelSeg.value1 = (start * targetLocation.inverse()).translation();
    travelSeg.value1 = (targetLocation.inverse()*start).translation() ;
    travelSeg.value2 = end;
    travelSeg.value3 = normal;
    return 1;
}

uint32_t PathManager::hash(uint32_t x) {
    return (117*x)%PATH_BUFFER_SIZE;
}

int PathManager::addNewPath(segment aPath){
    pathCount++;
    int h = hashAllocate(aPath.ID);
    if(h >= 0){
        array[h] = aPath;

        octree::dataPtr dataStart;
        dataStart.data = aPath.ID;
        dataStart.point = getStartPos(dataStart.data);

        octree::dataPtr dataEnd;
        dataEnd.data = aPath.ID;
        dataEnd.point = getEndPos(dataEnd.data);

        oct->insert(dataStart);
        oct->insert(dataEnd);

        return -1;
    }
    return 0;
}

int PathManager::getSpareSpace(){
    int count = 0;
    for(int i = 0 ; i < PATH_BUFFER_SIZE; i++){
        if(array[i].type == invalid) count++;
    }
    return count;
}

int PathManager::isComplete(){
    for(int i = 0 ; i < PATH_BUFFER_SIZE; i++){
        if(array[i].complete == 0){
            return 0;
        }
    }
    return 1;
}

int PathManager::isComplete(int ID){
    segment seg = hashLookupSegment(ID);
    if(seg.fromEndT + seg.fromStartT >= 1) seg.complete = 1;
    return seg.complete;
}

int PathManager::setComplete(int ID){
    if(ID >= 0){
        int h = hashLookup(ID);
        if(h != -1){
            array[h].complete = 1;
            return 0;
        }
    }
    if(ID == -2){
        travelSeg.complete = 1;
    }
    return -1; // this is not an ID in the list.
}

int PathManager::setNotComplete(){
    for(int i = 0; i < PATH_BUFFER_SIZE; i++){
        if(array[i].type == invalid) continue;
        array[i].complete = 0;
        array[i].fromStartT = 0;
        array[i].fromEndT = 0;
    }
    return 0;
}


int PathManager::markInvalid(int ID){
    int h = hashLookup(ID);
    if(h != -1){
        array[h].type = invalid;
        return 0;
    }
    return -1; // this is not an ID in the list.
}

int PathManager::hashLookup(int ID){
    int notFound = PATH_BUFFER_SIZE;
    uint32_t h = hash(ID);
    while(notFound){
        if(array[h%PATH_BUFFER_SIZE].ID == ID){
            return h%PATH_BUFFER_SIZE;
        }
        else{
            h++;
        }

        notFound--;
    }
    return -1; // seems it is not in there!
}

segment PathManager::hashLookupSegment(int ID){
    if(ID >=0){
        int hID = hashLookup(ID);
        if(hID >= 0) return array[hID];
    }
    if(ID == -2) return travelSeg;
    else{
        errorCounter ++;
        return travelSeg;
    }

}

int PathManager::hashAllocate(int ID){
    int notFound = PATH_BUFFER_SIZE;
    uint32_t h = hash(ID);
    while(notFound){
        if(array[h%PATH_BUFFER_SIZE].type == invalid || array[h%PATH_BUFFER_SIZE].ID == ID){
            return h%PATH_BUFFER_SIZE;
        }
        else{
            h++;
        }

        notFound--;
    }
    return -1; // there is no space left!
}

int  PathManager::getBit(uint32_t val, int bit){
    return (val & ( 1 << bit )) >> bit;
}

void PathManager::setBit(uint32_t &val, int bit, int to){
    uint32_t mask = 1 << bit;
    if(to == 0){

        val &= ~mask;
    }
    if(to == 1){
        val |= mask;
    }
}

void PathManager::addHintsClosest(){
    for(int i = 0; i < PATH_BUFFER_SIZE; i++){
        if(array[i].type == invalid) continue;
        int closestEnd[HINT_NUMBER];
        int closestStart[HINT_NUMBER] ;
        float valuesEnd[HINT_NUMBER] ;
        float valuesStart[HINT_NUMBER] ;

        for(int t = 0; t <HINT_NUMBER; t++ ){
            closestEnd[t] = std::numeric_limits<u_int16_t>::max();
            closestStart[t] =std::numeric_limits<u_int16_t>::max();
            valuesEnd[t] =std::numeric_limits<float>::max();
            valuesStart[t] = std::numeric_limits<float>::max();

        }
        uint32_t endHintToEnd = 0;
        uint32_t startHintToEnd = 0;
        for(int j = 0; j < PATH_BUFFER_SIZE; j ++){
            if(array[j].type == invalid) continue;
            if(j != i){
               int theEndStart = 0;
               int theEndEnd = 0;
               float startDist = (getStartPos(array[i]) - getStartPos(array[j])).norm();
               float startDist2 = (getStartPos(array[i]) - getEndPos(array[j])).norm();
               if (startDist2 < startDist){
                   theEndStart = 1;
                   startDist = startDist2;
               }
               float endDist = (getEndPos(array[i]) - getStartPos(array[j])).norm();
               float endDist2 = (getEndPos(array[i]) - getEndPos(array[j])).norm();
               if (endDist2 < endDist){
                   theEndEnd  = 1;
                   endDist = endDist2;
               }

               int ranking = HINT_NUMBER;
               for(; ranking > 0; ranking --){
                   if(valuesEnd[ranking - 1] < endDist) break;
               }
               if (ranking != HINT_NUMBER){
                   for(int pointer = HINT_NUMBER; pointer> ranking; pointer --){
                       valuesEnd[pointer-1] = valuesEnd[pointer-2];
                       closestEnd[pointer-1] = closestEnd[pointer-2];
                       setBit(endHintToEnd,pointer-1,getBit(endHintToEnd,pointer-2));
                   }
                   closestEnd[ranking] = j;
                   setBit(endHintToEnd,ranking,theEndEnd);
                   valuesEnd[ranking] = endDist;
               }
               ranking = HINT_NUMBER;
               for(; ranking > 0; ranking --){
                   if(valuesStart[ranking - 1] < startDist) break;
               }
               if (ranking != HINT_NUMBER){
                   for(int pointer = HINT_NUMBER; pointer> ranking; pointer --){
                       valuesStart[pointer-1] = valuesStart[pointer-2];
                       closestStart[pointer-1] = closestStart[pointer-2];
                       setBit(startHintToEnd,pointer-1,getBit(startHintToEnd,pointer-2));
                   }
                   closestStart[ranking] = j;
                   setBit(startHintToEnd,ranking,theEndStart);
                   valuesStart[ranking] = startDist;
               }

            }

        }
        for(int m = 0; m < HINT_NUMBER; m++){
            if(array[closestStart[m]].ID > this->pathCount || array[closestStart[m]].ID <0){
                array[i].startHints[m] = -1;
            }
            else array[i].startHints[m] = array[closestStart[m]].ID;
            if(array[closestEnd[m]].ID > this->pathCount ||array[closestEnd[m]].ID <0){
               array[i].endHints[m] = -1;
            }
            array[i].endHints[m] = array[closestEnd[m]].ID;
        }
        array[i].startHintToEnd = startHintToEnd;
        array[i].endHintToEnd= endHintToEnd;

    }
}

int PathManager::getBestHint(int ID,int &end){
    if(ID >=0 || ID == -2){
        segment thisSeg = hashLookupSegment(ID);
        uint32_t hintEnds;
        if(thisSeg.fromEndT + 0.01 >= 1.0) hintEnds =thisSeg.startHintToEnd;
        if(thisSeg.fromStartT + 0.01 >= 1.0) hintEnds = thisSeg.endHintToEnd;
        for( int i = 0 ; i < HINT_NUMBER; i++){
            segment ourSeg;
            if(end ==2) ourSeg = hashLookupSegment(thisSeg.startHints[i]);
            if(end ==1) ourSeg = hashLookupSegment(thisSeg.endHints[i]);
            int particularEnd = getBit(hintEnds,i);
            if(particularEnd == 0){
                if(ourSeg.fromStartT == 0 && !ourSeg.complete){
                    end = 1;
                    return ourSeg.ID;
                }
            }
            if(particularEnd == 1){
                if(ourSeg.fromEndT == 0 && !ourSeg.complete){
                    end = 2;
                    return ourSeg.ID;
                }
            }


        }

    }
    return -1;
}
int  PathManager::finishedOnEnd(int ID){
    segment thisSeg = hashLookupSegment(ID);
    float range = 0.01;

    if(thisSeg.fromEndT + range >= 1 || thisSeg.fromStartT + range >= 1){
        return 1;
    }
    return 0;
}

std::vector<segment> PathManager::returnVector(){
    std::vector<segment> list;
    for(int i = 0; i < PATH_BUFFER_SIZE; i++){
        if(array[i].type == invalid) continue;
        list.push_back(array[i]);
    }
    return list;
}
#ifdef DESKTOP
cv::Mat  PathManager::getHeatMap(float rad,Eigen::Vector3f centre,Eigen::Vector3f upDir ,Eigen::Vector3f sideDir){
    int rows = upDir.norm() / rad;
    int cols = sideDir.norm() / rad;
    cv::Mat image = cv::Mat(rows,cols,CV_32FC1,cv::Scalar(0));
    for(int rowCount = 0; rowCount < rows; rowCount ++){
        for(int colCount = 0; colCount < cols; colCount ++){
            Eigen::Vector3f position = centre + ((rowCount - 0.5*(rows-1))*upDir/(rows-1))  + ((colCount - 0.5*(cols-1))*sideDir/(cols-1));
            float lengthToComplete = getLengthToCompleteFast(rad, position);
           // float lengthToComplete2 = getLengthToComplete(rad, position);
            image.at<float>(rowCount, colCount) = 0.02*lengthToComplete;
        }
    }
    return image;
}
#endif
float PathManager::getLengthToComplete(float rad, Eigen::Vector3f position){
    float distAcc = 0;
    for(int i = 0; i < PATH_BUFFER_SIZE; i ++ ){
        if(array[i].ID == invalid) continue;
        Eigen::Vector3f  startPos = getStartPos(array[i]);
        Eigen::Vector3f  endPos = getEndPos(array[i]);
        float startDist = (startPos - position).norm();
        float endDist = (endPos - position).norm();
        if(endDist > 2.0*rad && startDist > 2.0*rad) continue;
        float dist = pathInCircle(array[i], rad, position);
        distAcc += dist;

    }
    return distAcc;
}
bool checked[PATH_BUFFER_SIZE];
float PathManager::getLengthToCompleteFast(float rad, Eigen::Vector3f position){
    float distAcc = 0;
    std::vector<octree::dataPtr> list = oct->getNnearest(position, INT_MAX,1.44*rad);

    for(int i = 0; i < PATH_BUFFER_SIZE; i++) checked[i] = false;
    for(int i = 0; i < list.size(); i ++ ){
        int hID = hashLookup(list[i].data);
        segment thisSeg = array[hID];
        if(thisSeg.type == invalid  || checked[hID] == true) continue;
        checked[hID] = true;
        if(thisSeg.complete == true) continue;
        Eigen::Vector3f  startPos = getStartPos(thisSeg);
        Eigen::Vector3f  endPos = getEndPos(thisSeg);
        float startDist = (startPos - position).norm();
        float endDist = (endPos - position).norm();
        if(endDist > 2.0*rad && startDist > 2.0*rad) continue;
        float dist = pathInCircle(thisSeg, rad, position);
        distAcc += dist;

    }
    return distAcc;
}

float PathManager::pathInCircle(segment seg,float  rad, Eigen::Vector3f position){
    float increments = 10;
    float tVal = 1  - seg.fromEndT -  seg.fromStartT;
    float tInc = tVal / increments;
    float distanceAcc = 0;
    for(int i = 0; i < increments-1; i ++){
        Eigen::Vector3f startSeg = getTPos(seg,seg.fromStartT + i*tInc);
        Eigen::Vector3f endSeg = getTPos(seg,seg.fromStartT + (i+1)*tInc);
        float startDist = (startSeg - position).norm();
        float endDist = (endSeg - position).norm();
        float interDist =  (startSeg - endSeg).norm();
        float startMarg = startDist - rad;
        float endMarg = endDist -rad;
        float totMarg = startMarg + endMarg;
        if(startDist < rad && endDist < rad){
            distanceAcc += interDist;
            continue;
        }
        else if(startDist < rad){
            distanceAcc += (startMarg * interDist) / totMarg;
            continue;

        }
        else if(endDist < rad){
            distanceAcc += (endMarg * interDist) / totMarg;
            continue;
        }

    }
    return distanceAcc;

}

bool PathManager::rebuildReset(){
    oct->reset();

    for(int i = 0; i < PATH_BUFFER_SIZE; i++){
        if(array[i].type != invalid){
            array[i].complete = false;
            array[i].fromEndT = 0;
            array[i].fromStartT = 0;
            octree::dataPtr dataStart;
            dataStart.data = array[i].ID;
            dataStart.point = getStartPos(dataStart.data);

            octree::dataPtr dataEnd;
            dataEnd.data = array[i].ID;
            dataEnd.point = getEndPos(dataEnd.data);


            oct->insert(dataStart);
            oct->insert(dataEnd);
        }
    }
}

std::vector<std::pair<int,int> > PathManager::makeSeriesPrediction(Eigen::Affine3f currentPos,Eigen::Affine3f baseLocation, Eigen::Affine3f targetLocation,float findHorizon,float envRad, int N){
    std::vector<std::pair<int,int> > IDList;
    if(pathMode == inOrder){
        int i = 0;
        while(IDList.size() != N){
            segment seg = hashLookupSegment(i);
            if(!seg.complete){
                std::pair<int,int> thisPair;
                thisPair.first = i;
                thisPair.second = 1;
                IDList.push_back(thisPair);
            }
        }
        return IDList;
    }
    while(IDList.size() != N){
        int retID = -1;
        int end = -1;
        std::vector<octree::dataPtr> priorityID;

        if(pathMode == octreeModeBiased){
             retID = getClosestPathBiased(currentPos,baseLocation,targetLocation,findHorizon*envRad,end,true,0.5,priorityID);
        }
        else if(pathMode == sideToSide){
            int biasPointNum = 10;
            std::vector<Eigen::Vector3f> biasPoints(biasPointNum);
            for(int i = 0; i < biasPoints.size(); i++){
                Eigen::Vector3f thisOne(0,-1,0.5 - (float)i/ biasPoints.size());
                //Eigen::Vector3f thisOne(0,0,0);
                biasPoints[i] = thisOne;
            }
             retID =getClosestPathBiasedWithGlobal(currentPos,baseLocation,targetLocation,biasPoints,findHorizon*envRad,end,false,0, 1.0,priorityID);
        }
        else if(pathMode == octreeMode){
            retID = getClosestPathFast(currentPos,baseLocation,targetLocation,findHorizon*envRad,end,true,priorityID);
        }
        else if(pathMode == inOrder){
             static int currentPath  = 0;
             if( isComplete(currentPath)) currentPath++;
             if(currentPath == pathCount) currentPath = -1;
             retID  = currentPath;
        }
        //pick the best one that is not already in the list;
        int priorityIndex = 0;
        for(priorityIndex = 0; priorityIndex < priorityID.size() ; priorityIndex++){
            bool duplicate = false;
            for(int j = 0; j < IDList.size(); j++){
                if(priorityID[priorityIndex].data == IDList[j].first){
                    duplicate = true;
                    break;
                }
            }
            if(duplicate == false) break;
        }
        retID = priorityID[priorityIndex].data;
        if(end ==1) end = 2;
        else if(end ==2) end = 1;
        currentPos = targetLocation * Eigen::Translation3f(getStartEndPos(retID,end));
       baseLocation = baseToCentre.inverse() * currentPos;
       std::pair<int,int> thisPair;
       thisPair.first = retID;
       thisPair.second = end;
       IDList.push_back(thisPair);

    }
    return IDList;
}
