#include "path.h"

using namespace Eigen;

PathManager::PathManager(){
    for(int i = 0; i < PATH_BUFFER_SIZE; i++){
        segment newSeg;
        newSeg.type = invalid;
        newSeg.ID = -1;
        newSeg.complete = 1;
        array[i] = newSeg;
    }
    errorCounter = 0;
    bezierDivs = 256;
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
float PathManager::lookupCubicBezierT(segment seg,Eigen::Vector3f newPosition, int end){


    int bestSegment = -1;
    float distance = std::numeric_limits<float>::max();
    int runnerUpSegment = -1;
    float runnerUpDistance = std::numeric_limits<float>::max();
    float smallDot = std::numeric_limits<float>::max();
    //we segment the bezier into line segments, and find the two end points that are both closest and roughly oposites ends of the point.
    if(end ==1){
        int startT = seg.fromStartT*(bezierDivs);
        int wasNeg = 0;
        for(int i = startT; i < bezierDivs; i++){
            Vector3f pointA = cubicBezier(seg,i* 1.0/bezierDivs);
            Vector3f pointB = cubicBezier(seg,(i+1)* 1.0/bezierDivs);
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
            if(dot > 0 && wasNeg) break;
        }
    }
    else{
        int startT = (1.0-seg.fromEndT)*(bezierDivs) + 1;
        int wasNeg = 0;
        for(int i =startT-1; i >=  -1 ; i--){
            Vector3f pointA = cubicBezier(seg,i* 1.0/bezierDivs);
            Vector3f pointB = cubicBezier(seg,(i+1)* 1.0/bezierDivs);
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
            if(dot > 0 && wasNeg) break;
        }
    }

    // we assume that the point is very close to the line, therfore the ratio of distances from A and B is the fraction traveled allong the line.
    if(bestSegment >= 0){
    Vector3f pointA = cubicBezier(seg,bestSegment* 1.0/bezierDivs);
    Vector3f pointB = cubicBezier(seg,(bestSegment+1)* 1.0/bezierDivs);
    Vector3f toA = pointA - newPosition;
    Vector3f toB = pointB - newPosition;
    Vector3f AtoB = pointB - pointA;
    Vector3f AtoPoint = newPosition -pointA;
    float dot = AtoB.dot(AtoPoint);
    float lineT =dot/pow(AtoB.norm(),2);
    //lineT = toA.norm()/(AtoB.norm());
    return ((float)bestSegment + lineT)*(1.0/bezierDivs);
    }
//    if(runnerUpSegment >=0){
//        Vector3f pointA = cubicBezier(seg,runnerUpSegment* 1.0/bezierDivs);
//        Vector3f pointB = cubicBezier(seg,(runnerUpSegment+1)* 1.0/bezierDivs);
//        Vector3f toA = pointA - newPosition;
//        Vector3f toB = pointB - newPosition;
//        float lineT = toA.norm()/(distance);
//        return ((float)runnerUpSegment + lineT)*(1.0/bezierDivs);
//    }
    if(end == 1)
        return 1;  // something went wrong, say that we have finished the line !
    if(end ==2)
        return 0;
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

int PathManager::isClose(int ID, int end, Eigen::Vector3f currentPos, float margin){
    segment seg = hashLookupSegment(ID);

    Vector3f location;
    if(end ==1) location = getStartPos(seg);
    if(end ==2) location = getEndPos(seg);
    float dist = (currentPos - location).norm();
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

float PathManager::getT(segment seg, int end){
    if(end == 1) return seg.fromStartT;
    if(end == 2) return seg.fromEndT;
}

int PathManager::reachable(int ID ,Vector3f currentPos, Vector3f centreEnvelope, float envelopeRadius,Vector3f &pos, int end){
    segment seg = hashLookupSegment(ID);
    return reachable(seg ,currentPos, centreEnvelope, envelopeRadius,pos, end);
}

//returns whether there is a reachable part of the segment (from either end)
int PathManager::reachable(segment seg ,Vector3f currentPos, Vector3f centreEnvelope, float envelopeRadius,Vector3f &pos, int &end){
    Vector3f startPos, endPos; // start and end after adding any existing progress.
    int ret = 0;
    int startReachable = 0, endReachable = 0;
    int error = 0;

    switch (seg.type) {
    case line:{
        Vector3f vec = seg.value2 - seg.value1; // vec from start to end of line.
        startPos = seg.value1 + vec * seg.fromStartT;
        endPos = seg.value2 - vec * seg.fromEndT;



        break;
    }

    case circle:{
        endPos = getEndPos(seg);
        startPos = getStartPos(seg);

        break;
    }

    case bezier:{
        endPos = getEndPos(seg);
        startPos = getStartPos(seg);
        break;
    }


    }
    float startDist = (startPos - currentPos).norm();
    float endDist = (endPos -  currentPos).norm();
    if((startPos - centreEnvelope).norm() < envelopeRadius) startReachable = 1;
    if((endPos - centreEnvelope).norm() < envelopeRadius) endReachable = 1;
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

        float newT = lookupCubicBezierT(seg,newPosition, end);
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

Eigen::Vector3f PathManager::getPos(int ID, int end){
    segment seg = hashLookupSegment(ID);
    if(end == 1){
        return getStartPos(seg);
    }
    if(end == 2){
        return getEndPos(seg);
    }
}

int PathManager::getClosestPath(Vector3f currentPos, Vector3f centreEnvelope, float envelopeRadius, int &end){
    int closestIn = -1;
    int closestOut = -1;
    int closestInEnd = -1;
    int closestOutEnd = -1;
    float distanceIn = std::numeric_limits<float>::max();

    float distanceOut = std::numeric_limits<float>::max();
    for(int i = 0 ; i < PATH_BUFFER_SIZE; i++){
        Vector3f pos;
        int end = -1;
        if(array[i].complete == 0 && array[i].type != invalid){
            int isIn = reachable(array[i],currentPos, centreEnvelope,envelopeRadius,pos, end);
            switch(isIn){
            case 1:
                if( (currentPos-pos).norm() < distanceIn){
                    distanceIn = (currentPos-pos).norm();
                    closestIn = i;
                    closestInEnd = end;
                }
                break;

            case 0:
                if( (currentPos-pos).norm() < distanceOut){
                    distanceOut = (currentPos-pos).norm();
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

int PathManager::pathType(int ID){
    segment seg = hashLookupSegment(ID);
    return seg.type;
}

int PathManager::setupTravel(Eigen::Vector3f start,Eigen::Vector3f end,Eigen::Vector3f normal, float speed){
    travelSeg.ID = -2;
    travelSeg.complete = 0;
    travelSeg.fromEndT = 0;
    travelSeg.fromStartT = 0;
    travelSeg.speed = speed;
    travelSeg.type = line;
    travelSeg.value1 = start;
    travelSeg.value2 = end;
    travelSeg.value3 = normal;
    return 1;
}

uint32_t PathManager::hash(uint32_t x) {
    return (117*x)%PATH_BUFFER_SIZE;
}

int PathManager::addNewPath(segment aPath){
    int h = hashAllocate(aPath.ID);
    if(h >= 0){
        array[h] = aPath;
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
        if(array[i].complete == 0) return 0;
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
