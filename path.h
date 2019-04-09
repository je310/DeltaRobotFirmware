#ifndef PATH_H
#define PATH_H

#define PATH_BUFFER_SIZE 512

#define HINT_NUMBER 8
#include <octree.h>
#include <stdint.h>
#include <Core.h>
#include <Geometry.h>
#include <math.h>
#include <limits>
#include <vector>

#ifdef DESKTOP
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#endif

enum shapes{
    invalid = 0,
    line = 1,
    circle = 2,
    bezier = 3
};

enum pathFindingMode{
    octreeModeBiased,
    octreeMode,
    inOrder,
    closestSlow,
    sideToSide
};

struct segment{
    int32_t ID;             //unique identifier for the segment
    int32_t complete;       //1 is complete.
    int32_t type;           //Type takes the shape enum, lines and circles usually.
    Eigen::Vector3f value1; // Values mean different things for different shapes.
    Eigen::Vector3f value2; // line: value1 = start point , value 2 = end point, value 3 = normal (not always needed).value 4 = not used
    Eigen::Vector3f value3; // Circle: value 1 = centre point, value 2 = vector to start, value3 = vector to end, value 4 = normal
    Eigen::Vector3f value4; // normal for the circle
    Eigen::Vector3f value5; //extra for normal interposation, especially for 3d bezier and other curved paths.
    Eigen::Vector3f value6;
    uint16_t startHints[HINT_NUMBER];
    uint16_t endHints[HINT_NUMBER];
    uint32_t endHintToEnd;
    uint32_t startHintToEnd;
    float fromStartT;   // Current progress from the start. 
    float fromEndT;     // Current progress from the end. 
    float speed;        // Velocity in m/s that the shape should be drawn with. 
};                      // all values are given in target space. (which can be set to global by setting target to 000, 0001)


// path manager holds the buffer for the current path and provides methods for tracking progress and choosing the next path to execute. 
class PathManager{
public:
    PathManager(octree * oct_);
    int addNewPath(segment aPath);                                                              // Adds a new segment to the buffer
    int getClosestPath(Eigen::Affine3f currentPos,Eigen::Affine3f baseLocation,Eigen::Affine3f targetLocation, float envelopeRadius, int &end); // Gets closest start/end point from the current position,
    int getClosestPathBiased(Eigen::Affine3f currentPos, Eigen::Affine3f baseLocation, Eigen::Affine3f targetLocation, float envelopeRadius, int &end, bool incBounds, float biasGain, std::vector<octree::dataPtr> &list);
    int getClosestPathFast(Eigen::Affine3f currentPos, Eigen::Affine3f baseLocation, Eigen::Affine3f targetLocation, float envelopeRadius, int &end, bool incBound,std::vector<octree::dataPtr> &list);
    int getClosestPathBiasedWithGlobal(Eigen::Affine3f currentPos,Eigen::Affine3f baseLocation,Eigen::Affine3f targetLocation,std::vector<Eigen::Vector3f> globalPoints, float envelopeRadius, int &end, bool incBounds,float biasGain, float GlobalGainTota,std::vector<octree::dataPtr> &list);
    std::vector<std::pair<int,int> > makeSeriesPrediction(Eigen::Affine3f currentPos,Eigen::Affine3f baseLocation, Eigen::Affine3f targetLocation,float findHorizon,float envRad, int N);
    // that is inside the envelope, returns one outside if not possible.
    int getSpareSpace();                                                                        // returns the amount of buffer space left. 
    int isComplete();                                                                           // checks all the complete flags in the buffer. returns 1 if all complete.
    int isComplete(int ID);
    int setComplete(int ID);                                                                    // Sets the complete flag.
    int setNotComplete();
    int markInvalid(int ID);                                                                            // marks all complete segments as invalid, alowing them to be overridden.
    int stepTime(int ID,float deltaT, int end);
    int stepTime(segment &seg, float deltaT, int end);
    int reachable(int ID , Eigen::Affine3f currentPos,Eigen::Affine3f targetPos, Eigen::Affine3f basePos, float envelopeRadius, Eigen::Affine3f &pos, int end);
    int getReachability(Eigen::Affine3f currentBasePos, Eigen::Affine3f currentHeadPos, Eigen::Affine3f targetPos, Eigen::Affine3f &posOut,  float &distance, float evelope);
    int getReachabilityFast(Eigen::Affine3f currentBasePos, Eigen::Affine3f currentHeadPos, Eigen::Affine3f targetPos,  float &distance, float evelope, float minMaxAngle);
    Eigen::Vector3f getPos(int ID, int end,Eigen::Affine3f targetLocation);
    int setupTravel(Eigen::Affine3f  start, Eigen::Vector3f end, Eigen::Affine3f targetLocation, Eigen::Vector3f normal, float speed, int toID, int toEnd);
    int isClose(int ID, int end, Eigen::Affine3f currentPos, Eigen::Affine3f targetLocation, float margin);
    Eigen::Vector3f getNormal(int ID);
    Eigen::Vector3f getEndPos(int ID);
    Eigen::Vector3f getStartPos(int ID);
    int pathType(int ID);
    void constrainHead(Eigen::Affine3f &currentPos, Eigen::Affine3f basePos, float envelope);
    int getBestHint(int ID, int &end);
    void setBit(uint32_t &val, int bit, int to);
    int  getBit(uint32_t val, int bit);
    std::vector<segment> returnVector();
    #ifdef DESKTOP
    cv::Mat  getHeatMap(float rad, Eigen::Vector3f centre, Eigen::Vector3f upDir , Eigen::Vector3f sideDir);
#endif
    float getLengthToComplete(float rad, Eigen::Vector3f position);
    float getLengthToCompleteFast(float rad, Eigen::Vector3f position);
    float pathInCircle(segment seg,float  rad, Eigen::Vector3f position);
    bool rebuildReset();

//private:
    octree* oct;
     uint32_t hash(uint32_t x);
    int hashLookup(int ID);                                                                     // Finds the array index for a given ID (usefull for large buffers when order is not
    segment hashLookupSegment(int ID);                                                                                      // guarenteed due to drawing segments out of order)
    int hashAllocate(int ID);                                                                   // gets the array id of the invalid segment where the ID will be stored.
    int reachable(segment seg , Eigen::Affine3f currentPos, Eigen::Affine3f targetPos, Eigen::Affine3f basePos, float envelopeRadius, Eigen::Affine3f &pos, int &end);
    void getEulerYPREigen(Eigen::Matrix3f mat, float& yaw, float& pitch, float& roll);
    Eigen::Vector3f getTPos(segment seg, float tIn);
    Eigen::Vector3f getEndPos(segment seg);
    Eigen::Vector3f getStartPos(segment seg);
    Eigen::Vector3f getStartEndPos(segment seg,int end);

    Eigen::Vector3f getStartEndPos(int ID,int end);
    float getT(segment seg, int end);
    Eigen::Vector3f quatraticBezier(Eigen::Vector3f start, Eigen::Vector3f CP, Eigen::Vector3f end, float t);
    Eigen::Vector3f cubicBezier(Eigen::Vector3f start, Eigen::Vector3f CP1,Eigen::Vector3f CP2, Eigen::Vector3f end, float t);
    Eigen::Vector3f cubicBezier(segment seg, float t);
    Eigen::Vector3f cubicBezierDerivitive(Eigen::Vector3f start, Eigen::Vector3f CP1,Eigen::Vector3f CP2, Eigen::Vector3f end, float t);
    float lookupCubicBezierT(segment seg, Eigen::Vector3f newPosition, int end, int divs);
    segment array[PATH_BUFFER_SIZE];                                                            // The array where the data is stored.  
    segment travelSeg;
    int bezierDivs;
    int errorCounter;
    int pathCount;
    int pathMode;
    int  finishedOnEnd(int ID);

    void addHintsClosest();

    Eigen::Affine3f GunMarkerToBaseCentre;
    Eigen::Matrix4f GunMarkerToBaseCentreM;
    Eigen::Affine3f GunMarkerToBaseCentreInv;
    Eigen::Matrix4f GunMarkerToBaseCentreInvM;
    Eigen::Affine3f HeadCentreToPitch;
    Eigen::Matrix4f HeadCentreToPitchM;
    Eigen::Affine3f HeadCentreToPitchInv;
    Eigen::Matrix4f HeadCentreToPitchInvM;
    Eigen::Affine3f PitchToYaw;
    Eigen::Matrix4f PitchToYawM;
    Eigen::Affine3f PitchToYawInv;
    Eigen::Matrix4f PitchToYawInvM;
    Eigen::Affine3f imuToOrigin;
    Eigen::Matrix4f imuToOriginM;
    Eigen::Affine3f imuToOriginInv;
    Eigen::Matrix4f imuToOriginInvM;
    Eigen::Affine3f boardMarkerToCentre;
    Eigen::Matrix4f boardMarkerToCentreM;
    Eigen::Affine3f boardMarkerToCentreInv;
    Eigen::Matrix4f boardMarkerToCentreInvM;
    Eigen::Affine3f yawToTip;
    Eigen::Matrix4f yawToTipM;
    Eigen::Affine3f yawToTipInv;
    Eigen::Matrix4f yawToTipInvM;

    Eigen::Affine3f originToCentre;
    Eigen::Matrix4f originToCentreM;

    Eigen::Affine3f DoriginToCentre;
    Eigen::Matrix4f DoriginToCentreM;

    Eigen::Affine3f baseToCentre;

};










#endif
