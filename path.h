#ifndef PATH_H
#define PATH_H

#define PATH_BUFFER_SIZE 512

#include <stdint.h>
#include <Core.h>
#include <Geometry.h>
#include <math.h>
#include <limits>

enum shapes{
	invalid = 0,
	line = 1,
    circle = 2,
    bezier = 3
};

struct segment{
    int32_t ID;				//unique identifier for the segment
    int32_t complete;		//1 is complete.
    int32_t type;			//Type takes the shape enum, lines and circles usually.
    Eigen::Vector3f value1;	// Values mean different things for different shapes.
    Eigen::Vector3f value2;	// line: value1 = start point , value 2 = end point, value 3 = normal (not always needed).value 4 = not used
    Eigen::Vector3f value3;	// Circle: value 1 = centre point, value 2 = vector to start, value3 = vector to end, value 4 = normal
    Eigen::Vector3f value4; // normal for the circle
    Eigen::Vector3f value5; //extra for normal interposation, especially for 3d bezier and other curved paths.
    Eigen::Vector3f value6;
	float fromStartT;	// Current progress from the start. 
	float fromEndT;		// Current progress from the end. 
	float speed;		// Velocity in m/s that the shape should be drawn with. 
};						// all values are given in target space. (which can be set to global by setting target to 000, 0001)


// path manager holds the buffer for the current path and provides methods for tracking progress and choosing the next path to execute. 
class PathManager{
public:
    PathManager();
	int addNewPath(segment aPath);																// Adds a new segment to the buffer
    int getClosestPath(Eigen::Vector3f currentPos, Eigen::Vector3f centreEnvelope, float envelopeRadius, int &end);	// Gets closest start/end point from the current position,
																								// that is inside the envelope, returns one outside if not possible.
	int getSpareSpace();																		// returns the amount of buffer space left. 
	int isComplete();																			// checks all the complete flags in the buffer. returns 1 if all complete.
    int isComplete(int ID);
	int setComplete(int ID);																	// Sets the complete flag.
    int markInvalid(int ID);																			// marks all complete segments as invalid, alowing them to be overridden.
    int stepTime(int ID,float deltaT, int end);
    int stepTime(segment &seg, float deltaT, int end);
    int reachable(int ID , Eigen::Vector3f currentPos, Eigen::Vector3f centreEnvelope, float envelopeRadius, Eigen::Vector3f &pos, int end);
    Eigen::Vector3f getPos(int ID, int end);
    int setupTravel(Eigen::Vector3f start,Eigen::Vector3f end,Eigen::Vector3f normal, float speed);
    int isClose(int ID, int end, Eigen::Vector3f currentPos, float margin);
    Eigen::Vector3f getNormal(int ID);
    Eigen::Vector3f getEndPos(int ID);
    Eigen::Vector3f getStartPos(int ID);
    int pathType(int ID);

//private:
     uint32_t hash(uint32_t x);
	int hashLookup(int ID);																		// Finds the array index for a given ID (usefull for large buffers when order is not
    segment hashLookupSegment(int ID);																						// guarenteed due to drawing segments out of order)
	int hashAllocate(int ID);																	// gets the array id of the invalid segment where the ID will be stored.
    int reachable(segment seg , Eigen::Vector3f currentPos, Eigen::Vector3f centreEnvelope, float envelopeRadius, Eigen::Vector3f &pos, int &end);
    Eigen::Vector3f getEndPos(segment seg);
    Eigen::Vector3f getStartPos(segment seg);
    Eigen::Vector3f getStartEndPos(segment seg,int end);
    float getT(segment seg, int end);
    Eigen::Vector3f quatraticBezier(Eigen::Vector3f start, Eigen::Vector3f CP, Eigen::Vector3f end, float t);
    Eigen::Vector3f cubicBezier(Eigen::Vector3f start, Eigen::Vector3f CP1,Eigen::Vector3f CP2, Eigen::Vector3f end, float t);
    Eigen::Vector3f cubicBezier(segment seg, float t);
    Eigen::Vector3f cubicBezierDerivitive(Eigen::Vector3f start, Eigen::Vector3f CP1,Eigen::Vector3f CP2, Eigen::Vector3f end, float t);
    float lookupCubicBezierT(segment seg, Eigen::Vector3f newPosition, int end);
	segment array[PATH_BUFFER_SIZE]; 															// The array where the data is stored.  
    segment travelSeg;
    int bezierDivs;
    int errorCounter;

};










#endif
