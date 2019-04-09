#include<eigen3/Eigen/Geometry>

#include <iostream>
#include <chrono>
#include <ctime>
#include <math.h>
#include "octree.h"


using namespace Eigen;
using namespace std;

typedef octree::node nodeType;


union Float_t
{
    Float_t(float num = 0.0f) : f(num) {}
    // Portable extraction of components.
    bool Negative() const { return (i >> 31) != 0; }
    int32_t RawMantissa() const { return i & ((1 << 23) - 1); }
    int32_t RawExponent() const { return (i >> 23) & 0xFF; }

    int32_t i;
    float f;
#ifdef _DEBUG
    struct
    {   // Bitfields for exploration. Do not use in production code.
        uint32_t mantissa : 23;
        uint32_t exponent : 8;
        uint32_t sign : 1;
    } parts;
#endif
};

int main(int argc, char** argv) {
    int nodeNumber = 3000;
    float dataOverhead = 2;
    int bufferNum = (int)(nodeNumber*dataOverhead);
    //int nodeBuffer2[bufferNum];
    Eigen::Vector3f centre(0,0,0);
    float range = 1.0;

    cout << "size of int" << sizeof(octree::node)<< endl;
    cout << "size of int" << sizeof(octree::node::dataBelow)<< endl;
    cout << "size of int" << sizeof(octree::node::nodeInfo)<< endl;
    cout << "size of int" << sizeof(octree::node::parent)<< endl;
    cout << "size of int" << sizeof(octree::node::pointers)<< endl;
    cout << (int)(nodeNumber*dataOverhead) * sizeof(nodeType) << endl;
    nodeType* nodeBuffer = new nodeType[(int)(nodeNumber*dataOverhead)];
    int dataBuffer[nodeNumber];

    octree myOct(32,2,centre,range,nodeBuffer,(int)(nodeNumber*dataOverhead));

    //tests
    octree::bounds bound;
    bound.max  = Vector3f(1,1,1);
    bound.min = Vector3f(0,0,0);

    //test octant division.
    if(bound.whichDir(Vector3f(0.75,0.75,0.75)) != 0) cout << "error0!" << endl;
    if(bound.whichDir(Vector3f(0.75,0.25,0.75)) != 1) cout << "error1!" << endl;
    if(bound.whichDir(Vector3f(0.25,0.25,0.75)) != 2) cout << "error2!" << endl;
    if(bound.whichDir(Vector3f(0.25,0.75,0.75)) != 3) cout << "error3!" << endl;

    if(bound.whichDir(Vector3f(0.75,0.75,0.25)) != 4) cout << "error4!" << endl;
    if(bound.whichDir(Vector3f(0.75,0.25,0.25)) != 5) cout << "error5!" << endl;
    if(bound.whichDir(Vector3f(0.25,0.25,0.25)) != 6) cout << "error6!" << endl;
    if(bound.whichDir(Vector3f(0.25,0.75,0.25)) != 7) cout << "error7!" << endl;

    // test bounding cube scaling (simple)
    bound.downScale(1);
    bound.upScale(1);
    if(bound.max !=Vector3f(1,1,1) && bound.min != Vector3f(0,0,0)) cout << "error upscale and downscale" << endl;

    //test insertion and N-closest, all insertions are given data that is incrementing, and a point that is increasingly far (at random direction) from the test point
    // therefore we know that we must get the elements back in order.
    clock_t begin = clock();
    float rad = 1.0/nodeNumber;
    float radInc = rad*0.7;
    for(int i = 0; i < nodeNumber; i++){
        //Eigen::Vector3f randVec(1.0*(float)rand()/RAND_MAX,1.0*(float)rand()/RAND_MAX,1.0*(float)rand()/RAND_MAX);
        Eigen::Vector3f randVec(2.0*(float)rand()/RAND_MAX,2.0*(float)rand()/RAND_MAX,2.0*(float)rand()/RAND_MAX);
        randVec -= Eigen::Vector3f(1,1,1);
        randVec.normalize();
        dataBuffer[i] = i+1;
        octree::dataPtr data;
        data.data = dataBuffer[i];
        data.point = rad*randVec + centre;
        int depth = myOct.insert(data);
        rad += radInc;

    }

    int toFind = nodeNumber / 2;
    std::vector<octree::dataPtr> list =  myOct.getNnearest(centre,toFind,3.0*range);
    list =  myOct.getNnearest(centre,toFind,3.0*range);
    for(int i = 0; i < toFind; i++){
        if(list[i].data != i+1) cout << "wrong order!" << endl;
    }


    //test removals
    nodeType* nodeBuffer3 = new nodeType[(int)(nodeNumber*dataOverhead)];
    octree myOct2(32,2,centre,range,&nodeBuffer3[0],(int)(nodeNumber*dataOverhead));
    Eigen::Vector3f randVec(2.0*(float)rand()/RAND_MAX,2.0*(float)rand()/RAND_MAX,2.0*(float)rand()/RAND_MAX);
    randVec -= Eigen::Vector3f(1,1,1);
    randVec.normalize();
    Eigen::Vector3f curPos = -0.9*randVec;
    float posInc = 0.9*2.0/nodeNumber;
    for(int i = 0; i < nodeNumber; i++){
        curPos += posInc * randVec;
        octree::dataPtr data;
        data.data = i+1;
        data.point = curPos;
        myOct2.insert(data);
    }
    curPos = -0.9*randVec;
    for(int i = 0; i < nodeNumber; i++){
        curPos += posInc * randVec;
        octree::dataPtr data;
        data.data = i+1;
        data.point = curPos;
        myOct2.remove(data);
    }
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

    // test strange cases
    // insterting outside the bounds
    // returning depth of insert == 0 means that the point was not inserted.
    octree::dataPtr data;
    data.point = centre + Eigen::Vector3f(1.1*range,1.1*range,1.1*range);
    data.data = 1;
    int depthInserted = myOct2.insert(data);
    if(depthInserted != 0) cout << "there is a problem in correctly managing out of bounds points" << endl;

    //multiple inserts
    data.point = centre;
    depthInserted = myOct2.insert(data);
    depthInserted = myOct2.insert(data);
    depthInserted = myOct2.insert(data);

    depthInserted = myOct2.remove(data);
    depthInserted = myOct2.remove(data);
    depthInserted = myOct2.remove(data);

    //inserting very close numbers
    Float_t x(0.5);
    for(int i = 0; i < nodeNumber; i++){
        data.point = centre + Eigen::Vector3f(x.f,0.5*range,0.5*range);
        data.data = i+1;
        depthInserted = myOct2.insert(data);
        x.i ++;
    }
    x = Float_t(0.5);
    for(int i = 0; i < nodeNumber; i++){
        data.point = centre + Eigen::Vector3f(x.f,0.5*range,0.5*range);
        data.data = i+1;
        depthInserted = myOct2.remove(data);
        x.i ++;
    }

    // test the find within constrained range
    rad = 1.0/nodeNumber;
    radInc = rad*0.7;
    for(int i = 0; i < nodeNumber; i++){

        //Eigen::Vector3f randVec(1.0*(float)rand()/RAND_MAX,1.0*(float)rand()/RAND_MAX,1.0*(float)rand()/RAND_MAX);
        Eigen::Vector3f randVec(2.0*(float)rand()/RAND_MAX,2.0*(float)rand()/RAND_MAX,2.0*(float)rand()/RAND_MAX);
        randVec -= Eigen::Vector3f(1,1,1);
        randVec.normalize();
        dataBuffer[i] = i+1;
        octree::dataPtr data;
        data.data = dataBuffer[i];
        data.point = rad*randVec + centre;
        int depth = myOct2.insert(data);
        rad += radInc;

    }

    toFind = nodeNumber;
    list =  myOct.getNnearest(centre,toFind,range/4);
    int errorCount = 0;
    for(int i = 0; i < list.size(); i++){
        float dist = (list[i].point - centre).norm();
        if(dist > range/4){
            errorCount ++;
            cout << "outside range!" << endl;
        }
    }


    cout << "error count" << errorCount <<endl;
    cout << "list times 2 : " << elapsed_secs << endl;
    cout << "finished";
    cout << "finished";


}

