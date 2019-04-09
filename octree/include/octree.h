#ifndef OCTREE_H
#define OCTREE_H

#include <Geometry.h>
#include <limits>
#include <queue>
#include <iostream>

class octree{
#define ISLEAF (1<<0)
#define CHECKA (1<<1)
#define CHECKB (1<<2)


public: class node;
public: struct dataPtr{
        Eigen::Vector3f point;
        int data;
    };

    struct dataPtrs{
        dataPtr data[2];
    };
public: union pBatch {
        node* nodes[8];
        dataPtrs data;
        pBatch(){new(&data) dataPtrs();}
    };
public: class node{

    public:
        int nodeInfo;  // this will define some properties of the octree node. for instance we can change whether this is considered a leaf node.
        // If it is a leaf node then the pointers act as pointers to data.
        int dataBelow;
        node* parent;
        pBatch pointers;

    };

    class bounds{
    public:
        Eigen::Vector3f min;
        Eigen::Vector3f max;
        void downScale(int direction);
        int whichDir(Eigen::Vector3f pointIn);
        void upScale(node* aNode);
        void upScale(int dir);
        float half(int dir);
        float extendBack(int dir);
        float extendForward(int dir);
    };

    class distAndPointer{
    public:
        float dist;
        node* aNode;
        bounds bound;



    };

    int levels;
    int batch;
    bounds topLevelBound;


    Eigen::Vector3f centre;
    float range;
    dataPtr nullDataPtrdata;

    int freeNodeStackPtr;
    int freeNodeCount;
    int totalNodes;
    node* nodeMem;
    node* head;
    node* scrapHead;
    int currentDepth;
    //    typedef nodeBatch node*[8];







    octree(int maxLevels, int batching, Eigen::Vector3f centrePoint, float edgeLength, node* nodeBuffer, int nodes);
    int insert(node* aNode, dataPtr data, bounds bound, int depth);
    int insert(octree::dataPtr data );
    node* allocateNode(node *parentIn);
    void deallocateNode(node* aNode);
    void initLeaf(node* aNode);
    bool remove(dataPtr data);
    void iterateDecCount(node* aNode, int countDec);
    bool isLeaf(node* aNode){
        return (aNode->nodeInfo & ISLEAF)!=0;
    }
    bool hasCheckA(node* aNode){
        return (aNode->nodeInfo & CHECKA)!=0;
    }
    bool hasCheckB(node* aNode){
        return (aNode->nodeInfo & CHECKB)!=0;
    }
    bool setCheckA(node* aNode){
        aNode->nodeInfo = aNode->nodeInfo | CHECKA;
    }
    bool setCheckB(node* aNode){
        aNode->nodeInfo =  aNode->nodeInfo | CHECKB;
    }
    bool clearCheckA(node* aNode){
        aNode->nodeInfo = aNode->nodeInfo & (~ CHECKA);
    }
    bool clearCheckB(node* aNode){
        aNode->nodeInfo = aNode->nodeInfo & (~ CHECKB);
    }
    int redistribute(node* aNode);
    int compact(node* aNode);
    int cullNode(node* aNode);
    float spaceLeft() { return (float)freeNodeStackPtr/totalNodes;}
    octree::node* getNode(dataPtr data);
    octree::node* getNode(node* aNode, bounds &bound, dataPtr data);
    std::vector<octree::dataPtr> getNnearest(Eigen::Vector3f point, int N, float radius);
    std::vector<octree::dataPtr> getNnearestBiased(Eigen::Vector3f point, Eigen::Vector3f BiasPoint, int N, float radius, float biasGain);
    std::vector<octree::dataPtr> getNnearestBiasedVector(Eigen::Vector3f point,std::vector<Eigen::Vector3f> BiasPoints, int N, float radius, std::vector<float> biasGains);
    std::vector<octree::dataPtr> getInsideRadius(Eigen::Vector3f point, float radius);
    Eigen::Vector3f nearestPointOnCube(Eigen::Vector3f point, octree::bounds  bound);
    octree::node* getWouldBeNode(node *aNode, Eigen::Vector3f target, bounds &bound);
    octree::bounds defaultBounds();
    bool isInBound(octree::bounds bound, octree::dataPtr data);
    bool reset();
};

inline bool operator >(const octree::distAndPointer &lhs,const octree::distAndPointer &rhs)
{
    return lhs.dist > rhs.dist;
}

inline bool operator ==(const octree::dataPtr &lhs,const octree::dataPtr &rhs)
{
    return lhs.data == rhs.data && lhs.point == rhs.point;
}
























#endif
