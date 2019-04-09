# octree
This is a work in progress for an octree implementation that is friendly with embedded systems. Currrently it is best used with a 32bit system for memory packing reasons. It has been tested with 64 bit systems and it works, though the memory usage is less efficient. 

# Structure
There is a linked list storing a list of nodes that can be allocated to the octree structure, which is referenced by 'scrapHead'. Each node has:
- An info variable for tracking if it is a octant/leaf node, and is used for marking nodes for search algorithms etc. 
- An integer that keeps the count of total points below this node.
- A parent pointer.
- 8 pointers to child nodes OR 2 data items. The data items consist of a 32 bit int ( will be a pointer ) and an Eigen::Vector3f point. This duplicity of these members is created with a union. On a 32 bit system this would share this memory space properly. Future implementation would either remove this duplicity, or make it configurable with different vector precisions and data member quantities. For example a 64 bit system could have these memory spaces used as 8 64bit pointers, and 4 data members of single precision or 2 of double precision. 

# Building 

mkdir build 

cd build 

cmake .. 

make -j8

