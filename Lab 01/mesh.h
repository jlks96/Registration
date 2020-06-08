#pragma once

// maximum number of vertices and triangles
#define MAXV 1000000
#define MAXT 1000000

#include <map>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <bitset>
#include <vector>
#include <Eigen\Dense>
#include "GL\glut.h"
#include "KDTree.hpp"

// hash for pair
template <class T>
inline void hash_combine(std::size_t& seed, const T& v) {
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

namespace std {
    template<typename S, typename T>
    struct hash<pair<S, T>> {
        inline size_t operator()(const pair<S, T> v) const {
            size_t seed = 0;
            ::hash_combine(seed, v.first);
            ::hash_combine(seed, v.second);
            return seed;
        }
    };
}

typedef int OrTri;
typedef int tIdx;

inline OrTri makeOrTri(tIdx t, int version) { return (t << 3) | version; };
inline tIdx idx(OrTri ot) { return ot >> 3; };
inline int ver(OrTri ot) { return ot & 0b111; };
/*
inline OrTri enext(OrTri ot) {
    int v = ver(ot);  
    return makeOrTri(idx(ot), v < 3 ? (v + 1) % 3 : 3 + ((v - 1) % 3));
};
inline OrTri sym(OrTri ot) { int v = ver(ot); return v < 3 ? ot + 3 : ot - 3; };
*/

class myObjType {
    int vcount = 0;
    int tcount = 0;
    int ecount = 0;

    double vlist[MAXV][3];   // vertices list (1 based)
    int tlist[MAXT][3];      // triangle list (1 based)
    int fnlist[MAXT][6];     // fnext list (1 based)
    double nlist[MAXT][3];   // storing triangle normals (1 based)
    double vnlist[MAXV][3];     // storing vertex normals (1 based)
    std::unordered_map<std::pair<int, int>, int> edgemap; // stores map of edges to triangle (for computing fnext)
    std::unordered_map<std::string, int> vnamemap; // stores map of "vertex name" to vertex index (for stl reading)
    std::unordered_map<int, int> vmap; // stores map of vertex to one of the surronding triangle index (for smooth shading)

    double lmax[3];          // the maximum coordinates of x,y,z
    double lmin[3];          // the minimum coordinates of x,y,z

    int statMinAngle[18]; // each bucket is degrees has a 10 degree range from 0 to 180 degree
    int statMaxAngle[18];

    //-----DS for Mesh Simplification-----//
    std::unordered_map<std::pair<int, int>, long> edgeDistMap; // a hash map mapping edge (pair of vertices) to its actual distance * 10000
    std::priority_queue<std::pair<double, std::pair<int, int>>> edgeDistancePQ = std::priority_queue<std::pair<double, std::pair<int, int>>>(); // PQ for storing edge distances
    std::bitset<MAXV> vlistBitset; // bitset for vlist, bit value = 1 if deleted
    std::bitset<MAXT> tlistBitset; // bitset for tlist, bit value = 1 if deleted

    // duplicate DS for undo function
    double vlist2[MAXV][3];
    int tlist2[MAXT][3];
    int fnlist2[MAXT][6];
    double nlist2[MAXT][3];
    double vnlist2[MAXV][3];
    std::unordered_map<std::pair<int, int>, int> edgemap2;
    std::unordered_map<std::string, int> vnamemap2;
    std::unordered_map<int, int> vmap2;

    //-----DS for ICP-----//
    int vcount3 = 0;
    int tcount3 = 0;
    double vlist3[MAXV][3];
    int tlist3[MAXT][3];
    int fnlist3[MAXT][6];
    double nlist3[MAXT][3];
    double vnlist3[MAXV][3];
    std::unordered_map<std::pair<int, int>, int> edgemap3;
    std::unordered_map<std::string, int> vnamemap3;
    std::unordered_map<int, int> vmap3;
    double lmax3[3];
    double lmin3[3];
    int statMinAngle3[18];
    int statMaxAngle3[18];
    std::vector<int> finalNeighbourIndices; // stores the final neighbour indices of the first set
    bool hasDrawnErrorLines = false;
    std::vector<double> finalNeighbourErrors; // stores all the errors of all pairs of neighbours
    double maxError; // stores the maximum error for all pairs of neighbours

private:
    double distance(double* v0, double* v1); // calculates the distance between v0 and v1 (3D vectors)
    double angle(double l0, double l1, double l2); // calculates the angle a2 of triangle with lengths l0, l1, l2
    int computeVersionFromVertices(int tIdx, int v0, int v1); // calculates the version number of triangle from order of 2 vertices
    bool isSameOrientation(int tIdx, int v0, int v1); // determines if tIdx has same orientation as the edge v0-v1
    void orientTriangle(int tIdx); // orientates the triangle by swapping the first 2 vertices
    void updateVmap(int vIdx, int tIdx); // updates vmap

    //-----private methods for Mesh Simplification-----//
    void computeEdges(); // compute the initial edgeDistMap, vertexToEdgesMap, edgeDistancePQ
    void updateFnlist(OrTri t0, OrTri t1); // update the fnext between t0 and t1
    void createDuplicate(); // make duplicates of all the DS

    //-----private methods for ICP -----//
    void updateVmap3(int vIdx, int tIdx); // updates vmap3 for second set of points
 
public:
    myObjType() { vcount = 0; tcount = 0; };
    void draw(bool smooth);
    void readFile(char* filename); // assumming file contains a manifold
    void readStlFile(char* filename); // reads in binary STL file (Lab 1)
    void writeFile(char* filename); // outputs OBJ file (Lab 1)
    void computeStat(); // computes statistics of file (Lab 1)
    void computeNormals(); // computes normals of triangles (Lab 1)
    void computeFnlist(); // computes fnlist (Lab 2)
    void computeComponents(); // computes number of components (Lab 2)
    bool orientTriangles(); // orients all triangles in the same facing direction, outputs false if not possible (Lab 2)
    void computeVertexNormals(); // computes normals of vertices (Lab 2)
    void colorBoundaryEdge(); // colors boundary edges red (Lab 2)
    OrTri enext(OrTri orTri); // returns OrTri of enext (Lab 2)
    OrTri sym(OrTri orTri); // returns OrTri of sym (Lab 2)
    int org(OrTri orTri); // returns vertex index of org (Lab 2)
    int dest(OrTri orTri); // returns vertex index of dest (Lab 2)
    OrTri fnext(OrTri orTri); // returns OrTri of fnext (Lab 2)

    //-----public methods for Mesh Simplification-----//
    void edgeContraction(); // carry out one edge contraction
    void restoreInitial(); // restore of all the DS to initial state

    //-----public methods for ICP-----//
    void readFile3(char* filename); // assumming file contains a manifold
    std::vector<int> getNearestDistanceIndices(); // returns the nearest distance indices for each of the vertices in first set (smaller)
    std::vector<int> getNearestDistanceIndicesFast(); // uses own kdtree
    std::vector<int> getNearestDistanceIndicesFastSecondVersion(); // uses good kdtree
    void ICP(); // do ICP on the two vertices set (first set must have fewer points)
    void ICPSlow(); // do slow ICP on the two vertices set (first set must have fewer points)
    void draw3(bool smooth); // draw the second set of points
    void computeNormals3(); // computes normals for second set of points
    void computeFnlist3(); // computes fnlist for second set of points
    void computeVertexNormals3(); // computes normals of vertices for second set of points
    void drawFinalErrorLines(std::vector<int> neighboursIndices); // draw all the error lines linking the 2 sets
    void transformVertices(); // for generation of data
    void drawInitialErrorLines(); // draw all initial error lines linking the 2 sets
    void toggleHasDrawnErrorLines(); // toggle flag indicating has drawn error lines or not
    void drawErrorMap(); // draw the error map on the first object to visualise error
    void setErrorColor(int vIdx); // set the material color to reflect the error of the vertex
    void computeErrors(); // compute the maximum error for the current ICP
};
