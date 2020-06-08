#include "mesh.h"

#ifdef _WIN32
#include <Windows.h>
#include "GL\glut.h"
#define M_PI 3.141592654
#elif __APPLE__
#include <OpenGL/gl.h>
#include <GLUT/GLUT.h>
#endif

#include "math.h"
#include <string>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <vector>
#include "mesh.h"
#include <queue>
#include <iomanip>
#include <unordered_set>
#include "kdtree.h"
using namespace std;
using namespace Eigen;

double myObjType::distance(double* v0, double* v1) {
    // Lab 1 Subtask 2: Compute Angle Statistics
    return sqrt(pow(v0[0] - v1[0], 2) + pow(v0[1] - v1[1], 2) + pow(v0[2] - v1[2], 2));
}

double myObjType::angle(double l0, double l1, double l2) {
    // Lab 1 Subtask 2: Compute Angle Statistics
    return acos((l0*l0 + l1 * l1 - l2 * l2) / (2 * l0 * l1)) * 180.0 / M_PI;
}

int myObjType::computeVersionFromVertices(int tIdx, int v0, int v1) {
    // Lab 2 Subtask 3: Implement fnext
    if (tlist[tIdx][0] == v0 && tlist[tIdx][1] == v1) {
        return 0;
    } else if (tlist[tIdx][1] == v0 && tlist[tIdx][2] == v1) {
        return 1;
    } else if (tlist[tIdx][2] == v0 && tlist[tIdx][0] == v1) {
        return 2;
    } else if (tlist[tIdx][1] == v0 && tlist[tIdx][0] == v1) {
        return 3;
    } else if (tlist[tIdx][2] == v0 && tlist[tIdx][1] == v1) {
        return 4;
    } else if (tlist[tIdx][0] == v0 && tlist[tIdx][2] == v1) {
        return 5;
    }
    return -1;
}

bool myObjType::isSameOrientation(int tIdx, int v0, int v1) {
    // Lab 2 Optional 2: Implement orientTriangles()
    if (tlist[tIdx][0] == v0 && tlist[tIdx][1] == v1 ||
        tlist[tIdx][1] == v0 && tlist[tIdx][2] == v1 ||
        tlist[tIdx][2] == v0 && tlist[tIdx][0] == v1) {
        return true;
    }
    return false;
}

void myObjType::orientTriangle(int tIdx) {
    // Lab 2 Optional 2: Implement orientTriangles();
    int fnextIdxMapping[] = { 3, 5, 4, 0, 2, 1 };
    int temp = tlist[tIdx][0];
    tlist[tIdx][0] = tlist[tIdx][1];
    tlist[tIdx][1] = temp;
    int fnextEntries[6];
    for (int i = 0; i < 6; i++) {
        fnextEntries[i] = fnlist[tIdx][i];
    }
    for (int i = 0; i < 6; i++) {
        fnlist[tIdx][i] = fnextEntries[fnextIdxMapping[i]];
    }
}

void myObjType::updateVmap(int vIdx, int tIdx) {
    // Lab 2 Optional 3: Smooth Shading
    if (vmap.find(vIdx) == vmap.end()) {
        vmap.insert({vIdx, tIdx});
    }
}

void myObjType::computeEdges() {
    for (int i = 1; i <= tcount; i++) {
        pair<int, int> first_edge = make_pair(min(tlist[i][0], tlist[i][1]), max(tlist[i][0], tlist[i][1]));
        pair<int, int> second_edge = make_pair(min(tlist[i][1], tlist[i][2]), max(tlist[i][1], tlist[i][2]));
        pair<int, int> third_edge = make_pair(min(tlist[i][2], tlist[i][0]), max(tlist[i][2], tlist[i][0]));
        if (edgeDistMap.find(first_edge) == edgeDistMap.end() && fnext(makeOrTri(i, 0)) != 0) {
            double point_a[3] = { vlist[tlist[i][0]][0], vlist[tlist[i][0]][1], vlist[tlist[i][0]][2] };
            double point_b[3] = { vlist[tlist[i][1]][0], vlist[tlist[i][1]][1], vlist[tlist[i][1]][2] };
            double dist = distance(point_a, point_b);
            edgeDistMap.insert({ first_edge, dist * 10000 });
            edgeDistancePQ.push({ -dist, first_edge });
            ecount++;
        }
        if (edgeDistMap.find(second_edge) == edgeDistMap.end() && fnext(makeOrTri(i, 1)) != 0) {
            double point_a[3] = { vlist[tlist[i][2]][0], vlist[tlist[i][2]][1], vlist[tlist[i][2]][2] };
            double point_b[3] = { vlist[tlist[i][1]][0], vlist[tlist[i][1]][1], vlist[tlist[i][1]][2] };
            double dist = distance(point_a, point_b);
            edgeDistMap.insert({ second_edge, dist * 10000 });
            edgeDistancePQ.push({ -dist, second_edge });
            ecount++;
        }
        if (edgeDistMap.find(third_edge) == edgeDistMap.end() && fnext(makeOrTri(i, 2)) != 0) {
            double point_a[3] = { vlist[tlist[i][0]][0], vlist[tlist[i][0]][1], vlist[tlist[i][0]][2] };
            double point_b[3] = { vlist[tlist[i][2]][0], vlist[tlist[i][2]][1], vlist[tlist[i][2]][2] };
            double dist = distance(point_a, point_b);
            edgeDistMap.insert({ third_edge, dist * 10000 });
            edgeDistancePQ.push({ -dist, third_edge });
            ecount++;
        }
    }
    cout << "Number of Edges: " << ecount << endl;
}

void myObjType::updateFnlist(OrTri t0, OrTri t1) {
    fnlist[idx(t0)][ver(t0)] = t1;
    fnlist[idx(t0)][ver(sym(t0))] = sym(t1);
    fnlist[idx(t1)][ver(t1)] = t0;
    fnlist[idx(t1)][ver(sym(t1))] = sym(t0);
}

void myObjType::createDuplicate() {
    for (int i = 1; i <= tcount; i++) {
        tlist2[i][0] = tlist[i][0];
        tlist2[i][1] = tlist[i][1];
        tlist2[i][2] = tlist[i][2];
        fnlist2[i][0] = fnlist[i][0];
        fnlist2[i][1] = fnlist[i][1];
        fnlist2[i][2] = fnlist[i][2];
        nlist2[i][0] = nlist[i][0];
        nlist2[i][1] = nlist[i][1];
        nlist2[i][2] = nlist[i][2];
    }
    for (int i = 1; i <= vcount; i++) {
        vlist2[i][0] = vlist[i][0];
        vlist2[i][1] = vlist[i][1];
        vlist2[i][2] = vlist[i][2];
        vnlist2[i][0] = vnlist[i][0];
        vnlist2[i][1] = vnlist[i][1];
        vnlist2[i][2] = vnlist[i][2];
    }
    edgemap2 = edgemap;
    vnamemap2 = vnamemap;
    vmap2 = vmap;
}

void myObjType::updateVmap3(int vIdx, int tIdx) {
    if (vmap3.find(vIdx) == vmap3.end()) {
        vmap3.insert({ vIdx, tIdx });
    }
}

void myObjType::draw(bool smooth) {
    glEnable(GL_LIGHTING);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    glPushMatrix();
    double longestSide = 0.0;
    for (int i = 0; i < 3; i++)
        if ((lmax[i] - lmin[i]) > longestSide)
            longestSide = (lmax[i] - lmin[i]);
    glScalef(4.0 / longestSide, 4.0 / longestSide, 4.0 / longestSide);
    glTranslated(-(lmin[0] + lmax[0]) / 2.0, -(lmin[1] + lmax[1]) / 2.0, -(lmin[2] + lmax[2]) / 2.0);
    for (int i = 1; i <= tcount; i++) {
        if (!tlistBitset.test(i)) {
            if (smooth) {
                // Lab 2 Optional 3: Smooth Shading
                glBegin(GL_POLYGON);
                glNormal3dv(vnlist[tlist[i][0]]);
                glVertex3dv(vlist[tlist[i][0]]);
                glNormal3dv(vnlist[tlist[i][1]]);
                glVertex3dv(vlist[tlist[i][1]]);
                glNormal3dv(vnlist[tlist[i][2]]);
                glVertex3dv(vlist[tlist[i][2]]);
                glEnd();
            } else {
                glBegin(GL_POLYGON);
                glNormal3dv(nlist[i]);
                for (int j = 0; j < 3; j++)
                    glVertex3dv(vlist[tlist[i][j]]);
                glEnd();
            }
        }
    }
    // Lab 2 Optional 4: Visualise boundary edges
    colorBoundaryEdge();
    glDisable(GL_LIGHTING);
    glPopMatrix();
}

void myObjType::writeFile(char* filename) {
    // Lab 1 Subtask 3: Write an OBJ file
    ofstream outFile(filename);
    for (int i = 1; i <= vcount; i++) {
        outFile << "v " << vlist[i][0] << " " << vlist[i][1] << " " << vlist[i][2] << "\n";
    }
    for (int i = 1; i <= tcount; i++) {
        outFile << "f " << tlist[i][0] << " " << tlist[i][1] << " " << tlist[i][2] << "\n";
    }
    outFile.close();
    cout << "Writing done!" << endl;
}

void myObjType::readFile(char* filename) {
    cout << "Opening " << filename << endl;
    ifstream inFile;
    inFile.open(filename);
    if (!inFile.is_open()) {
        cout << "We cannot find your file " << filename << endl;
        exit(1);
    }
    string line;
    int i, j;
    bool firstVertex = 1;
    double currCood;
    while (getline(inFile, line)) {
        if ((line[0] == 'v' || line[0] == 'f') && line[1] == ' ') {
            if (line[0] == 'v') {
                vcount++;
                i = 1;
                const char* linec = line.data();
                for (int k = 0; k < 3; k++) { // k is 0,1,2 for x,y,z
                    while (linec[i] == ' ') i++;
                    j = i;
                    while (linec[j] != ' ') j++;
                    currCood = vlist[vcount][k] = atof(line.substr(i, j - i).c_str());
                    if (firstVertex)
                        lmin[k] = lmax[k] = currCood;
                    else {
                        if (lmin[k] > currCood)
                            lmin[k] = currCood;
                        if (lmax[k] < currCood)
                            lmax[k] = currCood;
                    }
                    i = j;
                }
                firstVertex = 0;
            }
            if (line[0] == 'f') {
                tcount++;
                i = 1;
                const char* linec = line.data();
                for (int k = 0; k < 3; k++) {
                    while (linec[i] == ' ') i++;
                    j = i;
                    while (linec[j] != ' ' && linec[j] != '\\') j++;
                    int vIdx = atof(line.substr(i, j - i).c_str());
                    tlist[tcount][k] = vIdx;
                    updateVmap(vIdx, tcount);
                    i = j;
                    fnlist[tcount][k] = 0;
                    while (linec[j] != ' ') j++;
                }
            }
        }
    }
    computeFnlist();
    cout << "No. of vertices: " << vcount << endl;
    cout << "No. of triangles: " << tcount << endl;
    computeStat();
    computeComponents();
    orientTriangles();
    // Lab 1 Subtask 1: Compute Normal Vectors
    computeNormals();
    computeVertexNormals();

    computeEdges();
    createDuplicate();
}

void myObjType::readStlFile(char* filename) {
    // Lab 1 Optional: Read Other File Type (STL)
    cout << "Opening " << filename << endl;
    ifstream inFile(filename, ios::in | ios::binary);
    if (!inFile) {
        cout << "We cannot find your file " << filename << endl;
        exit(1);
    }
    char header_info[80] = "";
    char num_tri[4];
    long num_triangle;
    inFile.read(header_info, 80);
    inFile.read(num_tri, 4);
    num_triangle = *(long*)num_tri;
    double currCood;
    bool firstVertex = 1;
    for (int i = 0; i < num_triangle; i++) {
        for (int j = 0; j < 3; j++) {
            char buf[4];
            inFile.read(buf, 4);
            //nlist[i][j] = *(float*)buf;
        }
        // vertices and triangles
        for (int k = 1; k <= 3; k++) {
            string vertex_name;
            double vertex_coord[3];
            for (int j = 0; j < 3; j++) {
                char buf[4];
                inFile.read(buf, 4);
                currCood = vertex_coord[j] = *(float*)buf;
                vertex_name += to_string(*(float*)buf);
                if (firstVertex)
                    lmin[j] = lmax[j] = currCood;
                else {
                    if (lmin[j] > currCood)
                        lmin[j] = currCood;
                    if (lmax[j] < currCood)
                        lmax[j] = currCood;
                }
                firstVertex = 0;
            }
            if (vnamemap.find(vertex_name) != vnamemap.end()) {
                tlist[i + 1][k - 1] = vnamemap.at(vertex_name);
            } else {
                vcount++;
                vlist[vcount][0] = vertex_coord[0];
                vlist[vcount][1] = vertex_coord[1];
                vlist[vcount][2] = vertex_coord[2];
                vnamemap.insert({ vertex_name, vcount });
                tlist[i + 1][k - 1] = vcount;
                updateVmap(vcount, i + 1);
            }
        }
        char dummy[2];
        inFile.read(dummy, 2);
        tcount++;
    }
    computeFnlist();
    cout << "No. of vertices: " << vcount << endl;
    cout << "No. of triangles: " << tcount << endl;
    computeStat();
    computeComponents();
    orientTriangles();
    computeNormals();
    computeVertexNormals();
}

void myObjType::computeStat() {
    int i;
    // Lab 1 Subtask 2: Compute Angle Statistics
    double minAngle = INT_MAX;
    double maxAngle = INT_MIN;
    for (int i = 1; i <= tcount; i++) {
        if (!tlistBitset.test(i)) {
            double point_a[3];
            double point_b[3];
            double point_c[3];
            double angles[3];
            double localMinAngle = INT_MAX;
            double localMaxAngle = INT_MIN;
            for (int j = 0; j < 3; j++) {
                point_a[j] = vlist[tlist[i][0]][j];
                point_b[j] = vlist[tlist[i][1]][j];
                point_c[j] = vlist[tlist[i][2]][j];
            }
            angles[0] = angle(distance(point_a, point_b), distance(point_a, point_c), distance(point_b, point_c));
            angles[1] = angle(distance(point_b, point_c), distance(point_b, point_a), distance(point_c, point_a));
            angles[2] = angle(distance(point_c, point_a), distance(point_c, point_b), distance(point_a, point_b));
            for (int j = 0; j < 3; j++) {
                localMinAngle = min(localMinAngle, angles[j]);
                localMaxAngle = max(localMaxAngle, angles[j]);
            }
            statMaxAngle[(int)localMaxAngle / 10]++;
            statMinAngle[(int)localMinAngle / 10]++;
            minAngle = min(localMinAngle, minAngle);
            maxAngle = max(localMaxAngle, maxAngle);
        }
    }

    cout << "Min. angle = " << minAngle << endl;
    cout << "Max. angle = " << maxAngle << endl;

    cout << "Statistics for Maximum Angles" << endl;
    for (i = 0; i < 18; i++)
        cout << statMaxAngle[i] << " ";
    cout << endl;
    cout << "Statistics for Minimum Angles" << endl;
    for (i = 0; i < 18; i++)
        cout << statMinAngle[i] << " ";
    cout << endl;
}

void myObjType::computeNormals() {
    // Lab 1 Subtask 1: Compute Normal Vectors
    for (int i = 1; i <= tcount; i++) {
        if (!tlistBitset.test(i)) {
            double point_a[3];
            double point_b[3];
            double point_c[3];
            double v_0[3];
            double v_1[3];
            for (int j = 0; j < 3; j++) {
                point_a[j] = vlist[tlist[i][0]][j];
                point_b[j] = vlist[tlist[i][1]][j];
                point_c[j] = vlist[tlist[i][2]][j];
            }
            v_0[0] = point_b[0] - point_a[0];
            v_0[1] = point_b[1] - point_a[1];
            v_0[2] = point_b[2] - point_a[2];
            v_1[0] = point_c[0] - point_a[0];
            v_1[1] = point_c[1] - point_a[1];
            v_1[2] = point_c[2] - point_a[2];

            nlist[i][0] = v_0[1] * v_1[2] - v_0[2] * v_1[1];
            nlist[i][1] = -(v_0[0] * v_1[2] - v_0[2] * v_1[0]);
            nlist[i][2] = v_0[0] * v_1[1] - v_0[1] * v_1[0];
        }
    }
}

void myObjType::computeFnlist() {
    // Lab 2 Subtask 3: Implement fnext
    for (int i = 1; i <= tcount; i++) {
        if (!tlistBitset.test(i)) {
            int v0 = tlist[i][0];
            int v1 = tlist[i][1];
            int v2 = tlist[i][2];
            // consider edge v0-v1
            if (edgemap.find(make_pair(min(v0, v1), max(v0, v1))) != edgemap.end()) {
                int neighbour = edgemap.at(make_pair(min(v0, v1), max(v0, v1)));
                fnlist[i][0] = neighbour << 3 | computeVersionFromVertices(neighbour, v0, v1);
                fnlist[i][3] = neighbour << 3 | computeVersionFromVertices(neighbour, v1, v0);
                fnlist[neighbour][computeVersionFromVertices(neighbour, v0, v1)] = i << 3 | 0;
                fnlist[neighbour][computeVersionFromVertices(neighbour, v1, v0)] = i << 3 | 3;
                edgemap.erase(make_pair(min(v0, v1), max(v0, v1)));
            } else {
                edgemap.insert(pair<pair<int, int>, int>(make_pair(min(v0, v1), max(v0, v1)), i));
            }
            // consider edge v1-v2
            if (edgemap.find(make_pair(min(v1, v2), max(v1, v2))) != edgemap.end()) {
                int neighbour = edgemap.at(make_pair(min(v1, v2), max(v1, v2)));
                fnlist[i][1] = neighbour << 3 | computeVersionFromVertices(neighbour, v1, v2);
                fnlist[i][4] = neighbour << 3 | computeVersionFromVertices(neighbour, v2, v1);
                fnlist[neighbour][computeVersionFromVertices(neighbour, v1, v2)] = i << 3 | 1;
                fnlist[neighbour][computeVersionFromVertices(neighbour, v2, v1)] = i << 3 | 4;
                edgemap.erase(make_pair(min(v1, v2), max(v1, v2)));
            } else {
                edgemap.insert(pair<pair<int, int>, int>(make_pair(min(v1, v2), max(v1, v2)), i));
            }
            // consider edge v2-v0
            if (edgemap.find(make_pair(min(v2, v0), max(v2, v0))) != edgemap.end()) {
                int neighbour = edgemap.at(make_pair(min(v2, v0), max(v2, v0)));
                fnlist[i][2] = neighbour << 3 | computeVersionFromVertices(neighbour, v2, v0);
                fnlist[i][5] = neighbour << 3 | computeVersionFromVertices(neighbour, v0, v2);
                fnlist[neighbour][computeVersionFromVertices(neighbour, v2, v0)] = i << 3 | 2;
                fnlist[neighbour][computeVersionFromVertices(neighbour, v0, v2)] = i << 3 | 5;
                edgemap.erase(make_pair(min(v2, v0), max(v2, v0)));
            } else {
                edgemap.insert(pair<pair<int, int>, int>(make_pair(min(v2, v0), max(v2, v0)), i));
            }
        }
    }
}

void myObjType::computeComponents() {
    // Lab 2 Optional 1: Compute number of components
    int num_components = 0;

    vector<int> visited;
    for (int i = 0; i <= tcount; i++) {
        visited.push_back(0);
    }

    for (int i = 1; i <= tcount; i++) {
        if (!tlistBitset.test(i)) {
            if (visited[i] == 0) { // not visited before
                int s = i;
                list<int> queue;
                visited[s] = 1;
                queue.push_back(s);

                while (!queue.empty()) {
                    s = queue.front();
                    queue.pop_front();
                    if (fnext(makeOrTri(s, 0)) != 0) {
                        int first_neighbour_tIDx = idx(fnext(makeOrTri(s, 0)));
                        if (visited[first_neighbour_tIDx] == 0) {
                            visited[first_neighbour_tIDx] = 1;
                            queue.push_back(first_neighbour_tIDx);
                        }
                    }
                    if (fnext(makeOrTri(s, 1)) != 0) {
                        int second_neighbour_tIDx = idx(fnext(makeOrTri(s, 1)));
                        if (visited[second_neighbour_tIDx] == 0) {
                            visited[second_neighbour_tIDx] = 1;
                            queue.push_back(second_neighbour_tIDx);
                        }
                    }
                    if (fnext(makeOrTri(s, 2)) != 0) {
                        int third_neighbour_tIDx = idx(fnext(makeOrTri(s, 2)));
                        if (visited[third_neighbour_tIDx] == 0) {
                            visited[third_neighbour_tIDx] = 1;
                            queue.push_back(third_neighbour_tIDx);
                        }
                    }
                }
                num_components++;
            }
        }
    }
    cout << "Number of Components: " << num_components << endl;
}

bool myObjType::orientTriangles() {
    // Lab 2 Optional 2: Implement orientTriangles();
    int num_oriented = 0;
    bool success = true;

    vector<int> visited;
    for (int i = 0; i <= tcount; i++) {
        visited.push_back(0);
    }

    for (int i = 1; i <= tcount; i++) {
        if (!tlistBitset.test(i)) {
            if (visited[i] == 0) { // not visited before
                int s = i;
                list<int> queue;
                visited[s] = 1;
                queue.push_back(s);

                while (!queue.empty()) {
                    s = queue.front();
                    queue.pop_front();
                    if (fnext(makeOrTri(s, 0)) != 0) {
                        int first_neighbour_tIDx = idx(fnext(makeOrTri(s, 0)));
                        if (visited[first_neighbour_tIDx] == 0) {
                            if (!isSameOrientation(first_neighbour_tIDx, tlist[s][1], tlist[s][0])) {
                                orientTriangle(first_neighbour_tIDx);
                                num_oriented++;
                            }
                            visited[first_neighbour_tIDx] = 1;
                            queue.push_back(first_neighbour_tIDx);
                        } else {
                            if (!isSameOrientation(first_neighbour_tIDx, tlist[s][1], tlist[s][0])) {
                                cout << "Unable to orient triangles!" << endl;
                                return false;
                            }
                        }
                    }
                    if (fnext(makeOrTri(s, 1)) != 0) {
                        int second_neighbour_tIDx = idx(fnext(makeOrTri(s, 1)));
                        if (visited[second_neighbour_tIDx] == 0) {
                            if (!isSameOrientation(second_neighbour_tIDx, tlist[s][2], tlist[s][1])) {
                                orientTriangle(second_neighbour_tIDx);
                                num_oriented++;
                            }
                            visited[second_neighbour_tIDx] = 1;
                            queue.push_back(second_neighbour_tIDx);
                        } else {
                            if (!isSameOrientation(second_neighbour_tIDx, tlist[s][2], tlist[s][1])) {
                                cout << "Unable to orient triangles!" << endl;
                                return false;
                            }
                        }
                    }
                    if (fnext(makeOrTri(s, 2)) != 0) {
                        int third_neighbour_tIDx = idx(fnext(makeOrTri(s, 2)));
                        if (visited[third_neighbour_tIDx] == 0) {
                            if (!isSameOrientation(third_neighbour_tIDx, tlist[s][0], tlist[s][2])) {
                                orientTriangle(third_neighbour_tIDx);
                                num_oriented++;
                            }
                            visited[third_neighbour_tIDx] = 1;
                            queue.push_back(third_neighbour_tIDx);
                        } else {
                            if (!isSameOrientation(third_neighbour_tIDx, tlist[s][0], tlist[s][2])) {
                                cout << "Unable to orient triangles!" << endl;
                                return false;
                            }
                        }
                    }
                }
            }
        }
    }
    cout << "Number of Triangles Oriented: " << num_oriented << endl;
    return false;
}

void myObjType::computeVertexNormals() {
    // Lab 2 Optional 3: Smooth Shading
    for (int i = 1; i <= vcount; i++) {
        if (!vlistBitset.test(i)) {
            int neighbourTIdx = vmap.at(i);
            int neighbourVersion;
            if (tlist[neighbourTIdx][0] == i) neighbourVersion = 0;
            if (tlist[neighbourTIdx][1] == i) neighbourVersion = 1;
            if (tlist[neighbourTIdx][2] == i) neighbourVersion = 2;
            int numNeighbours = 0;
            double normalSumX = 0;
            double normalSumY = 0;
            double normalSumZ = 0;
            unordered_set<int> neighboursVisited;
            OrTri currTriangle = makeOrTri(neighbourTIdx, neighbourVersion);
            while (neighboursVisited.find(currTriangle) == neighboursVisited.end()) {
                // compute normal for currTriangle
                double point_a[3];
                double point_b[3];
                double point_c[3];
                double v_0[3];
                double v_1[3];
                for (int j = 0; j < 3; j++) {
                    point_a[j] = vlist[tlist[idx(currTriangle)][0]][j];
                    point_b[j] = vlist[tlist[idx(currTriangle)][1]][j];
                    point_c[j] = vlist[tlist[idx(currTriangle)][2]][j];
                }
                v_0[0] = point_b[0] - point_a[0];
                v_0[1] = point_b[1] - point_a[1];
                v_0[2] = point_b[2] - point_a[2];
                v_1[0] = point_c[0] - point_a[0];
                v_1[1] = point_c[1] - point_a[1];
                v_1[2] = point_c[2] - point_a[2];

                // normalise
                double x = v_0[1] * v_1[2] - v_0[2] * v_1[1];
                double y = -(v_0[0] * v_1[2] - v_0[2] * v_1[0]);
                double z = v_0[0] * v_1[1] - v_0[1] * v_1[0];
                double magnitude = sqrt(x * x + y * y + z * z);
                normalSumX += x / magnitude;
                normalSumY += y / magnitude;
                normalSumZ += z / magnitude;

                neighboursVisited.insert({ currTriangle });
                currTriangle = enext(sym(fnext(currTriangle)));
            }
            vnlist[i][0] = normalSumX;
            vnlist[i][1] = normalSumY;
            vnlist[i][2] = normalSumZ;
        }
    }
}

void myObjType::colorBoundaryEdge() {
    // Lab 2 Optional 4: Visualise boundary edge
    float mat_ambient[] = { 0.1745f, 0.01175f, 0.01175f, 0.55f };
    float mat_diffuse[] = { 0.61424f, 0.04136f, 0.04136f, 0.55f };
    float mat_specular[] = { 0.727811f, 0.626959f, 0.626959f, 0.55f };
    float shininess = 76.8f;
    glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
    glMaterialf(GL_FRONT, GL_SHININESS, shininess);
    OrTri currTri;
    glLineWidth(4);
    for (int i = 1; i <= tcount; i++) {
        if (!tlistBitset.test(i)) {
            currTri = makeOrTri(i, 0);
            for (int j = 0; j < 3; j++) {
                if (fnlist[i][j] == 0) {
                    glBegin(GL_LINES);
                    glVertex3dv(vlist[org(currTri)]);
                    glVertex3dv(vlist[dest(currTri)]);
                    glEnd();
                }
                currTri = enext(currTri);
            }
        }
    }
    glLineWidth(1);
}

OrTri myObjType::enext(OrTri orTri) {
    // Lab 2 Subtask 1: Implement enext
    tIdx tIdx = idx(orTri);
    int tVersion = ver(orTri);
    switch (tVersion) {
        case 0:
            return (tIdx << 3) | 1;
            break;
        case 1:
            return (tIdx << 3) | 2;
            break;
        case 2:
            return (tIdx << 3) | 0;
            break;
        case 3:
            return (tIdx << 3) | 5;
            break;
        case 4:
            return (tIdx << 3) | 3;
            break;
        case 5:
            return (tIdx << 3) | 4;
            break;
    }
    return -1;
}

int myObjType::sym(OrTri orTri) {
    // Lab 2 Subtask 1: Implement sym
    tIdx tIdx = idx(orTri);
    int tVersion = ver(orTri);
    switch (tVersion) {
    case 0:
        return (tIdx << 3) | 3;
        break;
    case 1:
        return (tIdx << 3) | 4;
        break;
    case 2:
        return (tIdx << 3) | 5;
        break;
    case 3:
        return (tIdx << 3) | 0;
        break;
    case 4:
        return (tIdx << 3) | 1;
        break;
    case 5:
        return (tIdx << 3) | 2;
        break;
    }
    return -1;
}

int myObjType::org(OrTri orTri) {
    // Lab 2 Subtask 2: Implement org
    tIdx tIdx = idx(orTri);
    int tVersion = ver(orTri);
    int vertexIdx;
    switch (tVersion) {
    case 0:
        vertexIdx = 0;
        break;
    case 1:
        vertexIdx = 1;
        break;
    case 2:
        vertexIdx = 2;
        break;
    case 3:
        vertexIdx = 1;
        break;
    case 4:
        vertexIdx = 2;
        break;
    case 5:
        vertexIdx = 0;
        break;
    }
    return tlist[tIdx][vertexIdx];
}

OrTri myObjType::dest(OrTri orTri) {
    // Lab 2 Subtask 2: Implement dest
    tIdx tIdx = idx(orTri);
    int tVersion = ver(orTri);
    int vertexIdx;
    switch (tVersion) {
    case 0:
        vertexIdx = 1;
        break;
    case 1:
        vertexIdx = 2;
        break;
    case 2:
        vertexIdx = 0;
        break;
    case 3:
        vertexIdx = 0;
        break;
    case 4:
        vertexIdx = 1;
        break;
    case 5:
        vertexIdx = 2;
        break;
    }
    return tlist[tIdx][vertexIdx];
}

OrTri myObjType::fnext(OrTri orTri) {
    // Lab 2 Subtask 3: Implement fnext
    tIdx tIdx = idx(orTri);
    int tVersion = ver(orTri);
    return fnlist[tIdx][tVersion];
}

void myObjType::edgeContraction() {
    if (edgeDistancePQ.empty()) {
        cout << "No more edge to contract!" << endl;
        return;
    }
    pair<int, int> top_edge = edgeDistancePQ.top().second;
    double dist = edgeDistancePQ.top().first;
    edgeDistancePQ.pop();
    int v0 = top_edge.first;
    int v1 = top_edge.second;
    bool contracted = false;
    if ((edgeDistMap.find(top_edge) != edgeDistMap.end()) &&
        ((long)(-dist * 10000) == edgeDistMap.at(top_edge)) && (v0 != v1)) {
        OrTri v0firstBoundaryTri;
        OrTri v0secondBoundaryTri;
        OrTri v1firstBoundaryTri;
        OrTri v1secondBoundaryTri;
        int deletedTri0;
        int deletedTri1;
        int edgeVertexToDelete0;
        int edgeVertexToDelete1;

        // keep v0, keep track of triangles deleted and the boundary triangles
        int v0neighbourTIdx = vmap.at(v0);
        int v0neighbourVersion;
        if (tlist[v0neighbourTIdx][0] == v0) v0neighbourVersion = 0;
        if (tlist[v0neighbourTIdx][1] == v0) v0neighbourVersion = 1;
        if (tlist[v0neighbourTIdx][2] == v0) v0neighbourVersion = 2;
        OrTri v0currTriangle = makeOrTri(v0neighbourTIdx, v0neighbourVersion);
        while (true) {
            if (dest(v0currTriangle) == v1) {
                // reach deleted edge, and keep track of boundary triangles and  deleted triangles
                deletedTri0 = idx(v0currTriangle);
                deletedTri1 = idx(fnext(v0currTriangle));
                v0firstBoundaryTri = fnext(enext(sym(v0currTriangle)));
                v0secondBoundaryTri = fnext(enext(sym(fnext(v0currTriangle))));
                break;
            }
            v0currTriangle = enext(sym(fnext(v0currTriangle)));
        }

        // discard v1, update triangle vertices, update fnext
        int v1neighbourTIdx = vmap.at(v1);
        int v1neighbourVersion;
        if (tlist[v1neighbourTIdx][0] == v1) v1neighbourVersion = 0;
        if (tlist[v1neighbourTIdx][1] == v1) v1neighbourVersion = 1;
        if (tlist[v1neighbourTIdx][2] == v1) v1neighbourVersion = 2;
        unordered_set<int> v1neighboursVisited;
        OrTri v1currTriangle = makeOrTri(v1neighbourTIdx, v1neighbourVersion);
        while (v1neighboursVisited.find(v1currTriangle) == v1neighboursVisited.end()) {
            // update all triangle vertex v0 to v1
            if (tlist[idx(v1currTriangle)][0] == v1) tlist[idx(v1currTriangle)][0] = v0;
            if (tlist[idx(v1currTriangle)][1] == v1) tlist[idx(v1currTriangle)][1] = v0;
            if (tlist[idx(v1currTriangle)][2] == v1) tlist[idx(v1currTriangle)][2] = v0;
            // reach deleted edge
            if (dest(v1currTriangle) == v0) {
                // keep track of boundary triangles
                v1firstBoundaryTri = fnext(enext(sym(fnext(v1currTriangle))));
                v1secondBoundaryTri = fnext(enext(sym(v1currTriangle)));
            }
            // update edge dist map (delete existing edges and insert new edges)
            edgeDistMap.erase(make_pair(min(v1, dest(v1currTriangle)), max(v1, dest(v1currTriangle))));
            if (v0 != dest(v1currTriangle)) {
                double point_a[3] = { vlist[v1][0], vlist[v1][1], vlist[v1][2] };
                double point_b[3] = { vlist[dest(v1currTriangle)][0], vlist[dest(v1currTriangle)][1], vlist[dest(v1currTriangle)][2] };
                double new_dist = distance(point_a, point_b);
                pair<int, int> new_edge = make_pair(min(v0, dest(v1currTriangle)), max(v0, dest(v1currTriangle)));
                edgeDistMap.insert({ new_edge, new_dist });
                edgeDistancePQ.push({ -new_dist, new_edge });
            }
            v1neighboursVisited.insert({ v1currTriangle });
            v1currTriangle = enext(sym(fnext(v1currTriangle)));
        }

        // delete 2 triangles
        tlistBitset.set(deletedTri0);
        tlistBitset.set(deletedTri1);

        // delete v1
        vlistBitset.set(v1);

        // update fnlist for boundary triangles
        updateFnlist(v0firstBoundaryTri, v1firstBoundaryTri);
        updateFnlist(v0secondBoundaryTri, v1secondBoundaryTri);

        // update vmap of v0
        if (vmap[v0] == deletedTri0 || vmap[v0] == deletedTri1) {
            if (idx(v0firstBoundaryTri) != 0) {
                vmap[v0] = idx(v0firstBoundaryTri);
            } else {
                vmap[v0] = idx(v0secondBoundaryTri);
            }
        }
        contracted = true;
        cout << "Edge contracted: " << v0 << "-" << v1 << endl;
    }
    if (!contracted) edgeContraction();
}

void myObjType::restoreInitial() {
    for (int i = 1; i <= tcount; i++) {
        tlist[i][0] = tlist2[i][0];
        tlist[i][1] = tlist2[i][1];
        tlist[i][2] = tlist2[i][2];
        fnlist[i][0] = fnlist2[i][0];
        fnlist[i][1] = fnlist2[i][1];
        fnlist[i][2] = fnlist2[i][2];
        nlist[i][0] = nlist2[i][0];
        nlist[i][1] = nlist2[i][1];
        nlist[i][2] = nlist2[i][2];
    }
    for (int i = 1; i <= vcount; i++) {
        vlist[i][0] = vlist2[i][0];
        vlist[i][1] = vlist2[i][1];
        vlist[i][2] = vlist2[i][2];
        vnlist[i][0] = vnlist2[i][0];
        vnlist[i][1] = vnlist2[i][1];
        vnlist[i][2] = vnlist2[i][2];
    }
    edgemap = edgemap2;
    vnamemap = vnamemap2;
    vmap = vmap2;
    computeEdges();
    vlistBitset.reset();
    tlistBitset.reset();
}

void myObjType::readFile3(char * filename) {
    cout << "Opening " << filename << endl;
    ifstream inFile;
    inFile.open(filename);
    if (!inFile.is_open()) {
        cout << "We cannot find your file " << filename << endl;
        exit(1);
    }
    string line;
    int i, j;
    bool firstVertex = 1;
    double currCood;
    while (getline(inFile, line)) {
        if ((line[0] == 'v' || line[0] == 'f') && line[1] == ' ') {
            if (line[0] == 'v') {
                vcount3++;
                i = 1;
                const char* linec = line.data();
                for (int k = 0; k < 3; k++) { // k is 0,1,2 for x,y,z
                    while (linec[i] == ' ') i++;
                    j = i;
                    while (linec[j] != ' ') j++;
                    currCood = vlist3[vcount3][k] = atof(line.substr(i, j - i).c_str());
                    if (firstVertex)
                        lmin3[k] = lmax3[k] = currCood;
                    else {
                        if (lmin3[k] > currCood)
                            lmin3[k] = currCood;
                        if (lmax3[k] < currCood)
                            lmax3[k] = currCood;
                    }
                    i = j;
                }
                firstVertex = 0;
            }
            if (line[0] == 'f') {
                tcount3++;
                i = 1;
                const char* linec = line.data();
                for (int k = 0; k < 3; k++) {
                    while (linec[i] == ' ') i++;
                    j = i;
                    while (linec[j] != ' ' && linec[j] != '\\') j++;
                    int vIdx = atof(line.substr(i, j - i).c_str());
                    tlist3[tcount3][k] = vIdx;
                    updateVmap3(vIdx, tcount3);
                    i = j;
                    fnlist3[tcount3][k] = 0;
                    while (linec[j] != ' ') j++;
                }
            }
        }
    }
    computeFnlist3();
    cout << "No. of vertices in second object: " << vcount << endl;
    cout << "No. of triangles in second object: " << tcount << endl;
    computeNormals3();
    computeVertexNormals3();
}

std::vector<int> myObjType::getNearestDistanceIndices() {
    vector<int> neighbours;
    neighbours.push_back(0); // make it 1 based
    for (int i = 1; i <= vcount; i++) {
        double shortest_dist = 999999999;
        int neighbour;
        for (int j = 1; j <= vcount3; j++) {
            double point_a[3] = { vlist[i][0], vlist[i][1], vlist[i][2] };
            double point_b[3] = { vlist3[j][0], vlist3[j][1], vlist3[j][2] };
            if (distance(point_a, point_b) < shortest_dist) {
                shortest_dist = distance(point_a, point_b);
                neighbour = j;
            }
        }
        neighbours.push_back(neighbour);
    }
    return neighbours;
}

std::vector<int> myObjType::getNearestDistanceIndicesFast() {
    // build kd tree
    double best_dist = std::numeric_limits<double>::max();
    node* all_points;
    all_points = (node*)calloc(vcount3, sizeof(node));
    for (int j = 1; j <= vcount3; j++) {
        all_points[j - 1] = { {vlist3[j][0], vlist3[j][1], vlist3[j][2]}, j };
    }
    node *root, *found;
    node testNode;
    root = buildTree(all_points, vcount3, 0);
    // get closest neighbours
    vector<int> neighbours;
    neighbours.push_back(0); // make it 1 based
    for (int i = 1; i <= vcount; i++) {
        int neighbour;
        found = 0;
        testNode = { {vlist[i][0], vlist[i][1], vlist[i][2]}, i };
        findNearest(root, &testNode, 0, &found, &best_dist);
        neighbour = found->vertex_index;
        neighbours.push_back(neighbour);
    }
    delete[] all_points;
    all_points = NULL;
    return neighbours;
}

std::vector<int> myObjType::getNearestDistanceIndicesFastSecondVersion() {
    // build kd tree
    pointVec points;
    point_t pt(3);
    for (int j = 1; j <= vcount3; j++) {
        pt = { vlist3[j][0], vlist3[j][1], vlist3[j][2] };
        points.push_back(pt);
    }
    KDTree tree(points);
    // get closest neighbours
    vector<int> neighbours;
    neighbours.push_back(0); // make it 1 based
    for (int i = 1; i <= vcount; i++) {
        pt = { vlist[i][0], vlist[i][1], vlist[i][2] };
        int neighbour = tree.nearest_index(pt) + 1;
        neighbours.push_back(neighbour);
    }
    return neighbours;
}

void myObjType::ICP() {
    int max_iter = 1000;
    double tolerance = 0.02;
    double prev_error = 1000;
    double epsilon = 0.0001;
    vector<int> neighboursIndices;
    while (max_iter > 0) {
        // compute shortest pairs
        neighboursIndices = getNearestDistanceIndicesFastSecondVersion();
        // compute errors
        double error = 0;
        double avgError = 0;
        for (int i = 1; i <= vcount; i++) {
            double point_a[3] = { vlist[i][0], vlist[i][1], vlist[i][2] };
            double point_b[3] = { vlist3[neighboursIndices[i]][0], vlist3[neighboursIndices[i]][1], vlist3[neighboursIndices[i]][2] };
            error += distance(point_a, point_b);
        }
        avgError = error / vcount;
        cout << "Average error is : " << avgError << endl;
        if (avgError <= tolerance || abs(prev_error - avgError) < epsilon) {
            break; // break when average error is within tolerance or no improvement
        }
        prev_error = avgError;
        // compute centroid for first set
        double centroid_x0 = 0;
        double centroid_y0 = 0;
        double centroid_z0 = 0;
        for (int i = 1; i <= vcount; i++) {
            centroid_x0 += vlist[i][0];
            centroid_y0 += vlist[i][1];
            centroid_z0 += vlist[i][2];
        }
        centroid_x0 /= vcount;
        centroid_y0 /= vcount;
        centroid_z0 /= vcount;
        // compute centroid for second set
        double centroid_x1 = 0;
        double centroid_y1 = 0;
        double centroid_z1 = 0;
        for (int i = 1; i <= vcount; i++) {
            centroid_x1 += vlist3[neighboursIndices[i]][0];
            centroid_y1 += vlist3[neighboursIndices[i]][1];
            centroid_z1 += vlist3[neighboursIndices[i]][2];
        }
        centroid_x1 /= vcount;
        centroid_y1 /= vcount;
        centroid_z1 /= vcount;
        // translate first set by centroid
        for (int i = 1; i <= vcount; i++) {
            vlist[i][0] -= centroid_x0;
            vlist[i][1] -= centroid_y0;;
            vlist[i][2] -= centroid_z0;;
        }
        // translate second set by centroid
        for (int i = 1; i <= vcount3; i++) {
            vlist3[i][0] -= centroid_x1;
            vlist3[i][1] -= centroid_y1;;
            vlist3[i][2] -= centroid_z1;;
        }
        // compute RTS
        MatrixXd RTS = MatrixXd::Zero(4, 4);
        for (int i = 1; i <= vcount; i++) {
            MatrixXd RT(4, 4);
            double x = vlist[i][0];
            double y = vlist[i][1];
            double z = vlist[i][2];
            RT << 1, x, y, z,
                -x, 1, -z, y,
                -y, z, 1, -x,
                -z, -y, x, 1;
            MatrixXd S(4, 4);
            double x0 = vlist3[neighboursIndices[i]][0];
            double y0 = vlist3[neighboursIndices[i]][1];
            double z0 = vlist3[neighboursIndices[i]][2];
            S << 1, -x0, -y0, -z0,
                x0, 1, -z0, y0,
                y0, z0, 1, -x0,
                z0, -y0, x0, 1;
            RTS = RTS + RT * S;
        }
        // compute eigenvector with biggest eigenvalue
        EigenSolver<Matrix4d> eigensolver(RTS);
        double largest_eigenvalue = 0;
        double largest_eigenvalue_index = 0;
        for (int i = 0; i < 4; i++) {
            if (eigensolver.eigenvalues()[i].real() > largest_eigenvalue && eigensolver.eigenvalues()[i].imag() == 0) {
                largest_eigenvalue = eigensolver.eigenvalues()[i].real();
                largest_eigenvalue_index = i;
            }
        }
        VectorXcd largest_eigenvector = eigensolver.eigenvectors().col(largest_eigenvalue_index);
        cout << "The largest eigenvector of RTS is:" << endl << largest_eigenvector << endl;
        // get q_w, q_x, q_y, q_z
        double q_w = largest_eigenvector[0].real();
        double q_x = largest_eigenvector[1].real();
        double q_y = largest_eigenvector[2].real();
        double q_z = largest_eigenvector[3].real();
        // rotate all first set points by q
        MatrixXd R(3, 3);
        R << (q_w * q_w + q_x * q_x - q_y * q_y - q_z * q_z), (2 * (q_x * q_y - q_w * q_z)), (2 * (q_x * q_z + q_w * q_y)),
            (2 * (q_x * q_y + q_w * q_z)), (q_w * q_w - q_x * q_x + q_y * q_y - q_z * q_z), (2 * (q_y * q_z - q_w * q_x)),
            (2 * (q_x * q_z - q_w * q_y)), (2 * (q_y * q_z + q_w * q_x)), (q_w * q_w - q_x * q_x - q_y * q_y + q_z * q_z);
        for (int i = 1; i <= vcount; i++) {
            double x = vlist[i][0];
            double y = vlist[i][1];
            double z = vlist[i][2];
            VectorXd point(3);
            point << vlist[i][0], vlist[i][1], vlist[i][2];
            VectorXd rotatedPoint(3);
            rotatedPoint = R * point;
            vlist[i][0] = rotatedPoint[0];
            vlist[i][1] = rotatedPoint[1];
            vlist[i][2] = rotatedPoint[2];
        }
        cout << "Points in first set are rotated by: " << 2 * acos(q_w) * 180 / 3.14159 << " degree" << endl;
        max_iter--;
    }
    finalNeighbourIndices = neighboursIndices;
}

void myObjType::ICPSlow() {
    int max_iter = 1000;
    double tolerance = 0.02;
    double prev_error = 1000;
    double epsilon = 0.0001;
    vector<int> neighboursIndices;
    while (max_iter > 0) {
        // compute shortest pairs
        neighboursIndices = getNearestDistanceIndices();
        // compute errors
        double error = 0;
        double avgError = 0;
        for (int i = 1; i <= vcount; i++) {
            double point_a[3] = { vlist[i][0], vlist[i][1], vlist[i][2] };
            double point_b[3] = { vlist3[neighboursIndices[i]][0], vlist3[neighboursIndices[i]][1], vlist3[neighboursIndices[i]][2] };
            error += distance(point_a, point_b);
        }
        avgError = error / vcount;
        cout << "Average error is : " << avgError << endl;
        if (avgError <= tolerance || abs(prev_error - avgError) < epsilon) {
            break; // break when average error is within tolerance or no improvement
        }
        prev_error = avgError;
        // compute centroid for first set
        double centroid_x0 = 0;
        double centroid_y0 = 0;
        double centroid_z0 = 0;
        for (int i = 1; i <= vcount; i++) {
            centroid_x0 += vlist[i][0];
            centroid_y0 += vlist[i][1];
            centroid_z0 += vlist[i][2];
        }
        centroid_x0 /= vcount;
        centroid_y0 /= vcount;
        centroid_z0 /= vcount;
        // compute centroid for second set
        double centroid_x1 = 0;
        double centroid_y1 = 0;
        double centroid_z1 = 0;
        for (int i = 1; i <= vcount; i++) {
            centroid_x1 += vlist3[neighboursIndices[i]][0];
            centroid_y1 += vlist3[neighboursIndices[i]][1];
            centroid_z1 += vlist3[neighboursIndices[i]][2];
        }
        centroid_x1 /= vcount;
        centroid_y1 /= vcount;
        centroid_z1 /= vcount;
        // translate first set by centroid
        for (int i = 1; i <= vcount; i++) {
            vlist[i][0] -= centroid_x0;
            vlist[i][1] -= centroid_y0;;
            vlist[i][2] -= centroid_z0;;
        }
        // translate second set by centroid
        for (int i = 1; i <= vcount3; i++) {
            vlist3[i][0] -= centroid_x1;
            vlist3[i][1] -= centroid_y1;;
            vlist3[i][2] -= centroid_z1;;
        }
        // compute RTS
        MatrixXd RTS = MatrixXd::Zero(4, 4);
        for (int i = 1; i <= vcount; i++) {
            MatrixXd RT(4, 4);
            double x = vlist[i][0];
            double y = vlist[i][1];
            double z = vlist[i][2];
            RT << 1, x, y, z,
                -x, 1, -z, y,
                -y, z, 1, -x,
                -z, -y, x, 1;
            MatrixXd S(4, 4);
            double x0 = vlist3[neighboursIndices[i]][0];
            double y0 = vlist3[neighboursIndices[i]][1];
            double z0 = vlist3[neighboursIndices[i]][2];
            S << 1, -x0, -y0, -z0,
                x0, 1, -z0, y0,
                y0, z0, 1, -x0,
                z0, -y0, x0, 1;
            RTS = RTS + RT * S;
        }
        // compute eigenvector with biggest eigenvalue
        EigenSolver<Matrix4d> eigensolver(RTS);
        double largest_eigenvalue = 0;
        double largest_eigenvalue_index = 0;
        for (int i = 0; i < 4; i++) {
            if (eigensolver.eigenvalues()[i].real() > largest_eigenvalue && eigensolver.eigenvalues()[i].imag() == 0) {
                largest_eigenvalue = eigensolver.eigenvalues()[i].real();
                largest_eigenvalue_index = i;
            }
        }
        VectorXcd largest_eigenvector = eigensolver.eigenvectors().col(largest_eigenvalue_index);
        cout << "The largest eigenvector of RTS is:" << endl << largest_eigenvector << endl;
        // get q_w, q_x, q_y, q_z
        double q_w = largest_eigenvector[0].real();
        double q_x = largest_eigenvector[1].real();
        double q_y = largest_eigenvector[2].real();
        double q_z = largest_eigenvector[3].real();
        // rotate all first set points by q
        MatrixXd R(3, 3);
        R << (q_w * q_w + q_x * q_x - q_y * q_y - q_z * q_z), (2 * (q_x * q_y - q_w * q_z)), (2 * (q_x * q_z + q_w * q_y)),
            (2 * (q_x * q_y + q_w * q_z)), (q_w * q_w - q_x * q_x + q_y * q_y - q_z * q_z), (2 * (q_y * q_z - q_w * q_x)),
            (2 * (q_x * q_z - q_w * q_y)), (2 * (q_y * q_z + q_w * q_x)), (q_w * q_w - q_x * q_x - q_y * q_y + q_z * q_z);
        for (int i = 1; i <= vcount; i++) {
            double x = vlist[i][0];
            double y = vlist[i][1];
            double z = vlist[i][2];
            VectorXd point(3);
            point << vlist[i][0], vlist[i][1], vlist[i][2];
            VectorXd rotatedPoint(3);
            rotatedPoint = R * point;
            vlist[i][0] = rotatedPoint[0];
            vlist[i][1] = rotatedPoint[1];
            vlist[i][2] = rotatedPoint[2];
        }
        cout << "Points in first set are rotated by: " << 2 * acos(q_w) * 180 / 3.14159 << " degree" << endl;
        max_iter--;
    }
    finalNeighbourIndices = neighboursIndices;
}

void myObjType::draw3(bool smooth) {
    glEnable(GL_LIGHTING);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    // make second object red
    float mat_ambient[] = { 0.1745f, 0.01175f, 0.01175f, 0.55f };
    float mat_diffuse[] = { 0.61424f, 0.04136f, 0.04136f, 0.55f };
    float mat_specular[] = { 0.727811f, 0.626959f, 0.626959f, 0.55f };
    float shininess = 76.8f;
    glPushMatrix();
    double longestSide = 0.0;
    for (int i = 0; i < 3; i++)
        if ((lmax[i] - lmin[i]) > longestSide)
            longestSide = (lmax[i] - lmin[i]);
    glScalef(4.0 / longestSide, 4.0 / longestSide, 4.0 / longestSide);
    glTranslated(-(lmin[0] + lmax[0]) / 2.0, -(lmin[1] + lmax[1]) / 2.0, -(lmin[2] + lmax[2]) / 2.0);
    for (int i = 1; i <= tcount3; i++) {
        if (smooth) {
            glBegin(GL_POLYGON);
            glNormal3dv(vnlist3[tlist3[i][0]]);
            glVertex3dv(vlist3[tlist3[i][0]]);
            glNormal3dv(vnlist3[tlist3[i][1]]);
            glVertex3dv(vlist3[tlist3[i][1]]);
            glNormal3dv(vnlist3[tlist3[i][2]]);
            glVertex3dv(vlist3[tlist3[i][2]]);
            glEnd();
        } else {
            glBegin(GL_POLYGON);
            glNormal3dv(nlist3[i]);
            for (int j = 0; j < 3; j++)
                glVertex3dv(vlist3[tlist3[i][j]]);
            glEnd();
        }
    }
    glDisable(GL_LIGHTING);
    drawInitialErrorLines();
    drawFinalErrorLines(finalNeighbourIndices);
    glPopMatrix();
}

void myObjType::computeNormals3() {
    // Lab 1 Subtask 1: Compute Normal Vectors
    for (int i = 1; i <= tcount3; i++) {
        double point_a[3];
        double point_b[3];
        double point_c[3];
        double v_0[3];
        double v_1[3];
        for (int j = 0; j < 3; j++) {
            point_a[j] = vlist3[tlist3[i][0]][j];
            point_b[j] = vlist3[tlist3[i][1]][j];
            point_c[j] = vlist3[tlist3[i][2]][j];
        }
        v_0[0] = point_b[0] - point_a[0];
        v_0[1] = point_b[1] - point_a[1];
        v_0[2] = point_b[2] - point_a[2];
        v_1[0] = point_c[0] - point_a[0];
        v_1[1] = point_c[1] - point_a[1];
        v_1[2] = point_c[2] - point_a[2];

        nlist3[i][0] = v_0[1] * v_1[2] - v_0[2] * v_1[1];
        nlist3[i][1] = -(v_0[0] * v_1[2] - v_0[2] * v_1[0]);
        nlist3[i][2] = v_0[0] * v_1[1] - v_0[1] * v_1[0];
    }
}

void myObjType::computeFnlist3() {
    for (int i = 1; i <= tcount3; i++) {
        int v0 = tlist3[i][0];
        int v1 = tlist3[i][1];
        int v2 = tlist3[i][2];
        // consider edge v0-v1
        if (edgemap3.find(make_pair(min(v0, v1), max(v0, v1))) != edgemap3.end()) {
            int neighbour = edgemap3.at(make_pair(min(v0, v1), max(v0, v1)));
            fnlist3[i][0] = neighbour << 3 | computeVersionFromVertices(neighbour, v0, v1);
            fnlist3[i][3] = neighbour << 3 | computeVersionFromVertices(neighbour, v1, v0);
            fnlist3[neighbour][computeVersionFromVertices(neighbour, v0, v1)] = i << 3 | 0;
            fnlist3[neighbour][computeVersionFromVertices(neighbour, v1, v0)] = i << 3 | 3;
        } else {
            edgemap3.insert(pair<pair<int, int>, int>(make_pair(min(v0, v1), max(v0, v1)), i));
        }
        // consider edge v1-v2
        if (edgemap3.find(make_pair(min(v1, v2), max(v1, v2))) != edgemap3.end()) {
            int neighbour = edgemap3.at(make_pair(min(v1, v2), max(v1, v2)));
            fnlist3[i][1] = neighbour << 3 | computeVersionFromVertices(neighbour, v1, v2);
            fnlist3[i][4] = neighbour << 3 | computeVersionFromVertices(neighbour, v2, v1);
            fnlist3[neighbour][computeVersionFromVertices(neighbour, v1, v2)] = i << 3 | 1;
            fnlist3[neighbour][computeVersionFromVertices(neighbour, v2, v1)] = i << 3 | 4;
        } else {
            edgemap3.insert(pair<pair<int, int>, int>(make_pair(min(v1, v2), max(v1, v2)), i));
        }
        // consider edge v2-v0
        if (edgemap3.find(make_pair(min(v2, v0), max(v2, v0))) != edgemap3.end()) {
            int neighbour = edgemap3.at(make_pair(min(v2, v0), max(v2, v0)));
            fnlist3[i][2] = neighbour << 3 | computeVersionFromVertices(neighbour, v2, v0);
            fnlist3[i][5] = neighbour << 3 | computeVersionFromVertices(neighbour, v0, v2);
            fnlist3[neighbour][computeVersionFromVertices(neighbour, v2, v0)] = i << 3 | 2;
            fnlist3[neighbour][computeVersionFromVertices(neighbour, v0, v2)] = i << 3 | 5;
        } else {
            edgemap3.insert(pair<pair<int, int>, int>(make_pair(min(v2, v0), max(v2, v0)), i));
        }
    }
}

void myObjType::computeVertexNormals3() {
    // Lab 2 Optional 3: Smooth Shading
    for (int i = 1; i <= vcount3; i++) {
        int neighbourTIdx = vmap3.at(i);
        int neighbourVersion;
        if (tlist3[neighbourTIdx][0] == i) neighbourVersion = 0;
        if (tlist3[neighbourTIdx][1] == i) neighbourVersion = 1;
        if (tlist3[neighbourTIdx][2] == i) neighbourVersion = 2;
        int numNeighbours = 0;
        double normalSumX = 0;
        double normalSumY = 0;
        double normalSumZ = 0;
        unordered_set<int> neighboursVisited;
        OrTri currTriangle = makeOrTri(neighbourTIdx, neighbourVersion);
        while (neighboursVisited.find(currTriangle) == neighboursVisited.end()) {
            // compute normal for currTriangle
            double point_a[3];
            double point_b[3];
            double point_c[3];
            double v_0[3];
            double v_1[3];
            for (int j = 0; j < 3; j++) {
                point_a[j] = vlist3[tlist3[idx(currTriangle)][0]][j];
                point_b[j] = vlist3[tlist3[idx(currTriangle)][1]][j];
                point_c[j] = vlist3[tlist3[idx(currTriangle)][2]][j];
            }
            v_0[0] = point_b[0] - point_a[0];
            v_0[1] = point_b[1] - point_a[1];
            v_0[2] = point_b[2] - point_a[2];
            v_1[0] = point_c[0] - point_a[0];
            v_1[1] = point_c[1] - point_a[1];
            v_1[2] = point_c[2] - point_a[2];

            // normalise
            double x = v_0[1] * v_1[2] - v_0[2] * v_1[1];
            double y = -(v_0[0] * v_1[2] - v_0[2] * v_1[0]);
            double z = v_0[0] * v_1[1] - v_0[1] * v_1[0];
            double magnitude = sqrt(x * x + y * y + z * z);
            normalSumX += x / magnitude;
            normalSumY += y / magnitude;
            normalSumZ += z / magnitude;

            neighboursVisited.insert({ currTriangle });
            currTriangle = enext(sym(fnext(currTriangle)));
        }
        vnlist3[i][0] = normalSumX;
        vnlist3[i][1] = normalSumY;
        vnlist3[i][2] = normalSumZ;
    }
}

void myObjType::drawFinalErrorLines(std::vector<int> neighboursIndices) {
    glColor3f(0, 1, 0);
    glLineWidth(4);
    if (neighboursIndices.size() == 0) {
        glLineWidth(1);
        return;
    }
    for (int i = 1; i <= vcount; i++) {
        if (!vlistBitset.test(i)) {
            glBegin(GL_LINES);
            glVertex3dv(vlist[i]);
            glVertex3dv(vlist3[neighboursIndices[i]]);
            glEnd();
        }
    }
    glLineWidth(1);
}

void myObjType::transformVertices() {
    MatrixXd R(3, 3);
    R << 1, 0, 0,
        0, sqrt(3) / 2, 0.5,
        0, -0.5, sqrt(3) / 2;
    for (int i = 1; i <= vcount; i++) {
        VectorXd point(3);
        point << vlist[i][0], vlist[i][1], vlist[i][2];
        VectorXd rotatedPoint(3);
        rotatedPoint = R * point;
        vlist[i][0] = rotatedPoint[0] + 3;
        vlist[i][1] = rotatedPoint[1] + 3;
        vlist[i][2] = rotatedPoint[2] + 3;
    }

}

void myObjType::drawInitialErrorLines() {
    if (hasDrawnErrorLines) {
        std::vector<int> neighbourIndices = getNearestDistanceIndicesFastSecondVersion();
        glColor3f(1, 0, 1);
        glLineWidth(1);
        for (int i = 1; i <= vcount; i++) {
            if (!vlistBitset.test(i)) {
                glBegin(GL_LINES);
                glVertex3dv(vlist[i]);
                glVertex3dv(vlist3[neighbourIndices[i]]);
                glEnd();
            }
        }
    }
}

void myObjType::toggleHasDrawnErrorLines() {
    hasDrawnErrorLines = !hasDrawnErrorLines;
}

void myObjType::drawErrorMap() {
    computeErrors();
    glDisable(GL_LIGHTING);
    glPushMatrix();
    double longestSide = 0.0;
    for (int i = 0; i < 3; i++)
        if ((lmax[i] - lmin[i]) > longestSide)
            longestSide = (lmax[i] - lmin[i]);
    glScalef(4.0 / longestSide, 4.0 / longestSide, 4.0 / longestSide);
    glTranslated(-(lmin[0] + lmax[0]) / 2.0, -(lmin[1] + lmax[1]) / 2.0, -(lmin[2] + lmax[2]) / 2.0);
    for (int i = 1; i <= tcount; i++) {
        if (!tlistBitset.test(i)) {
            glBegin(GL_POLYGON);
            glNormal3dv(nlist[i]);
            for (int j = 0; j < 3; j++) {
                setErrorColor(tlist[i][j]);
                glVertex3dv(vlist[tlist[i][j]]);
            }
            glEnd();
        }
    }
    glPopMatrix();
}

void myObjType::setErrorColor(int vIdx) {
    if (finalNeighbourErrors.at(vIdx) < maxError / 3) {
        glColor3f(0, 1, 0);
    } else if (finalNeighbourErrors.at(vIdx) < 2 * maxError / 3) {
        glColor3f(1, 0.55, 0);
    } else {
        glColor3f(1, 0, 0);
    }
}

void myObjType::computeErrors() {
    maxError = 0;
    finalNeighbourErrors.clear();
    finalNeighbourErrors.push_back(0);
    for (int i = 1; i <= vcount; i++) {
        double point_a[3] = { vlist[i][0], vlist[i][1], vlist[i][2] };
        double point_b[3] = { vlist3[finalNeighbourIndices[i]][0], vlist3[finalNeighbourIndices[i]][1], vlist3[finalNeighbourIndices[i]][2] };
        double dist = distance(point_a, point_b);
        finalNeighbourErrors.push_back(dist);
        if (dist > maxError) maxError = dist;
    }
}
