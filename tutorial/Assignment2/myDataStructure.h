#pragma once
#pragma once
#include <igl/circulation.h>
#include <igl/collapse_edge.h>
#include <igl/edge_flaps.h>
#include <igl/decimate.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/parallel_for.h>
#include <igl/read_triangle_mesh.h>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/per_face_normals.h>
#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/Core>
#include <iostream>
#include <set>
using namespace std;
using namespace igl;
using namespace Eigen;
class myDataStructure {
public:
    myDataStructure() = default;
    myDataStructure(const Eigen::MatrixXi& F, const Eigen::MatrixXd& V);
    bool newsimplify(int faces);

    void calculateMatrix();
    bool myCollapseEdge();
    void fillQueue();
    void calculateEdgeCost(int e);

    VectorXi EMAP;
    MatrixXi F, E, EF, EI;
    MatrixXd C, V, FN;
    Eigen::VectorXi EQ;

    std::set<std::pair<double, int>> myQ;
    std::vector<std::set<std::pair<double, int>>::iterator> Qiterators;

    vector<Matrix4d> QMatrices;
};