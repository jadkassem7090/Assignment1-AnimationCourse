#include "myDataStructure.h"

using namespace std;
//From 8 - 11
myDataStructure::myDataStructure(const Eigen::MatrixXi& F, const Eigen::MatrixXd& V) : F(F), V(V) {
    edge_flaps(F, this->E, this->EMAP, this->EF, this->EI);
    ////Here we want to fill the priority queue
    myQ.clear();
    Qiterators.resize(E.rows());
    QMatrices.resize(V.rows());
    calculateMatrix();
    //Here we initailize the C matrix to have the number of edges as rows and 3 columns 
    //                              //The allows us to give each edge the vertex position that we will replace the edge with.
    fillQueue();
    EQ = Eigen::VectorXi::Zero(E.rows());
}
void myDataStructure::calculateEdgeCost(int e) {
    int ev1 = E(e, 0);
    int ev2 = E(e, 1);
    Vector4d myV;
    double cost;
    Matrix4d Qt = QMatrices[ev1] + QMatrices[ev2];
    Matrix4d invQ = Qt;
    invQ.row(3) = Vector4d(0, 0, 0, 1);
    if (invQ.determinant() == 0) {//So the matrix in not invertible 
        //Here I'm choosing the vertex of v1, v2, (v1+v2)/2
        Vector4d v1(V.row(ev1)[0], V.row(ev1)[1], V.row(ev1)[2], 1);
        Vector4d v2(V.row(ev2)[0], V.row(ev2)[1], V.row(ev2)[2], 1);
        Vector4d v3 = (v1 + v2) / 2;
        v3[3] = 1;
        double cost1 = v1.transpose() * Qt * v1;
        double cost2 = v2.transpose() * Qt * v2;
        double cost3 = v3.transpose() * Qt * v3;
        if (cost1 < cost2 && cost1 < cost3) {
            myV = v1;
            cost = cost1;
        }
        else if (cost1 > cost2 && cost2 < cost3) {
            myV = v2;
            cost = cost2;
        }
        else {
            myV = v3;
            cost = cost3;
        }
    }
    else {//here the matrix is invertabile so we can find the minimum error.
        myV = invQ.inverse() * Vector4d(0, 0, 0, 1);
        cost = myV.transpose() * Qt * myV;
    }
    Qiterators[e] = myQ.insert(pair<double, int>(cost, e)).first;
    C.row(e) = Vector3d(myV[0], myV[1], myV[2]);
}
void myDataStructure::fillQueue() {
    C.resize(E.rows(), V.cols());
    Eigen::VectorXd costs(E.rows());

    for (int i = 0; i < E.rows(); i++) {
        calculateEdgeCost(i);
    }
}



/*
    |aa ab ac ad|
Kp= |ab bb bc bd|
    |ac bc cc cd|
    |ad bd cd dd|
*/
void myDataStructure::calculateMatrix() {//Here we want to calculate for each vertex it's cost matrix.
    vector<vector<int>> VF, VI;
    vertex_triangle_adjacency(V.rows(), F, VF, VI);
    per_face_normals(V, F, FN);
    for (int i = 0; i < V.rows(); i++) {//here we are going on the vertecies to calculate for each one the Q
        Matrix4d x = Matrix4d::Zero();
        for (int j = 0; j < VF[i].size(); j++) {//here we want to calculate the Kp of each face for the current vertex
            Matrix4d Kp;
            Vector3d norm = FN.row(VF[i][j]).normalized();
            double a = norm[0];
            double b = norm[1];
            double c = norm[2];
            double d = V.row(i) * norm;
            d = -d;
            Kp.row(0) = Vector4d(a * a, a * b, a * c, a * d);
            Kp.row(1) = Vector4d(a * b, b * b, b * c, b * d);
            Kp.row(2) = Vector4d(a * c, b * c, c * c, c * d);
            Kp.row(3) = Vector4d(a * d, d * b, d * c, d * d);
            x += Kp;
        }
        QMatrices[i] = x;
    }
}

bool myDataStructure::myCollapseEdge() {
    int e1, e2, f1, f2; // for the collapse edge functions.
    std::pair<double, int> pair = *(myQ.begin());
    if (pair.first == std::numeric_limits<double>::infinity())
    {
        return false;
    }
    myQ.erase(myQ.begin());
    int e = pair.second;

    int v1 = E.row(e)[0];
    int v2 = E.row(e)[1];
    Qiterators[e] = myQ.end();//sending the edge iterator to the end because we want to delete it.
    // get the neighbors of the edge so we can update the cost of them after the collapse.
    std::vector<int> N = igl::circulation(e, true, EMAP, EF, EI);
    std::vector<int> Nd = igl::circulation(e, false, EMAP, EF, EI);
    N.insert(N.begin(), Nd.begin(), Nd.end());

    Eigen::VectorXd newPosition = C.row(e);
    bool is_collapsed = igl::collapse_edge(e, C.row(e), V, F, E, EMAP, EF, EI, e1, e2, f1, f2);
    if (is_collapsed)
    {
        myQ.erase(Qiterators[e1]);
        Qiterators[e1] = myQ.end();
        myQ.erase(Qiterators[e2]);
        Qiterators[e2] = myQ.end();
        QMatrices[v1] = QMatrices[v1] + QMatrices[v2];
        QMatrices[v2] = QMatrices[v1];
        //here we looping over the neihbors to update the changed edges.
        for (int n : N)
        {
            if (F(n, 0) != IGL_COLLAPSE_EDGE_NULL ||
                F(n, 1) != IGL_COLLAPSE_EDGE_NULL ||
                F(n, 2) != IGL_COLLAPSE_EDGE_NULL)
            {
                for (int v = 0; v < 3; v++)
                {

                    const int ei = EMAP(v * F.rows() + n);//here we go over the face's edges and delete them from the Q
                                                          //we go like that because EMAP maps for every face to the edges
                                                          //EMAP = {first edge for every face, second edge for every face ,
                                                            // third edge for every face }
                    myQ.erase(Qiterators[ei]);
                    calculateEdgeCost(ei);
                }
            }
        }
        cout << "Edge: " << e << ", Cost: " << pair.first
            << ", New Position: (" << newPosition[0] << "," << newPosition[1] << "," << newPosition[2] << ")" << std::endl;
    }
    else {
        pair.first = std::numeric_limits<double>::infinity();
        Qiterators[e] = myQ.insert(pair).first;
    }
    return is_collapsed;
}

bool myDataStructure::newsimplify(int faces) {
    bool something_collapsed = false;
    for (int i = 0; i < faces; i++) {
        if (myQ.empty())
            break;
        if (!myCollapseEdge())
            break;
        something_collapsed = true;
    }
    return something_collapsed;
}