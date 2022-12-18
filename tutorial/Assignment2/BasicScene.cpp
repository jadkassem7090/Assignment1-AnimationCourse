#include "BasicScene.h"
#include <utility>
#include "ObjLoader.h"
#include "IglMeshLoader.h"


using namespace cg3d;
using namespace igl;

void BasicScene::Init(float fov, int width, int height, float near, float far)
{
    camera = Camera::Create("camera", fov, float(width) / float(height), near, far);
    program = std::make_shared<Program>("shaders/basicShader");
    auto material = std::make_shared<Material>("material", program);

    auto daylight{ std::make_shared<Material>("daylight", "shaders/cubemapShader") };
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{ Model::Create("background", Mesh::Cube(), daylight) };

    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();

    material->AddTexture(0, "textures/box0.bmp", 2);
    //Sphere
    auto mesh1{ IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj") };
    auto mesh2{ IglLoader::MeshFromFiles("cube_igl", "data/sphere.obj") };
    //Bunny
    /*auto mesh1{ IglLoader::MeshFromFiles("sphere_igl", "data/bunny.off") };
    auto mesh2{ IglLoader::MeshFromFiles("cube_igl", "data/bunny.off") };*/
    //Constructing the model
    obj1 = Model::Create("sphere", mesh1, material);
    obj2 = Model::Create("cube", mesh2, material);
    
    auto morphFunc = [](Model* model, cg3d::Visitor* visitor)
    {
        return model->meshIndex;
    };
    //Auto Objects
    autoObj1 = AutoMorphingModel::Create(*obj1, morphFunc);
    autoObj2 = AutoMorphingModel::Create(*obj2, morphFunc);
    //Camera
    camera->Translate({ -1,0,2 });
    //AutoObject 1
    autoObj1->showFaces = false;
    autoObj1->showWireframe = false;
    //autoObj1->Translate({ -1, 0, 0 });//for bunny
    autoObj1->Translate({ -8, 0, 0 });//for Sphere
    //autoObj1->Scale(15);
    AddChild(autoObj1);
    //AutoObject 2
    autoObj2->showFaces = false;
    autoObj2->showWireframe = false;
    //autoObj2->Translate({ 1, 0, 0 });//for bunny
    autoObj2->Translate({ 8, 0, 0 });//for Sphere
    //autoObj2->Scale(10);
    AddChild(autoObj2);
    //Initializing the trees
    Tree1.init(autoObj1->GetMesh()->data[0].vertices, autoObj1->GetMesh()->data[0].faces);
    Tree2.init(autoObj2->GetMesh()->data[0].vertices, autoObj2->GetMesh()->data[0].faces);
    //Setting Up Bounding Box
    makeBouningBox(Tree1.m_box, 0);
    makeBouningBox(Tree2.m_box, 1);
    boundingBox1->showFaces = false;
    boundingBox1->showWireframe = true;
    boundingBox2->showFaces = false;
    boundingBox2->showWireframe = true;
}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    autoObj1->Rotate(0.000f, Axis::XYZ);
    autoObj2->Rotate(0.000f, Axis::XYZ);

    if (tellMeCollapsed && CheckCollision(&Tree1, &Tree2)) {
            tellMeCollapsed = false;
            std::cout << "There is a collision BRO!" << std::endl;
    }
    else {
        if (tellMeCollapsed) {
            autoObj2->Translate(speed, Axis::X);
            autoObj1->Rotate(0.001f, Axis::XYZ);
        }
    }
}   


//A recursive function that go down to the leafs of the AABB tree and checking if there and intersection between two boxes.
bool BasicScene::CheckCollision(igl::AABB<Eigen::MatrixXd, 3>* tree1, igl::AABB<Eigen::MatrixXd, 3>* tree2) {
    if (tree1 == nullptr || tree2 == nullptr)
    {
        return false;
    }
    if (!isThereIntersection(tree1->m_box, tree2->m_box)) {
        return false;
    }
    if (!tree1->is_leaf() && !tree2->is_leaf()) {
        return CheckCollision(tree1->m_left, tree2->m_left) || CheckCollision(tree1->m_left, tree2->m_right)
            || CheckCollision(tree1->m_right, tree2->m_left) || CheckCollision(tree1->m_right, tree2->m_right);
    }
    else if (tree1->is_leaf() && !tree2->is_leaf()) {
        return CheckCollision(tree1, tree2->m_left) || CheckCollision(tree1, tree2->m_right);
    }
    else if (!tree1->is_leaf() && tree2->is_leaf()) {
        return CheckCollision(tree1->m_left, tree2) || CheckCollision(tree1->m_right, tree2);
    }
    makeBouningBox(tree1->m_box, 2);
    makeBouningBox(tree2->m_box, 3);
    hit1->showFaces = true;
    hit2->showFaces = true;
    hit1->showWireframe = false;
    hit2->showWireframe = false;
    return true;
}

//Checks the if two boxes intersect.
bool BasicScene::isThereIntersection(Eigen::AlignedBox<double, 3>& box1, Eigen::AlignedBox<double, 3>& box2) {
    //Getting the centers of the boxes.
    Eigen::Vector3d box1C = box1.center();
    Eigen::Vector3d box2C = box2.center();
    //Getting the size of the boxes.
    Eigen::Affine3f scaling1 = autoObj1->GetScaling(autoObj1->GetTransform());
    Eigen::Affine3f scaling2 = autoObj2->GetScaling(autoObj1->GetTransform());
    Eigen::Vector3d a = scaling1(0, 0)*(box1.sizes()/2);
    Eigen::Vector3d b = scaling2(0, 0)*(box2.sizes()/2);
    //Getting the rotation matrix for each object.
    Eigen::Matrix3d A = autoObj1->GetRotation().cast<double>();
    Eigen::Matrix3d B = autoObj2->GetRotation().cast<double>();
    Eigen::Matrix3d C = A.transpose() * B;
    //Calculating D matrix.
    Eigen::Vector4d C1 = Eigen::Vector4d(box1C(0), box1C(1), box1C(2),1);
    Eigen::Vector4d C2 = Eigen::Vector4d(box2C(0), box2C(1), box2C(2),1);
    Eigen::Vector4d D4 = autoObj2->GetTransform().cast<double>() * C2 - (autoObj1->GetTransform().cast<double>() * C1);
    Eigen::Vector3d D = Eigen::Vector3d(D4(0), D4(1), D4(2));
    
    //Checking the intersection of two boxes relativly with the tranformation and the rotation that occurred on the object.
    //Variables from the table(obb_sat.pdf).
    float R,R0, R1;
   
    //First Condition A0
    R0 = a(0);
    R1 = b(0)*abs(C(0,0)) + b(1) * abs(C(0, 1)) + b(2) * abs(C(0, 2));
    R = abs(A.col(0).transpose() * D);
    if (R > R0 + R1) return false;
    //Second Condition A1
    R0 = a(1);
    R1 = b(0) * abs(C(1, 0)) + b(1) * abs(C(1, 1)) + b(2) * abs(C(1, 2));
    R = abs(A.col(1).transpose() * D);
    if (R > R0 + R1) return false;
    //third Condition A2
    R0 = a(2);
    R1 = b(0) * abs(C(2, 0)) + b(1) * abs(C(2, 1)) + b(2) * abs(C(2, 2));
    R = abs(A.col(2).transpose() * D);
    if (R > R0 + R1) return false;
    //4 Condition B0
    R1 = b(0);
    R0 = a(0)*abs(C(0,0)) + a(1) * abs(C(1, 0)) + a(2) * abs(C(2, 0));
    R = abs(B.col(0).transpose() * D);
    if (R > R0 + R1) return false;
    //5 Condition B1
    R1 = b(1);
    R0 = a(0) * abs(C(0, 1)) + a(1) * abs(C(1, 1)) + a(2) * abs(C(2, 1));
    R = abs(B.col(1).transpose() * D);
    if (R > R0 + R1) return false;
    //6 Condition B2
    R1 = b(2);
    R0 = a(0) * abs(C(0, 2)) + a(1) * abs(C(1, 2)) + a(2) * abs(C(2, 2));
    R = abs(B.col(2).transpose() * D);
    if (R > R0 + R1) return false;
    //7 condition A0*B0
    R0 = a(1) * abs(C(2, 0)) + a(2) * abs(C(1, 0));
    R1 = b(1) * abs(C(0, 2)) + b(2) * abs(C(0, 1));
    R = C(1, 0) * A.col(2).transpose() * D;
    R = abs(R - (C(2, 0) * A.col(1).transpose() * D));
    if (R > R0 + R1) return false;
    //8 condition A0*B1
    R0 = a(1) * abs(C(2, 1)) + a(2) * abs(C(1, 1));
    R1 = b(0) * abs(C(0, 2)) + b(2) * abs(C(0, 0));
    R = C(1, 1) * A.col(2).transpose() * D;
    R = abs(R - (C(2, 1) * A.col(1).transpose() * D));
    if (R > R0 + R1) return false;
    //9 condition A0*B2
    R0 = a(1) * abs(C(2, 2)) + a(2) * abs(C(1, 2));
    R1 = b(0) * abs(C(0, 1)) + b(1) * abs(C(0, 0));
    R = C(1, 2) * A.col(2).transpose() * D;
    R = abs(R - (C(2, 2) * A.col(1).transpose() * D));
    if (R > R0 + R1) return false;
    //10 condition A1*B0
    R0 = a(0) * abs(C(2, 0)) + a(2) * abs(C(0, 0));
    R1 = b(1) * abs(C(1, 2)) + b(2) * abs(C(1, 1));
    R = C(2, 0) * A.col(0).transpose() * D;
    R = abs(R - (C(0, 0) * A.col(2).transpose() * D));
    if (R > R0 + R1) return false;
    //11 condition A1*B1
    R0 = a(0) * abs(C(2, 1)) + a(2) * abs(C(0, 1));
    R1 = b(0) * abs(C(1, 2)) + b(2) * abs(C(1, 0));
    R = C(2, 1) * A.col(0).transpose() * D;
    R = abs(R - (C(0, 1) * A.col(2).transpose() * D));
    if (R > R0 + R1) return false;
    //12 condition A1*B2
    R0 = a(0) * abs(C(2, 2)) + a(2) * abs(C(0, 2));
    R1 = b(0) * abs(C(1, 1)) + b(1) * abs(C(1, 2));
    R = C(2, 2) * A.col(0).transpose() * D;
    R = abs(R - (C(0, 2) * A.col(2).transpose() * D));
    if (R > R0 + R1) return false;
    //13 condition A2*B0
    R0 = a(0) * abs(C(1, 0)) + a(1) * abs(C(0, 0));
    R1 = b(1) * abs(C(2, 2)) + b(2) * abs(C(2, 1));
    R = C(0, 0) * A.col(1).transpose() * D;
    R = abs(R - (C(1, 0) * A.col(0).transpose() * D));
    if (R > R0 + R1) return false;
    //14 condition A2*B1
    R0 = a(0) * abs(C(1, 1)) + a(1) * abs(C(0, 1));
    R1 = b(0) * abs(C(2, 2)) + b(2) * abs(C(2, 0));
    R = C(0, 1) * A.col(1).transpose() * D;
    R = abs(R - (C(1, 1) * A.col(0).transpose() * D));
    if (R > R0 + R1) return false;
    //15 condition A2*B2
    R0 = a(0) * abs(C(1, 2)) + a(1) * abs(C(0, 2));
    R1 = b(0) * abs(C(2, 1)) + b(1) * abs(C(2, 0));
    R = C(0, 2) * A.col(1).transpose() * D;
    R = abs(R - (C(1, 2) * A.col(0).transpose() * D));
    if (R > R0 + R1) return false;

    return true;
}

//assigns to boundingBox1/2 or hit1/2 it's correct mesh.
void BasicScene::makeBouningBox(Eigen::AlignedBox<double, 3>& aligned_box, int num)
{
    Eigen::MatrixXd V, VN, T;
    Eigen::MatrixXi F;

    //Vertices
    V.resize(8, 3);
    V.row(0) = aligned_box.corner(aligned_box.BottomLeftCeil);
    V.row(1) = aligned_box.corner(aligned_box.TopLeftCeil);
    V.row(2) = aligned_box.corner(aligned_box.TopRightCeil);
    V.row(3) = aligned_box.corner(aligned_box.TopRightFloor);
    V.row(4) = aligned_box.corner(aligned_box.BottomRightCeil);
    V.row(5) = aligned_box.corner(aligned_box.BottomRightFloor);
    V.row(6) = aligned_box.corner(aligned_box.BottomLeftFloor);
    V.row(7) = aligned_box.corner(aligned_box.TopLeftFloor);

    //Faces
    F.resize(12, 3);
    F.row(0) = Eigen::Vector3i(1,4,2);
    F.row(1) = Eigen::Vector3i(1,0,4);
    F.row(2) = Eigen::Vector3i(0,1,6);
    F.row(3) = Eigen::Vector3i(1,7,6);
    F.row(4) = Eigen::Vector3i(1,2,7);
    F.row(5) = Eigen::Vector3i(2,3,7);
    F.row(6) = Eigen::Vector3i(2,4,5);
    F.row(7) = Eigen::Vector3i(2,3,5);
    F.row(8) = Eigen::Vector3i(7,6,5);
    F.row(9) = Eigen::Vector3i(7,3,5);
    F.row(10) = Eigen::Vector3i(0,4,6);
    F.row(11) = Eigen::Vector3i(6,4,5);

    igl::per_vertex_normals(V, F, VN);
    T = Eigen::MatrixXd::Zero(V.rows(), 2);

    std::shared_ptr<Mesh> newM = std::make_shared<Mesh>("new",V,F,VN,T);
    auto EmptyMat{ std::make_shared<Material>("material", program) };

    if (num == 0) {
        boundingBox1 = Model::Create("boundingBox1", newM, EmptyMat);
        autoObj1->AddChild(boundingBox1);
    }
    else if (num == 1) {
        boundingBox2 = Model::Create("boundingBox2", newM, EmptyMat);
        autoObj2->AddChild(boundingBox2);
    }
    else if (num == 2) {
        hit1 = Model::Create("Hit1", newM, EmptyMat);
        autoObj1->AddChild(hit1);
    }else{
        hit2 = Model::Create("Hit2", newM, EmptyMat);
        autoObj2->AddChild(hit2);
    }
}

void BasicScene::KeyCallback(Viewport* viewport, int x, int y, int key, int scancode, int action, int mods)
{
    auto system = camera->GetRotation().transpose();

    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key) // NOLINT(hicpp-multiway-paths-covered)
        {
        case GLFW_KEY_ESCAPE:
            glfwSetWindowShouldClose(window, GLFW_TRUE);
            break;
        case GLFW_KEY_UP:
            if (pickedModel == NULL)
                camera->RotateInSystem(system, 0.1f, Axis::X);
            else {
                pickedModel->meshIndex++;
                pickedModel->meshIndex = pickedModel->meshIndex % pickedModel->GetMesh()->data.size();
            }
            break;
        case GLFW_KEY_DOWN:
            if (pickedModel == NULL)
                camera->RotateInSystem(system, -0.1f, Axis::X);
            else {
                if (pickedModel->meshIndex == 0)
                    pickedModel->meshIndex = pickedModel->GetMesh()->data.size() - 1;
                else
                    pickedModel->meshIndex--;
            }            break;
        case GLFW_KEY_LEFT:
            speed -= 0.01;
            break;
        case GLFW_KEY_RIGHT:
            speed += 0.01;
            break;
        case GLFW_KEY_W:
            camera->TranslateInSystem(system, { 0, 0.05f, 0 });
            break;
        case GLFW_KEY_S:
            camera->TranslateInSystem(system, { 0, -0.05f, 0 });
            break;
        case GLFW_KEY_A:
            camera->TranslateInSystem(system, { -0.05f, 0, 0 });
            break;
        case GLFW_KEY_D:
            camera->TranslateInSystem(system, { 0.05f, 0, 0 });
            break;
        case GLFW_KEY_B:
            camera->TranslateInSystem(system, { 0, 0, 0.05f });
            break;
        case GLFW_KEY_F:
            camera->TranslateInSystem(system, { 0, 0, -0.05f });
            break;
        case GLFW_KEY_SPACE:
            if (pickedModel != NULL) {
                if (collapse(pickedModel->GetMesh())) {
                    pickedModel->meshIndex = pickedModel->GetMesh()->data.size() - 1;
                }
            }
            else
                cout << "Your tried to simplify nothing!" << endl;
            break;
        case  GLFW_KEY_R:
            pickedModel->meshIndex = 0;
            break;
        }
    }
}

//for simplification.
bool BasicScene::collapse(std::shared_ptr<cg3d::Mesh> mesh) {
    const int max_iter = std::ceil(0.1 * mesh->dataS.myQ.size());
    bool collapsed = mesh->dataS.newsimplify(max_iter);
    if (collapsed) {
        Eigen::MatrixXd VN;
        igl::per_vertex_normals(mesh->dataS.V, mesh->dataS.F, VN);
        Eigen::MatrixXd TC = Eigen::MatrixXd::Zero(mesh->dataS.V.rows(), 2);
        mesh->data.push_back(MeshData{ mesh->dataS.V, mesh->dataS.F, VN, TC });
        pickedModel->SetMeshList({ pickedModel->GetMesh() });
    }
    return collapsed;
}
