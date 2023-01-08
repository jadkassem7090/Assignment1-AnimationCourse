#include "BasicScene.h"
#include <Eigen/src/Core/Matrix.h>
#include <edges.h>
#include <memory>
#include <per_face_normals.h>
#include <read_triangle_mesh.h>
#include <utility>
#include <vector>
#include "GLFW/glfw3.h"
#include "Mesh.h"
#include "PickVisitor.h"
#include "Renderer.h"
#include "ObjLoader.h"
#include "IglMeshLoader.h"

#include "igl/per_vertex_normals.h"
#include "igl/per_face_normals.h"
#include "igl/unproject_onto_mesh.h"
#include "igl/edge_flaps.h"
#include "igl/loop.h"
#include "igl/upsample.h"
#include "igl/AABB.h"
#include "igl/parallel_for.h"
#include "igl/shortest_edge_and_midpoint.h"
#include "igl/circulation.h"
#include "igl/edge_midpoints.h"
#include "igl/collapse_edge.h"
#include "igl/edge_collapse_is_valid.h"
#include "igl/write_triangle_mesh.h"

// #include "AutoMorphingModel.h"

using namespace cg3d;

void BasicScene::Init(float fov, int width, int height, float near, float far)
{
    camera = Camera::Create("camera", fov, float(width) / height, near, far);

    AddChild(root = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    auto daylight{ std::make_shared<Material>("daylight", "shaders/cubemapShader") };
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{ Model::Create("background", Mesh::Cube(), daylight) };
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();


    auto program = std::make_shared<Program>("shaders/phongShader");
    auto program1 = std::make_shared<Program>("shaders/pickingShader");

    auto material{ std::make_shared<Material>("material", program) }; // empty material
    auto material1{ std::make_shared<Material>("material", program1) }; // empty material
//    SetNamedObject(cube, Model::Create, Mesh::Cube(), material, shared_from_this());

    material->AddTexture(0, "textures/box0.bmp", 2);
    auto sphereMesh{ IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj") };
    auto cylMesh{ IglLoader::MeshFromFiles("cyl_igl","data/ycylinder.obj") };
    auto cubeMesh{ IglLoader::MeshFromFiles("cube_igl","data/cube_old.obj") };
    sphere1 = Model::Create("sphere", sphereMesh, material);

    //Axis
    Eigen::MatrixXd vertices(6, 3);
    vertices << -1, 0, 0, 1, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, -1, 0, 0, 1;
    Eigen::MatrixXi faces(3, 2);
    faces << 0, 1, 2, 3, 4, 5;
    Eigen::MatrixXd vertexNormals = Eigen::MatrixXd::Ones(6, 3);
    Eigen::MatrixXd textureCoords = Eigen::MatrixXd::Ones(6, 2);
    std::shared_ptr<Mesh> coordsys = std::make_shared<Mesh>("coordsys", vertices, faces, vertexNormals, textureCoords);
    axis.push_back(Model::Create("axis", coordsys, material1));
    axis[0]->mode = 1;
    axis[0]->Scale(4, Axis::XYZ);
    // axis[0]->lineWidth = 5;
    root->AddChild(axis[0]);
    float scaleFactor = 1;
    cyls.push_back(Model::Create("cyl", cylMesh, material));
    cyls[0]->Scale(scaleFactor, Axis::Y);
    cyls[0]->SetCenter(Eigen::Vector3f(0, -0.8f * scaleFactor, 0));
    root->AddChild(cyls[0]);

    for (int i = 1; i < 15; i++)
    {
        cyls.push_back(Model::Create("cyl", cylMesh, material));
        cyls[i]->Scale(scaleFactor, Axis::Y);
        cyls[i]->Translate(1.6f * scaleFactor, Axis::Y);
        cyls[i]->SetCenter(Eigen::Vector3f(0, -0.8f * scaleFactor,0));
        cyls[i - 1]->AddChild(cyls[i]);


        axis.push_back(Model::Create("axis", coordsys, material1));
        axis[i]->mode = 1;
        axis[i]->Scale(4, Axis::XYZ);
        cyls[i - 1]->AddChild(axis[i]);
        axis[i]->Translate(0.8*scaleFactor, Axis::Y);
    }
    cyls[0]->Translate({ 0,0.8f * scaleFactor,0 });
    root->RotateByDegree(0, Eigen::Vector3f(1, 0, 0));

    auto morphFunc = [](Model* model, cg3d::Visitor* visitor) {
        return model->meshIndex;//(model->GetMeshList())[0]->data.size()-1;
    };
    
    sphere1->showWireframe = true;
    sphere1->Translate({4,0,0});
    camera->Translate(22, Axis::Z);
    root->AddChild(sphere1);
}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 0.8f, 0.3f, 0.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 0.3f, 0.6f, 1.0f);
    program.SetUniform4f("Kdi", 0.5f, 0.5f, 0.0f, 1.0f);
    program.SetUniform1f("specular_exponent", 5.0f);
    program.SetUniform4f("light_position", 0.0, 15.0f, 0.0, 1.0f);
    //    cyl->Rotate(0.001f, Axis::Y);
    //autoCube->Rotate(0.001f, Axis::Z);
    /*if (runCCD)
        CCD();*/
}

void BasicScene::MouseCallback(Viewport* viewport, int x, int y, int button, int action, int mods, int buttonState[])
{
    // note: there's a (small) chance the button state here precedes the mouse press/release event

    if (action == GLFW_PRESS) { // default mouse button press behavior
        PickVisitor visitor;
        visitor.Init();
        renderer->RenderViewportAtPos(x, y, &visitor); // pick using fixed colors hack
        auto modelAndDepth = visitor.PickAtPos(x, renderer->GetWindowHeight() - y);
        renderer->RenderViewportAtPos(x, y); // draw again to avoid flickering
        pickedModel = modelAndDepth.first ? std::dynamic_pointer_cast<Model>(modelAndDepth.first->shared_from_this()) : nullptr;
        pickedModelDepth = modelAndDepth.second;
        camera->GetRotation().transpose();
        xAtPress = x;
        yAtPress = y;

        // if (pickedModel)
        //     debug("found ", pickedModel->isPickable ? "pickable" : "non-pickable", " model at pos ", x, ", ", y, ": ",
        //           pickedModel->name, ", depth: ", pickedModelDepth);
        // else
        //     debug("found nothing at pos ", x, ", ", y);

        if (pickedModel && !pickedModel->isPickable)
            pickedModel = nullptr; // for non-pickable models we need only pickedModelDepth for mouse movement calculations later

        if (pickedModel)
            pickedToutAtPress = pickedModel->GetTout();
        else
            cameraToutAtPress = camera->GetTout();
    }
}

void BasicScene::ScrollCallback(Viewport* viewport, int x, int y, int xoffset, int yoffset, bool dragging, int buttonState[])
{
    // note: there's a (small) chance the button state here precedes the mouse press/release event
    auto system = camera->GetRotation().transpose();
    if (pickedModel) {
        int size = cyls.size();
        bool is = false;
        for (int i = 0; i < size && !is; i++) {
            if (pickedModel == cyls[i]) {
                Eigen::Vector3f cam = camera->GetTranslation();
                Eigen::Vector3f oj = cyls[0]->GetTranslation();
                Eigen::Vector3f xx = (cam - oj).normalized();
                cyls[0]->Translate(root->GetRotation().inverse() *(xx * -float(yoffset)));
                is = true;
            }
        }
        if (!is) {
            Eigen::Vector3f cam = camera->GetTranslation();
            Eigen::Vector3f oj = pickedModel->GetTranslation();
            Eigen::Vector3f xx = (cam - oj).normalized();
            pickedModel->Translate(root->GetRotation().inverse() * (xx * -float(yoffset)));
        }
    }
    else {
        root->TranslateInSystem(system, { 0, 0,-float(yoffset) });
        cameraToutAtPress = root->GetTout();
    }
}

void BasicScene::CursorPosCallback(Viewport* viewport, int x, int y, bool dragging, int* buttonState)
{
    if (dragging) {
        auto system = camera->GetRotation().transpose() * GetRotation();
        auto moveCoeff = camera->CalcMoveCoeff(pickedModelDepth, viewport->width);
        auto angleCoeff = camera->CalcAngleCoeff(viewport->width);
        if (pickedModel) {
            //pickedModel->SetTout(pickedToutAtPress);
            if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE) {
                //Here I translate the pickeck objects according to the root rotation because it's more easier to translate.
                bool is = false;
                int size = cyls.size();
                for (int i = 0; i < size; i++) {
                    if (pickedModel == cyls[i]) {
                        cyls[0]->TranslateInSystem(system * root->GetRotation(), { -float(xAtPress - x) / moveCoeff, float(yAtPress - y) / moveCoeff, 0 });
                        is = true;
                    }
                }
                if(!is)
                    pickedModel->TranslateInSystem(system * root->GetRotation(), { -float(xAtPress - x) / moveCoeff, float(yAtPress - y) / moveCoeff, 0 });

            }if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
                pickedModel->RotateInSystem(system, float(xAtPress - x) / angleCoeff, Axis::Z);
            if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE) {
                //Here To make the rotation like pressing on the arrows.
                Eigen::Vector3f angles = pickedModel->GetRotation().eulerAngles(2,0,2);
                Eigen::Matrix3f phiMat;
                phiMat.row(0) = Eigen::Vector3f({ cos(angles[0]),-sin(angles[0]),0 });
                phiMat.row(1) = Eigen::Vector3f({ sin(angles[0]),cos(angles[0]), 0 });
                phiMat.row(2) = Eigen::Vector3f({ 0,0, 1 });
                Eigen::Matrix3f thetaMat;
                thetaMat.row(0) = Eigen::Vector3f({ 1, 0, 0 });
                thetaMat.row(1) = Eigen::Vector3f({ 0, cos(angles[1]),-sin(angles[1]) });
                thetaMat.row(2) = Eigen::Vector3f({ 0, sin(angles[1]),cos(angles[1]) });
                Eigen::Matrix3f psiMat;
                psiMat.row(0) = Eigen::Vector3f({ cos(angles[2]),-sin(angles[2]),0 });
                psiMat.row(1) = Eigen::Vector3f({ sin(angles[2]),cos(angles[2]), 0 });
                psiMat.row(2) = Eigen::Vector3f({ 0, 0, 1 });
                Eigen::Matrix3f rotationX;
                rotationX << 1, 0, 0,
                    0, cos(float(yAtPress - y) / angleCoeff), -sin(float(yAtPress - y) / angleCoeff),
                    0, sin(float(yAtPress - y) / angleCoeff), cos(float(yAtPress - y) / angleCoeff);
                Eigen::Matrix3f rotationY;
                rotationY << cos(float(xAtPress - x) / angleCoeff), -sin(float(xAtPress - x) / angleCoeff), 0,
                    sin(float(xAtPress - x) / angleCoeff), cos(float(xAtPress - x) / angleCoeff), 0,
                    0, 0, 1;
                pickedModel->Rotate(pickedModel->GetRotation().inverse() * phiMat * thetaMat * psiMat * rotationX * rotationY);
               // pickedModel->RotateInSystem(system, float(xAtPress - x) / angleCoeff, Axis::Y);
                //pickedModel->RotateInSystem(system, float(yAtPress - y) / angleCoeff, Axis::X);
            }
        }
        else {
            // camera->SetTout(cameraToutAtPress);
            if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE)
                root->TranslateInSystem(system, { -float(xAtPress - x) / moveCoeff / 10.0f, float(yAtPress - y) / moveCoeff / 10.0f, 0 });
            if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
                root->RotateInSystem(system, float(x - xAtPress) / 180.0f, Axis::Z);
            if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE) {
                //Here To make the rotation like pressing on the arrows.
                root->RotateInSystem(system, float(x - xAtPress) / angleCoeff, Axis::Y);
                root->RotateInSystem(system, float(y - yAtPress) / angleCoeff, Axis::X);
            }
        }
        xAtPress = x;
        yAtPress = y;
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
            if (pickedModel) {
                int size = cyls.size();
                for (int i = 0; i < size; i++) {
                    if (pickedModel == cyls[i]) {
                        Eigen::Vector3f angles = pickedModel->GetRotation().eulerAngles(2, 0, 2);
                        Eigen::Matrix3f phiMat;
                        phiMat.row(0) = Eigen::Vector3f({ cos(angles[0]),-sin(angles[0]),0 });
                        phiMat.row(1) = Eigen::Vector3f({ sin(angles[0]),cos(angles[0]), 0 });
                        phiMat.row(2) = Eigen::Vector3f({ 0,0, 1 });
                        Eigen::Matrix3f thetaMat;
                        thetaMat.row(0) = Eigen::Vector3f({ 1, 0, 0 });
                        thetaMat.row(1) = Eigen::Vector3f({ 0, cos(angles[1]),-sin(angles[1]) });
                        thetaMat.row(2) = Eigen::Vector3f({ 0, sin(angles[1]),cos(angles[1]) });
                        Eigen::Matrix3f psiMat;
                        psiMat.row(0) = Eigen::Vector3f({ cos(angles[2]),-sin(angles[2]),0 });
                        psiMat.row(1) = Eigen::Vector3f({ sin(angles[2]),cos(angles[2]), 0 });
                        psiMat.row(2) = Eigen::Vector3f({ 0, 0, 1 });
                        Eigen::Matrix3f rotation;
                        rotation << 1,0,0,
                                    0,cos(0.1f), -sin(0.1f),
                                    0, sin(0.1f), cos(0.1f);
                        pickedModel->Rotate(pickedModel->GetRotation().inverse() * phiMat * thetaMat * psiMat * rotation);
                    }
                }
            }
            else {
                root->RotateInSystem(system, 0.1f, Axis::X);
            }
            break;
        case GLFW_KEY_DOWN:
            if (pickedModel) {
                int size = cyls.size();
                for (int i = 0; i < size; i++) {
                    if (pickedModel == cyls[i]) {
                        Eigen::Vector3f angles = pickedModel->GetRotation().eulerAngles(2, 0, 2);
                        Eigen::Matrix3f phiMat;
                        phiMat.row(0) = Eigen::Vector3f({ cos(angles[0]),-sin(angles[0]),0 });
                        phiMat.row(1) = Eigen::Vector3f({ sin(angles[0]),cos(angles[0]), 0 });
                        phiMat.row(2) = Eigen::Vector3f({ 0,0, 1 });
                        Eigen::Matrix3f thetaMat;
                        thetaMat.row(0) = Eigen::Vector3f({ 1, 0, 0 });
                        thetaMat.row(1) = Eigen::Vector3f({ 0, cos(angles[1]),-sin(angles[1]) });
                        thetaMat.row(2) = Eigen::Vector3f({ 0, sin(angles[1]),cos(angles[1]) });
                        Eigen::Matrix3f psiMat;
                        psiMat.row(0) = Eigen::Vector3f({ cos(angles[2]),-sin(angles[2]),0 });
                        psiMat.row(1) = Eigen::Vector3f({ sin(angles[2]),cos(angles[2]), 0 });
                        psiMat.row(2) = Eigen::Vector3f({ 0, 0, 1 });
                        Eigen::Matrix3f rotation;
                        rotation << 1, 0, 0,
                            0, cos(-0.1f), -sin(-0.1f),
                            0, sin(-0.1f), cos(-0.1f);
                        pickedModel->Rotate(pickedModel->GetRotation().inverse() * phiMat * thetaMat * psiMat * rotation);
                    }
                }
            }
            else {
                root->RotateInSystem(system, -0.1f, Axis::X);
            }
            break;
        case GLFW_KEY_LEFT:
            if (pickedModel) {
                int size = cyls.size();
                for (int i = 0; i < size; i++) {
                    if (pickedModel == cyls[i]) {
                                                
                        Eigen::Vector3f angles = pickedModel->GetRotation().eulerAngles(2, 0, 2);
                        Eigen::Matrix3f phiMat;
                        phiMat.row(0) = Eigen::Vector3f({ cos(angles[0]),-sin(angles[0]),0 });
                        phiMat.row(1) = Eigen::Vector3f({ sin(angles[0]),cos(angles[0]), 0 });
                        phiMat.row(2) = Eigen::Vector3f({ 0,0, 1 });
                        Eigen::Matrix3f thetaMat;
                        thetaMat.row(0) = Eigen::Vector3f({ 1, 0, 0 });
                        thetaMat.row(1) = Eigen::Vector3f({ 0, cos(angles[1]),-sin(angles[1]) });
                        thetaMat.row(2) = Eigen::Vector3f({ 0, sin(angles[1]),cos(angles[1]) });
                        Eigen::Matrix3f psiMat;
                        psiMat.row(0) = Eigen::Vector3f({ cos(angles[2]),-sin(angles[2]),0 });
                        psiMat.row(1) = Eigen::Vector3f({ sin(angles[2]),cos(angles[2]), 0 });
                        psiMat.row(2) = Eigen::Vector3f({ 0, 0, 1 });
                        Eigen::Matrix3f rotation;
                        rotation << cos(0.1f), -sin(0.1f), 0,
                            sin(0.1f), cos(0.1f), 0,
                            0, 0, 1;
                        pickedModel->Rotate(pickedModel->GetRotation().inverse() * phiMat*thetaMat*psiMat*rotation);
                    }
                }
            }
            else {
                root->RotateInSystem(system, 0.1f, Axis::Y);
            }
            break;
        case GLFW_KEY_RIGHT:
            if (pickedModel) {
                int size = cyls.size();
                for (int i = 0; i < size; i++) {
                    if (pickedModel == cyls[i]) {
                        Eigen::Vector3f angles = pickedModel->GetRotation().eulerAngles(2, 0, 2);
                        Eigen::Matrix3f phiMat;
                        phiMat.row(0) = Eigen::Vector3f({ cos(angles[0]),-sin(angles[0]),0 });
                        phiMat.row(1) = Eigen::Vector3f({ sin(angles[0]),cos(angles[0]), 0 });
                        phiMat.row(2) = Eigen::Vector3f({ 0,0, 1 });
                        Eigen::Matrix3f thetaMat;
                        thetaMat.row(0) = Eigen::Vector3f({ 1, 0, 0 });
                        thetaMat.row(1) = Eigen::Vector3f({ 0, cos(angles[1]),-sin(angles[1]) });
                        thetaMat.row(2) = Eigen::Vector3f({ 0, sin(angles[1]),cos(angles[1]) });
                        Eigen::Matrix3f psiMat;
                        psiMat.row(0) = Eigen::Vector3f({ cos(angles[2]),-sin(angles[2]),0 });
                        psiMat.row(1) = Eigen::Vector3f({ sin(angles[2]),cos(angles[2]), 0 });
                        psiMat.row(2) = Eigen::Vector3f({ 0, 0, 1 });
                        Eigen::Matrix3f rotation;
                        rotation << cos(-0.1f), -sin(-0.1f), 0,
                            sin(-0.1f), cos(-0.1f), 0,
                            0, 0, 1;
                        pickedModel->Rotate(pickedModel->GetRotation().inverse() * phiMat * thetaMat * psiMat * rotation);
                    }
                }
            }
            else {
                root->RotateInSystem(system, -0.1f, Axis::Y);
            }
            break;
        case GLFW_KEY_W:
            camera->TranslateInSystem(system, { 0, 0.1f, 0 });
            break;
        case GLFW_KEY_S:
            camera->TranslateInSystem(system, { 0, -0.1f, 0 });
            break;
        case GLFW_KEY_A:
            camera->TranslateInSystem(system, { -0.1f, 0, 0 });
            break;
        case GLFW_KEY_B:
            camera->TranslateInSystem(system, { 0, 0, 0.1f });
            break;
        case GLFW_KEY_F:
            camera->TranslateInSystem(system, { 0, 0, -0.1f });
            break;
        case GLFW_KEY_SPACE:
            runCCD = !runCCD;
            break;
        case GLFW_KEY_P:
            if (pickedModel == NULL) {
                Eigen::Matrix3f rM = root->GetRotation();
                std::cout << "The Rotation Matrix of The root: \n";
                std::cout << rM << std::endl;
            }
            else {
                Eigen::Vector3f angles = pickedModel->GetRotation().eulerAngles(2, 0, 2);
                Eigen::Matrix3f phiMat;
                phiMat.row(0) = Eigen::Vector3f({ cos(angles[0]),-sin(angles[0]),0 });
                phiMat.row(1) = Eigen::Vector3f({ sin(angles[0]),cos(angles[0]), 0 });
                phiMat.row(2) = Eigen::Vector3f({ 0,0, 1 });
                Eigen::Matrix3f thetaMat;
                thetaMat.row(0) = Eigen::Vector3f({ 1, 0, 0 });
                thetaMat.row(1) = Eigen::Vector3f({ 0, cos(angles[1]),-sin(angles[1]) });
                thetaMat.row(2) = Eigen::Vector3f({ 0, sin(angles[1]),cos(angles[1]) });
                Eigen::Matrix3f psiMat;
                psiMat.row(0) = Eigen::Vector3f({ cos(angles[2]),-sin(angles[2]),0 });
                psiMat.row(1) = Eigen::Vector3f({ sin(angles[2]),cos(angles[2]), 0 });
                psiMat.row(2) = Eigen::Vector3f({ 0,0, 1 });

                std::cout << "The Rotation Matrcies: \n";
                std::cout << "phi matrix:\n" << phiMat << std::endl;
                std::cout << "theta matrix:\n" << thetaMat << std::endl;
                std::cout << "psi matrix:\n" << psiMat << std::endl;
            }
            break;
        case GLFW_KEY_T: {
            Eigen::Vector3f tippos = GetArmTip();
            std::cout << "Tip position : " << "(" << tippos[0] << "," << tippos[1] << "," << tippos[2] << ")" << std::endl;
        }
            break;
        case GLFW_KEY_D: {
            Eigen::Vector3f goalpos = getGoalPosition();
            std::cout << "Goal position : " << "(" << goalpos[0] << "," << goalpos[1] << "," << goalpos[2] << ")" << std::endl;
            }
            break;
        case GLFW_KEY_N:
            nextLink();
            break;
        }
    }
}
void BasicScene::nextLink() {
    bool is = false;
    for (int i = 0; i < cyls.size(); i++) {
        if (pickedModel == cyls[i]) {
            std::cout << "The pickedModel is: " << i << std::endl;
            is = true;
            if (i + 1 == cyls.size())
                pickedModel = cyls[0];
            else
                pickedModel = cyls[i + 1];
            break;
        }
    }
    if (!is)
        pickedModel = cyls[0];
}

Eigen::Vector3f BasicScene::GetArmTip() {
    Eigen::Vector3f v = Eigen::Vector3f(0,0.8f,0);//The length of the first link
    Eigen::Vector3f center = cyls[cyls.size()-1]->GetTranslation();
    Eigen::Vector3f tip = cyls[cyls.size() - 1]->GetRotation() * v + center;
    return tip;
}

Eigen::Vector3f BasicScene::getGoalPosition() {
    return sphere1->GetTranslation();
}
Eigen::Vector3f BasicScene::getLinkPosition(int index) {
    Eigen::Vector3f v = Eigen::Vector3f(0, 0.8f, 0);
    Eigen::Vector3f center = cyls[index]->GetTranslation();
    Eigen::Vector3f linkPos = center - cyls[index]->GetRotation() * v;
    return linkPos;
}


void BasicScene::CCD() {
    //Checking if we can reach it by calculate the distance between the root and the goal and checks if the
    //sum of the links is less than the distance so we can't reach it else we start the algorithm.
    if (runCCD) {
        Eigen::Vector3f D = getGoalPosition();
        Eigen::Vector3f G = getLinkPosition(0);
        int size = cyls.size();
        float dist = (D - G).norm();
        bool canReach = dist <= 1.6 * size;
        if (canReach) {
            for (int i = size - 1; i >= 0; i--) {
                D = getGoalPosition();
                Eigen::Vector3f E = GetArmTip();
                Eigen::Vector3f R = getLinkPosition(i);
                Eigen::Vector3f RD = D - R;
                Eigen::Vector3f RE = E - R;
                //Calculate distance and check if we are close enough to the goal.
                float distance = (D - E).norm();
                if (distance <= delta && !arrived) {
                    std::cout << "Arrived!" << std::endl;
                    arrived = true;
                    break;
                }
                else if (distance > delta) {
                    arrived = false;
                    float dot = RD.normalized().dot(RE.normalized());
                    dot = dot > 1 ? 1 : dot < -1 ? -1 : dot;
                    float angle = ( acosf(dot) /100.f);
                    Eigen::Vector3f direction = cyls[i]->GetRotation().inverse() *  RE.normalized().cross(RD.normalized());
                    cyls[i]->Rotate(angle, direction);
                }
            }
        }
        else {//we cant reach the goal
            std::cout << "Cannot Reach!" << std::endl;
            runCCD = false;
        }
    }
}