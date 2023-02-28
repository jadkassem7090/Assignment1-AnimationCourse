
#include "BasicScene.h"
#include <Eigen/src/Core/Matrix.h>
#include <edges.h>
#include <memory>
#include <per_face_normals.h>
#include <read_triangle_mesh.h>
#include <utility>
#include "imgui.h"
#include "file_dialog_open.h"
#include <vector>
#include "GLFW/glfw3.h"
#include "Mesh.h"
#include "PickVisitor.h"
#include "Renderer.h"
#include "ObjLoader.h"
#include "IglMeshLoader.h"
#include <thread>
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
#include "SceneWithImGui.h"
#include <igl/directed_edge_parents.h>
#include <igl/forward_kinematics.h>
#include <igl/dqs.h>
#include <cstdlib>
#include <ctime>
#include <stdio.h>
#include <conio.h>
#include <random>
#include <Windows.h>
#include <mmsystem.h>
#pragma comment(lib, "WinMM.lib")

// #include "AutoMorphingModel.h"

using namespace cg3d;

void BasicScene::BuildImGui() {
    int flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize;
    bool* pOpen = nullptr;
    ImGui::Begin("Menu", pOpen, flags);
    //StartMenu
    if (currentState == startmenu) {
        float width = 200;
        float height = 180;
        ImGui::SetWindowPos(ImVec2((renderer->GetWindowWidth() - width) / 2, (renderer->GetWindowHeight() - height) / 2), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(width, height));
        TextCentered("Snake Game : Main Menu", 30);
        auto windowWidth = ImGui::GetWindowSize().x / 2;
        auto buttonS = ImVec2(100, 30);
        ImGui::SetCursorPosX(windowWidth - buttonS.x / 2);
        if (ImGui::Button("Start Game", buttonS))
        {
            timeBeforePause = lap;
            currentState = level1;
            time = std::chrono::steady_clock::now();
            Level1(true);
        }
        ImGui::SetCursorPosX(windowWidth - buttonS.x / 2);
        if (ImGui::Button("Settings", buttonS))
        {
            previousState = currentState;
            currentState = settings;
        }
        ImGui::SetCursorPosX(windowWidth - buttonS.x / 2);
        if (ImGui::Button("Quit Game", buttonS))
        {
            exit(0);
        }
    }
    else if (currentState == level1 || currentState == level2 || currentState == level3) {
        float width = 120, height = 150;
        ImGui::SetWindowPos(ImVec2(0, 0), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(width, height));
        std::string level = currentState == level1 ? "Level: 1" : currentState == level2 ? "Level: 2" : "Level: 3";
        std::string tscore = "Score: " + std::to_string(score);
        std::string thisL = "This Level: " + std::to_string(thisLevelScore);
        std::string lifeT = "Life: " + std::to_string(life);
        std::chrono::duration<int> time1 = std::chrono::duration_cast<std::chrono::duration<int>>(std::chrono::steady_clock::now() - time);
        std::string mytime = "Time: " + std::to_string(timeBeforePause-time1.count());
        if (timeBeforePause - time1.count() == 0) {
            PlaySound(TEXT("data/lose.wav"), NULL, SND_FILENAME | SND_ASYNC);
            previousState = currentState;
            currentState = die;
            thisLevelScore = 0;
            life = 0;
        }
        ImGui::Text(mytime.c_str());
        ImGui::Text(level.c_str());
        ImGui::Text(tscore.c_str());
        ImGui::Text(thisL.c_str());
        ImGui::Text(lifeT.c_str());
        if (ImGui::Button("Pause Game")) {
            timeBeforePause = timeBeforePause - time1.count();
            previousState = currentState;
            currentState = paused;
        }
    }
    else if (currentState == paused) {
        float width = 200;
        float height = 180;
        ImGui::SetWindowPos(ImVec2((renderer->GetWindowWidth() - width) / 2, (renderer->GetWindowHeight() - height) / 2), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(width, height));
        TextCentered("Snake Game : Paused", 30);
        auto windowWidth = ImGui::GetWindowSize().x / 2;
        auto buttonS = ImVec2(100, 30);
        ImGui::SetCursorPosX(windowWidth - buttonS.x / 2);
        if (ImGui::Button("Resume", buttonS))
        {
            time = std::chrono::steady_clock::now();
            currentState = previousState;
        }
        ImGui::SetCursorPosX(windowWidth - buttonS.x / 2);
        if (ImGui::Button("Settings", buttonS))
        {
            currentState = settings;
        }
        ImGui::SetCursorPosX(windowWidth - buttonS.x / 2);
        if (ImGui::Button("Quit Game", buttonS))
        {
            exit(0);
        }
    }else if (currentState == betweenLevels || currentState == die) {
        float width = 250;
        float height = 180;
        ImGui::SetWindowPos(ImVec2((renderer->GetWindowWidth() - width) / 2, (renderer->GetWindowHeight() - height) / 2), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(width, height));
        TextCentered("Snake Game", 30);
        std::string tscore = "Total Score: " + std::to_string(score);
        TextCentered(tscore);
        auto windowWidth = ImGui::GetWindowSize().x / 2;
        auto buttonS = ImVec2(100, 30);
        if (previousState == level3 && currentState == betweenLevels) {
            TextCentered("Congrats", 0);
            TextCentered("You Finished All The Levels!", 0);
        }
        if (currentState == die) {
            TextCentered("You Failed SUCCESSFULY", 0);
        }
        ImGui::SetCursorPosX(windowWidth - buttonS.x / 2);
        life = 0;
        if (ImGui::Button("Play Again", buttonS))
        {
            timeBeforePause = lap;
            time = std::chrono::steady_clock::now();
            thisLevelScore = 0;
            currentState = previousState;
            if (currentState == level1)
                Level1(false);
            else if (currentState == level2)
                Level2();
            else
                Level3(false);
        }
        if (currentState == betweenLevels && (previousState == level1 || previousState == level2)) {
            ImGui::SetCursorPosX(windowWidth - buttonS.x / 2);
            if (ImGui::Button("Next Level", buttonS))
            {
                timeBeforePause = lap;
                time = std::chrono::steady_clock::now();
                score += thisLevelScore;
                thisLevelScore = 0;
                currentState = previousState == level1 ? level2 : level3;
                if(currentState == level2){
                    Level2();
                }
                else if (currentState == level3) {
                    Level3(true);
                }
            }
        }
        ImGui::SetCursorPosX(windowWidth - buttonS.x / 2);
        if (ImGui::Button("Quit Game", buttonS))
        {
            exit(0);
        }
    }
    else if (currentState == settings) {
        float width = 200;
        float height = 180;
        ImGui::SetWindowPos(ImVec2((renderer->GetWindowWidth() - width) / 2, (renderer->GetWindowHeight() - height) / 2), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(width, height));
        TextCentered("Snake Game : Settings", 30);
        auto windowWidth = ImGui::GetWindowSize().x/2;
        auto buttonS = ImVec2(100, 30);
        auto checkboxS = ImVec2(30, 20);
        ImGui::SetCursorPosX(windowWidth - checkboxS.x);
        ImGui::Checkbox("Music", &music);
        ImGui::SetCursorPosX(windowWidth - checkboxS.x);
        ImGui::Checkbox("Effects", &effects);
        ImGui::SetCursorPosX(windowWidth - buttonS.x/2);
        if (ImGui::Button("Back", buttonS)) {
            if (previousState == startmenu) {
                currentState = previousState;
            }
            else {
                currentState = paused;
            }
        }
    }
    ImGui::End();
}
void BasicScene::TextCentered(std::string text, float margin)
{
    auto windowWidth = ImGui::GetWindowSize().x;
    auto textWidth = ImGui::CalcTextSize(text.c_str()).x;

    ImGui::SetCursorPosX((windowWidth - textWidth) * 0.5f);
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + margin);

    ImGui::Text(text.c_str());
}
void BasicScene::cursorCentered(std::string text, float margin)
{
    auto windowWidth = ImGui::GetWindowSize().x;
    auto textWidth = ImGui::CalcTextSize(text.c_str()).x;

    ImGui::SetCursorPosX((windowWidth - textWidth) * 0.5f);
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + margin);
}
void BasicScene::Init(float fov, int width, int height, float nnear, float ffar)
{
    camera = Camera::Create("camera", fov, float(width) / height, nnear, ffar);
    dheight = height;
    dwidth = width;
    mynear = nnear;
    myfar = ffar;
    auto daylight{ std::make_shared<Material>("daylight", "shaders/cubemapShader") };
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{ Model::Create("background", Mesh::Cube(), daylight) };
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();


//    auto program = std::make_shared<Program>("shaders/phongShader");
//    auto program1 = std::make_shared<Program>("shaders/pickingShader");
//
//    auto material{ std::make_shared<Material>("material", program) }; // empty material
//    auto material1{ std::make_shared<Material>("material", program1) }; // empty material
////    SetNamedObject(cube, Model::Create, Mesh::Cube(), material, shared_from_this());
//
//    material->AddTexture(0, "textures/box0.bmp", 2);
//    auto sphereMesh{ IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj") };
//    auto cylMesh{ IglLoader::MeshFromFiles("cyl_igl","data/zcylinder.obj") };
//    auto snakeMesh{ IglLoader::MeshFromFiles("cyl_igl","data/snake1.obj") };
//    auto cubeMesh{ IglLoader::MeshFromFiles("cube_igl","data/cube_old.obj") };
//    sphere1 = Model::Create("sphere", sphereMesh, material);
//
//    //Map
//    walls.push_back(Model::Create("wall0", cubeMesh, material));
//    walls[0]->Scale(10,Axis::X);
//    walls[0]->Translate(20,Axis::Z);
//    walls.push_back(Model::Create("wall1", cubeMesh, material));
//    walls[1]->Scale(10,Axis::X);
//    walls[1]->Translate(40,Axis::Z);
//    //Axis
//    Eigen::MatrixXd vertices(6, 3);
//    vertices << -1, 0, 0, 1, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, -1, 0, 0, 1;
//    Eigen::MatrixXi faces(3, 2);
//    faces << 0, 1, 2, 3, 4, 5;
//    Eigen::MatrixXd vertexNormals = Eigen::MatrixXd::Ones(6, 3);
//    Eigen::MatrixXd textureCoords = Eigen::MatrixXd::Ones(6, 2);
//    std::shared_ptr<Mesh> coordsys = std::make_shared<Mesh>("coordsys", vertices, faces, vertexNormals, textureCoords);
//    //axis.push_back(Model::Create("axis", coordsys, material1));
//    //axis[0]->mode = 1;
//    //axis[0]->Scale(4, Axis::XYZ);
//    // axis[0]->lineWidth = 5;
//    //root->AddChild(axis[0]);
//    float scaleFactor = 1;
//    cyls.push_back(Model::Create("cyl", cylMesh, material));
//    cyls[0]->Scale(scaleFactor, Axis::Z);
//    cyls[0]->SetCenter(Eigen::Vector3f(0, 0, -0.8f * scaleFactor));
//    root->AddChild(cyls[0]);
//    cylsTree.resize(linksnum);
//    cylsTree[0].init(cyls[0]->GetMesh()->data[0].vertices, cyls[0]->GetMesh()->data[0].faces);
//    cyls[0]->showFaces = true;
//    cyls[0]->showWireframe = false;
//    cyls[0]->showTextures = false;
//
//    //skinning
//    C = Eigen::MatrixXd(linksnum + 1, 3);
//    BE = Eigen::MatrixXi(linksnum, 2);
//    C.row(0) << 0, 0, -1.6 * (linksnum / 2.0f);
//    BE.row(0) << 0, 1;
//    for (int i = 1; i < linksnum; i++)
//    {
//        cyls.push_back(Model::Create("cyl", cylMesh, material));
//        cyls[i]->Translate(1.6f * scaleFactor, Axis::Z);
//        if(i == linksnum-1)
//            cyls[i]->Translate(1.6f * (linksnum-2) * scaleFactor, Axis::Z);
//        cyls[i]->SetCenter(Eigen::Vector3f(0, 0, -0.8f * scaleFactor));
//        if (i != linksnum-1) {
//            cyls[i - 1]->AddChild(cyls[i]);
//        }
//        else {
//            root->AddChild(cyls[i]);
//        }
//        cylsTree[i].init(cyls[i]->GetMesh()->data[0].vertices, cyls[i]->GetMesh()->data[0].faces);
//        cyls[i]->showFaces = true;
//        cyls[i]->showWireframe = false;
//        C.row(i) << 0, 0, 1.6 * (i-(linksnum / 2.0f));
//        BE.row(i) << i, i+1;
//    }
//    C.row(linksnum) << 0, 0, 1.6 * ((linksnum / 2.0f));
//    root->RotateByDegree(-90.0f, Axis::X);
//    root2->RotateByDegree(-90.0f, Axis::X);
//    snake = Model::Create("snake", snakeMesh, material);
//    root->AddChild(snake);
//    
//    snake->Translate(1.6*linksnum/2,Axis::Z);
//    V = scaledVertices(snakeMesh->data[0].vertices, { 1, 1, (float)linksnum});
//    F = snakeMesh->data[0].faces;
//    U = V;
//    /*Eigen::MatrixXd vN;
//    igl::per_vertex_normals(U, snake->GetMesh()->data[0].faces, vN);
//    Mesh nextPose("snake", U, F, vN, snake->GetMesh()->data[0].textureCoords);
//    snake->SetMeshList({ std::make_shared<Mesh>(nextPose) });*/
//    igl::directed_edge_parents(BE, P);
//    CalculateWeights();
//    cyls[0]->Translate({ 0,0,0.8f * scaleFactor });
//    root->RotateByDegree(0, Eigen::Vector3f(1, 0, 0));
//    
//    sphereTree.init(sphere1->GetMesh()->data[0].vertices, sphere1->GetMesh()->data[0].faces);
//    auto morphFunc = [](Model* model, cg3d::Visitor* visitor) {
//        return model->meshIndex;//(model->GetMeshList())[0]->data.size()-1;
//    };
//    cyls[linksnum - 1]->AddChild(camera);
//    root->Translate(-20,Axis::Y);
//    sphere1->showWireframe = true;
//    sphere1->Translate({ 0,0,30 });
//    camera->Translate(50, Axis::Y);
//    camera->RotateByDegree(-90, Axis::X);
//    camera->RotateByDegree(180, Axis::Z);
//    root2->AddChild(sphere1);
//    root2->AddChild(walls[0]);
//    root2->AddChild(walls[1]);
}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 0.8f, 0.3f, 0.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 0.3f, 0.6f, 1.0f);
    program.SetUniform4f("Kdi", 0.5f, 0.5f, 0.0f, 1.0f);
    program.SetUniform1f("specular_exponent", 5.0f);
    program.SetUniform4f("light_position", 0.0, 15.0f, 0.0, 1.0f);
    if (currentState == level1 || currentState == level2 || currentState == level3) {
        if (arrived) {
            previousState = currentState;
            currentState = betweenLevels;
            arrived = false;
            runCCD = false;
        }
        if (currentState == level3) {
            if (bombs[0]->GetTranslation().x() == 0 || bombs[0]->GetTranslation().x() == 40)
                direc = -direc;
            bombs[0]->Translate(-0.05f*direc, Axis::X);
            bombs[1]->Translate(-0.05f*direc, Axis::X);
            bombs[2]->Translate(0.05f*direc, Axis::X);
            bombs[3]->Translate(0.05f*direc, Axis::X);
        }
        if(currentState == level1)
            root2->Translate(-0.01f, Axis::Y);
        for (int i = 0; i < walls.size(); i++) {
            walls[i]->Rotate(0, Axis::XYZ);
        }

        Eigen::Vector3f t(0, 0, 1);
        t = root->GetRotation().inverse() * cyls[linksnum - 1]->GetRotation() * t;
        t = t.normalized() * 0.01;
        cyls[linksnum - 1]->Translate(t);

        FabricAlgo(getLinkPosition(cyls.size() - 1), cyls.size() - 2);
        checkCollisionBetweenCylsAndShpere();
        if(currentState == level3)
            checkCollisionBetweenCylsAndBombs();
        checkCollisionBetweenCylsAndWalls();
        if (frameC % 10 == 0) {
            frameC = 0;
        }
        frameC++;
    }
}
//Skinning functions!
void BasicScene::ApplySkinning() {
    RotationList anim_pose(cyls.size());

    for (size_t i = 0; i < cyls.size(); i++)
    {
        auto qf = Eigen::Quaternionf(cyls[i]->GetTout().rotation()).normalized();
        anim_pose[i] = Eigen::Quaterniond(qf.w(), qf.x(), qf.y(), qf.z());
    }
    anim_pose[linksnum - 1] = an[linksnum-1];

    RotationList vQ;
    std::vector<Eigen::Vector3d> vT;

    igl::forward_kinematics(C, BE, P, anim_pose, vQ, vT);

    igl::dqs(V, W, vQ, vT, U);
    Eigen::MatrixXd vN;
    igl::per_vertex_normals(U, snake->GetMesh()->data[0].faces, vN);
    Mesh nextPose("snake", U, F, vN, snake->GetMesh()->data[0].textureCoords);
    snake->SetMeshList({ std::make_shared<Mesh>(nextPose) });
}
Eigen::MatrixXd BasicScene::scaledVertices(const Eigen::MatrixXd& vertices, const Eigen::Vector3d& scale)
{
    Eigen::DiagonalMatrix<double, 3> scalingMatrix(scale);
    Eigen::MatrixXd scaledVertices = vertices * scalingMatrix;

    return scaledVertices;
}
void BasicScene::CalculateWeights() {
    W = Eigen::MatrixXd::Zero(V.rows(), C.rows() - 1);
    for (size_t i = 0; i < V.rows(); i++)
    {
        Eigen::Vector3f v = V.row(i).cast<float>().eval();
        auto res = getWeightsandIndecies(v);
        W.row(i)[(int)res[0]] = (1/res[2])/((1/res[2])+ (1 / res[3]));
        W.row(i)[(int)res[1]] = (1/res[3]) / ((1 / res[2]) + (1 / res[3]));
    }
}
std::vector<float> BasicScene::getWeightsandIndecies(Eigen::Vector3f v) {
    int index1 = 0, index2 = 1;
    Eigen::Vector3f v1 = C.row(0).cast<float>(),eval();
    float distance1 = (v - v1).norm();
    v1 = C.row(1).cast<float>().eval();
    float distance2 = (v - v1).norm();
    if (distance1 > distance2) {//Swaping the indcies and the distance in case that the second link is closer than the first link
        index1 = 1;
        index2 = 0;
        float x = distance1;
        distance1 = distance2;
        distance2 = x;
    }
    for (int i = 2; i < C.rows(); i++) {
        v1 = C.row(i).cast<float>().eval();
        float dist = (v - v1).norm();
        if (dist < distance1) {
            index2 = index1;
            distance2 = distance1;
            index1 = i;
            distance1 = dist;
        }
        else if (dist < distance2) {
            index2 = i;
            distance2 = dist;
        }
    }
    if (index1 == C.rows() - 1)
    {
        index1 = index2;
    }
    else if (index2 == C.rows() - 1)
    {
        index2 = index1;
    }
    std::vector<float> ret;
    ret.push_back(index1);
    ret.push_back(index2);
    ret.push_back(distance1);
    ret.push_back(distance2);
    return ret;
}

void BasicScene::MouseCallback(Viewport* viewport, int x, int y, int button, int action, int mods, int buttonState[])
{
    // note: there's a (small) chance the button state here precedes the mouse press/release event
    if (ImGui::GetIO().WantCaptureMouse)
        return;
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
        //Eigen::Vector3f qevoon(xAtPress,yAtPress,cyls[cyls.size()-1]->GetTranslation().z());

        //float dotProduct = qevoon.dot(cyls[cyls.size() - 1]->GetRotation()*Eigen::Vector3f(0,1,0)); // calculate dot product
        //float mag1 = qevoon.norm(); // calculate magnitude of first vector
        //float mag2 = (cyls[cyls.size() - 1]->GetRotation() * Eigen::Vector3f(0, 1, 0)).norm(); // calculate magnitude of second vector

        //float cosTheta = dotProduct / (mag1 * mag2); // calculate cosine of angle between vectors

        //float angle = std::acos(cosTheta) * 180 / pi;
        //cyls[cyls.size() - 1]->Rotate(angle,qevoon);/*
        
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
    if (ImGui::GetIO().WantCaptureMouse)
        return;
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
                cyls[0]->Translate(root->GetRotation().inverse() * (xx * -float(yoffset)));
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
    if (ImGui::GetIO().WantCaptureMouse)
        return;
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
                if (!is)
                    pickedModel->TranslateInSystem(system * root->GetRotation(), { -float(xAtPress - x) / moveCoeff, float(yAtPress - y) / moveCoeff, 0 });

            }if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
                pickedModel->RotateInSystem(system, float(xAtPress - x) / angleCoeff, Axis::Z);
            if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE) {
                //Here To make the rotation like pressing on the arrows.
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
void BasicScene::AddViewportCallback(Viewport* _viewport)
{
    viewport = _viewport;

    Scene::AddViewportCallback(viewport);
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
        case GLFW_KEY_UP: {
            Eigen::Vector3f t(0,0, 1);
            t = root->GetRotation().inverse() * cyls[linksnum-1]->GetRotation() * t;
            t = t.normalized()*0.5; 
            cyls[linksnum-1]->Translate(t);
            break; }
        case GLFW_KEY_DOWN:
            std::cout << "NOT" << std::endl;
            break;
        case GLFW_KEY_LEFT:
        {
            if (currentState == level1) {
                Eigen::Matrix3f rot1 = camera->GetRotation();
                cyls[cyls.size() - 1]->RotateByDegree(10, Axis::Y);
                Eigen::Matrix3f rot2 = camera->GetRotation();
                Eigen::Quaternionf qf = Eigen::Quaternionf(an[linksnum - 1].toRotationMatrix().cast<float>() * (rot2.inverse() * rot1));
                Eigen::Quaterniond qd = Eigen::Quaterniond(qf.w(), qf.x(), qf.y(), qf.z());
                an[linksnum - 1] = qd;
                camera->Rotate(rot2.inverse());
                camera->Rotate(rot1);
            }
            else if (currentState == level2 || currentState == level3){
                Eigen::Matrix3f rot1 = camera->GetRotation();
                cyls[cyls.size() - 1]->RotateByDegree(-10, rot1*Eigen::Vector3f(0,1,0));
            }
            break; }
        case GLFW_KEY_RIGHT: {
            if (currentState == level1) {
                Eigen::Matrix3f rot1 = camera->GetRotation();
                cyls[cyls.size() - 1]->RotateByDegree(-10, Axis::Y);
                Eigen::Matrix3f rot2 = camera->GetRotation();
                Eigen::Quaternionf qf = Eigen::Quaternionf(an[linksnum-1].toRotationMatrix().cast<float>() *(rot2.inverse() * rot1));
                Eigen::Quaterniond qd = Eigen::Quaterniond(qf.w(),qf.x(),qf.y(),qf.z());
                an[linksnum - 1] = qd;
                camera->Rotate(rot2.inverse());
                camera->Rotate(rot1);
            }
            else if (currentState == level2 || currentState == level3) {
                Eigen::Matrix3f rot1 = camera->GetRotation();
                cyls[cyls.size() - 1]->RotateByDegree(10, rot1 * Eigen::Vector3f(0, 1, 0));
            }
            break; }
        case GLFW_KEY_W:
            //camera->TranslateInSystem(system, { 0, 0.1f, 0 });
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
        case GLFW_KEY_C:
            if (currentState == level1 || currentState == level2 || currentState == level3) {
                switchCamera();
            }
            break;
        }
    }
    //SceneWithImGui::KeyCallback(nullptr, x, y, key, scancode, action, mods);
}
void BasicScene::switchCamera() {
        viewport->camera = viewport->camera ==camera ?camList[0] : camera;
}
Eigen::Vector3f BasicScene::GetArmTip() {
    Eigen::Vector3f v = Eigen::Vector3f(0, 0, 0.8f);//The length of the first link
    Eigen::Vector3f center = cyls[cyls.size() - 1]->GetTranslation();
    Eigen::Vector3f tip = cyls[cyls.size() - 1]->GetRotation() * v + center;
    return tip;
}

Eigen::Vector3f BasicScene::getGoalPosition() {
    return sphere1->GetTranslation();
}
Eigen::Vector3f BasicScene::getLinkPosition(int index) {
    Eigen::Vector3f v = Eigen::Vector3f(0, 0, 0.8f);
    Eigen::Vector3f center = cyls[index]->GetTranslation();
    Eigen::Vector3f linkPos = center - cyls[index]->GetRotation() * v;
    return linkPos;
}
Eigen::Vector3f BasicScene::getTipLinkPosition(int index) {
    Eigen::Vector3f v = Eigen::Vector3f(0, 0, 0.8f);
    Eigen::Vector3f center = cyls[index]->GetTranslation();
    Eigen::Vector3f linkPos = center + cyls[index]->GetRotation() * v;
    return linkPos;
}

void BasicScene::FabricAlgo(Eigen::Vector3f Goal, int index) {
    if (runCCD) {
            Eigen::Vector3f D = Goal;
            Eigen::Vector3f G = getLinkPosition(0);
            int size = index + 1;
            float dist = (D - G).norm();
            bool canReach = dist <= 1.6 * size;
            std::vector<Eigen::Vector3f> p;
            p.resize(size + 1);
            int curr = 0;
            while (curr != size) {
                p[curr] = getLinkPosition(curr);
                curr++;
            }
            p[curr] = getTipLinkPosition(index);
            std::vector<double> ri;
            std::vector<double> lambdai;
            ri.resize(size + 1);
            lambdai.resize(size + 1);

            Eigen::Vector3f b = p[0];
            Eigen::Vector3f end_effector = p[size];
           
            float diff = (end_effector - D).norm();
            while (diff > delta) {
                p[size] = D;
                int parent = size - 1;
                int child = size;
                while (parent != -1) {
                    ri[parent] = (p[child] - p[parent]).norm();
                    lambdai[parent] = 1.6f / ri[parent];
                    p[parent] = (1 - lambdai[parent]) * p[child] + lambdai[parent] * p[parent];
                    child = parent;
                    parent = parent - 1;
                }
                diff = (p[size] - D).norm();
            }
            int curre = 0;
            int target = 1;
            while (curre != size) {
                Help(curre, p[target]);
                curre++;
                target++;
            }
            Eigen::Vector3f xx = (getLinkPosition(0) - p[0]).normalized();
            float dis = (getLinkPosition(0) - p[0]).norm();
            //dis = dis > 0.1f ? 0.1f : dis;
            if (withSkinning && frameC % 10 == 0) {
                ApplySkinning();
            }
            cyls[0]->Translate(root->GetRotation().inverse() * (xx * -float(dis)));
            Eigen::Vector3f xx1 = (snake->GetTranslation() - p[0]);
            if(withSkinning)
                snake->Translate(root->GetRotation().inverse() * (xx * -float(dis)));
            float distance = (D - getTipLinkPosition(index)).norm();
        }
    }

void BasicScene::Help(int i, Eigen::Vector3f D) {
    Eigen::Vector3f E = getTipLinkPosition(i);
    Eigen::Vector3f R = getLinkPosition(i);
    Eigen::Vector3f RD = D - R;
    Eigen::Vector3f RE = E - R;
    float distance = (D - E).norm();
    float dot = RD.normalized().dot(RE.normalized());
    dot = dot > 1 ? 1 : dot < -1 ? -1 : dot;
    float angle = (acosf(dot));
    Eigen::Vector3f direction = cyls[i]->GetRotation().inverse() * RE.normalized().cross(RD.normalized());
    Eigen::Matrix3f rot = cyls[i]->GetRotation().inverse();
    cyls[i]->Rotate(angle, direction);
    rot = rot * cyls[i]->GetRotation();//here is the rotation that happend last!
    rot = an[i].toRotationMatrix().cast<float>()* rot;
    Eigen::Quaternionf q = Eigen::Quaternionf(rot).normalized();
    an[i] = Eigen::Quaterniond(q.w(),q.x(), q.y(), q.z());
}

float BasicScene::GenerateRandomNumber(int a , int b) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(a, b);

    float rand_num = dis(gen);
    return rand_num;
}
void BasicScene::checkCollisionBetweenCylsAndShpere() {
    for (int i = 0; i < spheres.size();i++) {
        if (atenSpheres[i] == false && CheckCollision(&cylsTree[cyls.size() - 1], &sphereTrees[i], cyls[cyls.size() - 1], spheres[i])) {
            if (i == spheres.size() - 1) {
                previousState = currentState;
                currentState = betweenLevels;
            }
            PlaySound(TEXT("data/eat.wav"), NULL, SND_FILENAME | SND_ASYNC);
            spheres[i]->isHidden = true;
            //root2->RemoveChild(spheres[i]);
            atenSpheres[i] = true;
            if (currentState == level2 || currentState == level3) {
                auto program = std::make_shared<Program>("shaders/phongShader");

                auto material{ std::make_shared<Material>("material", program) };

                material->AddTexture(0, "textures/box0.bmp", 2);
                auto sphereMesh{ IglLoader::MeshLoader2("sphere_igl", "data/sphere.obj") };
                root2->RemoveChild(spheres[i]);
                spheres[i] = Model::Create("sphere", sphereMesh, material);
                
                sphereTrees[i].init(spheres[i]->GetMesh()->data[0].vertices, spheres[i]->GetMesh()->data[0].faces);
                root2->AddChild(spheres[i]);
                spheres[i]->Translate({ GenerateRandomNumber(-38,38),GenerateRandomNumber(-27,27),0 });
                atenSpheres[i] = false;
            }

            thisLevelScore++;
            if (thisLevelScore % 2 == 0) {
                life++;
            }
        }
    }
}
void BasicScene::checkCollisionBetweenCylsAndBombs() {
    for (int i = 0; i < bombs.size(); i++) {
        for (int j = 0; j < cyls.size(); j++) {
            if (CheckCollision(&cylsTree[j], &bombsTree[i], cyls[j], bombs[i])) {
                if (life > 0) {
                    life--;
                    Eigen::Vector3f t(0, 0, -1);
                    t = root->GetRotation().inverse() * cyls[linksnum - 1]->GetRotation() * t;
                    t = t.normalized() * 5;
                    cyls[linksnum - 1]->Translate(t);
                    cyls[0]->Translate(t);
                }
                else {
                    PlaySound(TEXT("data/lose.wav"), NULL, SND_FILENAME | SND_ASYNC);
                    previousState = currentState;
                    currentState = die;
                }
            }
        }
    }
}
void BasicScene::checkCollisionBetweenCylsAndWalls() {
    for (int i = 0; i < walls.size(); i++) {
        if (CheckCollision(&cylsTree[cyls.size() - 1], &wallsTree[i], cyls[cyls.size() - 1], walls[i])) {
            if (life > 0) {
                life--;
                Eigen::Vector3f t(0, 0, -1);
                t = root->GetRotation().inverse() * cyls[linksnum - 1]->GetRotation() * t;
                t = t.normalized() * 5;
                cyls[linksnum - 1]->Translate(t);
                cyls[0]->Translate(t);
            }
            else {
                PlaySound(TEXT("data/lose.wav"), NULL, SND_FILENAME | SND_ASYNC);
                previousState = currentState;
                currentState = die;
            }
        }
    }
}
bool BasicScene::CheckCollision(igl::AABB<Eigen::MatrixXd, 3>* tree1, igl::AABB<Eigen::MatrixXd, 3>* tree2, std::shared_ptr<cg3d::Model> model1, std::shared_ptr<cg3d::Model> model2) {
    if (tree1 == nullptr || tree2 == nullptr)
    {
        return false;
    }
    if (!isThereIntersection(tree1->m_box, tree2->m_box,model1,model2)) {
        return false;
    }
    if (!tree1->is_leaf() && !tree2->is_leaf()) {
        return CheckCollision(tree1->m_left, tree2->m_left,model1,model2) || CheckCollision(tree1->m_left, tree2->m_right, model1, model2)
            || CheckCollision(tree1->m_right, tree2->m_left, model1, model2) || CheckCollision(tree1->m_right, tree2->m_right, model1, model2);
    }
    else if (tree1->is_leaf() && !tree2->is_leaf()) {
        return CheckCollision(tree1, tree2->m_left, model1, model2) || CheckCollision(tree1, tree2->m_right, model1, model2);
    }
    else if (!tree1->is_leaf() && tree2->is_leaf()) {
        return CheckCollision(tree1->m_left, tree2, model1, model2) || CheckCollision(tree1->m_right, tree2, model1, model2);
    }
    return true;
}

//Checks the if two boxes intersect.
bool BasicScene::isThereIntersection(Eigen::AlignedBox<double, 3>& box1, Eigen::AlignedBox<double, 3>& box2, std::shared_ptr<cg3d::Model> model1, std::shared_ptr<cg3d::Model> model2) {
    //Getting the centers of the boxes.
    Eigen::Vector3d box1C = box1.center();
    Eigen::Vector3d box2C = box2.center();
    //Getting the size of the boxes.
    Eigen::Affine3f scaling1 = model1->GetScaling(model1->GetTransform());
    Eigen::Affine3f scaling2 = model2->GetScaling(model2->GetTransform());
    Eigen::Vector3d a = (box1.sizes() / 2);
    Eigen::Vector3d b = (box2.sizes() / 2);
    //Getting the rotation matrix for each object.
    Eigen::Matrix3d A = model1->GetRotation().cast<double>();
    Eigen::Matrix3d B = model2->GetRotation().cast<double>();
    Eigen::Matrix3d C = A.transpose() * B;
    //Calculating D matrix.
    Eigen::Vector4d C1 = Eigen::Vector4d(box1C(0), box1C(1), box1C(2), 1);
    Eigen::Vector4d C2 = Eigen::Vector4d(box2C(0), box2C(1), box2C(2), 1);
    Eigen::Vector4d D4 = model2->GetAggregatedTransform().cast<double>() * C2 - (model1->GetAggregatedTransform().cast<double>() * C1);
    Eigen::Vector3d D = Eigen::Vector3d(D4(0), D4(1), D4(2));

    //Checking the intersection of two boxes relativly with the tranformation and the rotation that occurred on the object.
    //Variables from the table(obb_sat.pdf).
    float R, R0, R1;

    //First Condition A0
    R0 = a(0);
    R1 = b(0) * abs(C(0, 0)) + b(1) * abs(C(0, 1)) + b(2) * abs(C(0, 2));
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
    R0 = a(0) * abs(C(0, 0)) + a(1) * abs(C(1, 0)) + a(2) * abs(C(2, 0));
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


void BasicScene::Level1(bool firstTime) {
    if (!firstTime) {
        cyls[linksnum - 1]->RemoveChild(camList[0]);
        root->RemoveChild(cyls[0]);
        root->RemoveChild(cyls[linksnum - 1]);
        root->RemoveChild(snake);
        for (int i = 0; i < spheres.size(); i++) {
            spheres[i].~shared_ptr();
            root2->RemoveChild(spheres[i]);
        }
        cyls.clear();
        spheres.clear();
        sphereTrees.clear();
        atenSpheres.clear();
        for (int i = 0; i < walls.size(); i++) {
            walls[i].~shared_ptr();
            root2->RemoveChild(walls[i]);
        }
        walls.clear();
        viewport->camera = Camera::Create("camera", camera->fov, camera->ratio, mynear, myfar);
        camera.~shared_ptr();
        camera = viewport->camera;
        camList[0].~shared_ptr();
        camList.clear();
    }
    an.resize(linksnum);
    for (int z = 0; z < an.size(); z++) {
        an[z] = Eigen::Quaterniond(Eigen::Matrix3d::Identity());
    }
    AddChild(root = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    AddChild(root2 = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    camList.push_back(Camera::Create("camera2", camera->fov, camera->ratio, mynear, myfar));
    auto program = std::make_shared<Program>("shaders/phongShader");
    auto program1 = std::make_shared<Program>("shaders/pickingShader");
    auto program2 = std::make_shared<Program>("shaders/basicShader");

    auto material{ std::make_shared<Material>("material", program) }; // empty material
    auto material1{ std::make_shared<Material>("material", program1) }; // empty material
    auto goldmaterial{ std::make_shared<Material>("material", program2) }; // empty material

    material->AddTexture(0, "textures/box0.bmp", 2);
    goldmaterial->AddTexture(0, "textures/gold.jpg", 2);
    auto sphereMesh{ IglLoader::MeshLoader2("sphere_igl", "data/sphere.obj") };
    auto cylMesh{ IglLoader::MeshLoader2("cyl_igl","data/zcylinder.obj") };
    auto snakeMesh{ IglLoader::MeshLoader2("cyl_igl","data/snake1.obj") };
    auto cubeMesh{ IglLoader::MeshLoader2("cube_igl","data/cube_old.obj") };


    //Map
    walls.push_back(Model::Create("wall0", cubeMesh, material));
    walls[0]->Scale(10, Axis::X);
    walls[0]->Translate(20, Axis::Z);
    walls.push_back(Model::Create("wall1", cubeMesh, material));
    walls[1]->Scale(10, Axis::X);
    walls[1]->Translate(40, Axis::Z);

    float scaleFactor = 1;
    cyls.push_back(Model::Create("cyl", cylMesh, material));
    cyls[0]->Scale(scaleFactor, Axis::Z);
    cyls[0]->SetCenter(Eigen::Vector3f(0, 0, -0.8f * scaleFactor));
    root->AddChild(cyls[0]);
    cylsTree.resize(linksnum);
    cylsTree[0].init(cyls[0]->GetMesh()->data[0].vertices, cyls[0]->GetMesh()->data[0].faces);
    cyls[0]->showFaces = true;
    cyls[0]->showWireframe = false;
    cyls[0]->showTextures = false;
    if (withSkinning)
        cyls[0]->isHidden = true;
    //cyls[0]->isHidden = true;
    //skinning
    C = Eigen::MatrixXd(linksnum + 1, 3);
    BE = Eigen::MatrixXi(linksnum, 2);
    C.row(0) << 0, 0, -1.6f * (linksnum / 2);
    BE.row(0) << 0, 1;
    for (int i = 1; i < linksnum; i++)
    {
        cyls.push_back(Model::Create("cyl", cylMesh, material));
        cyls[i]->Translate(1.6f * scaleFactor, Axis::Z);
        if (i == linksnum - 1)
            cyls[i]->Translate(1.6f * (linksnum - 2) * scaleFactor, Axis::Z);
        cyls[i]->SetCenter(Eigen::Vector3f(0, 0, -0.8f * scaleFactor));
        if (i != linksnum - 1) {
            cyls[i - 1]->AddChild(cyls[i]);
        }
        else {
            root->AddChild(cyls[i]);
        }
        cylsTree[i].init(cyls[i]->GetMesh()->data[0].vertices, cyls[i]->GetMesh()->data[0].faces);
        cyls[i]->showFaces = true;
        cyls[i]->isHidden = false;
        cyls[i]->showWireframe = false;
        if (withSkinning && i != linksnum-1)
            cyls[i]->isHidden = true;
        C.row(i) << 0, 0, 1.6f * (i - (linksnum / 2.0f));
        BE.row(i) << i, i + 1;
    }
    C.row(linksnum) << 0, 0, 1.6f * (linksnum /2);
    root->RotateByDegree(-90.0f, Axis::X);
    root2->RotateByDegree(-90.0f, Axis::X);
    snake = Model::Create("snake", snakeMesh, material);
    root->AddChild(snake);

    snake->Translate(1.6 * linksnum / 2, Axis::Z);
    snake->SetCenter(Eigen::Vector3f(0, 0, 1.6f * linksnum / 2));
    V = scaledVertices(snakeMesh->data[0].vertices, { 1, 1, (float)linksnum });
    F = snakeMesh->data[0].faces;
    U = V;
    Eigen::MatrixXd vN;
    igl::per_vertex_normals(U, F, vN);
    Mesh nextPose("snake", U, F, vN, snake->GetMesh()->data[0].textureCoords);
    snake->SetMeshList({ std::make_shared<Mesh>(nextPose) });
    igl::directed_edge_parents(BE, P);
    CalculateWeights();
    cyls[0]->Translate({ 0,0,0.8f * scaleFactor });
    root->RotateByDegree(0, Eigen::Vector3f(1, 0, 0));
    sphereTrees.resize(2);
    for (int i = 0; i < 2; i++) {
        if(i == 1)
            spheres.push_back(Model::Create("sphere" + i, sphereMesh, goldmaterial));
        else
            spheres.push_back(Model::Create("sphere" + i, sphereMesh, material));
        sphereTrees[i].init(spheres[i]->GetMesh()->data[0].vertices, spheres[i]->GetMesh()->data[0].faces);
        root2->AddChild(spheres[i]);
        spheres[i]->Translate({ 0,0,30.0f*i});
    }
    auto morphFunc = [](Model* model, cg3d::Visitor* visitor) {
        return model->meshIndex;//(model->GetMeshList())[0]->data.size()-1;
    };
    if (!withSkinning)
        snake->isHidden = true;
    cyls[linksnum - 1]->AddChild(camera);
    cyls[linksnum - 1]->AddChild(camList[0]); 
    camList[0]->RotateByDegree(-155, Axis::X);
    camList[0]->RotateByDegree(180, Axis::Z);
    camList[0]->Translate(5.0f, Axis::Y);
    camList[0]->Translate(-8.0f, Axis::Z);
    root->Translate(-20, Axis::Y);
    camera->Translate(50, Axis::Y);
    camera->RotateByDegree(-90, Axis::X);
    camera->RotateByDegree(180, Axis::Z);
    root2->AddChild(walls[0]);
    root2->AddChild(walls[1]);
    atenSpheres.resize(spheres.size());
    wallsTree.resize(walls.size());
    for (int i = 0; i < walls.size(); i++) {
        wallsTree[i].init(scaledVertices(walls[i]->GetMesh()->data[0].vertices, { 10.0f,1.0f,1.0f }), walls[i]->GetMesh()->data[0].faces);
    }
    for (int i = 0; i < atenSpheres.size(); i++)
        atenSpheres[i] = false;
    time = std::chrono::steady_clock::now();
}
void BasicScene::Level2() {
    cyls[linksnum - 1]->RemoveChild(camList[0]);
    root->RemoveChild(cyls[0]);
    root->RemoveChild(snake);
    root->RemoveChild(cyls[linksnum - 1]);
    cyls.clear();
    for (int i = 0; i < spheres.size(); i++) {
        root2->RemoveChild(spheres[i]);
    }
    spheres.clear();
    sphereTrees.clear();
    atenSpheres.clear();
    for (int i = 0; i < walls.size(); i++) {
        root2->RemoveChild(walls[i]);
    }
    walls.clear();
    viewport->camera = Camera::Create("camera", camera->fov, camera->ratio, mynear, myfar);
    camera.~shared_ptr();
    camera = viewport->camera;
    camList[0].~shared_ptr();
    camList.clear();
    an.resize(linksnum);
    for (int z = 0; z < an.size(); z++) {
        an[z] = Eigen::Quaterniond(Eigen::Matrix3d::Identity());
    }
    AddChild(root = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    AddChild(root2 = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    camList.push_back(Camera::Create("camera2", camera->fov, camera->ratio, mynear, myfar));
    auto program = std::make_shared<Program>("shaders/phongShader");
    auto program1 = std::make_shared<Program>("shaders/pickingShader");
    auto program2 = std::make_shared<Program>("shaders/basicShader");

    auto material = std::make_shared<Material>("material", program);
    auto bricksMaterial = std::make_shared<Material>("material", program2);
    auto goldMaterial = std::make_shared<Material>("material", program2);
    auto grassMaterial = std::make_shared<Material>("material", program2);
    auto snakeMaterial = std::make_shared<Material>("material", program2);

    material->AddTexture(0, "textures/box0.bmp", 2);
    bricksMaterial->AddTexture(0, "textures/bricks.jpg", 2);
    goldMaterial->AddTexture(0, "textures/gold.jpg", 2);
    grassMaterial->AddTexture(0, "textures/grass.bmp", 2);
    snakeMaterial->AddTexture(0, "textures/snake1.png", 2);
    auto sphereMesh{ IglLoader::MeshLoader2("sphere_igl", "data/sphere.obj") };
    auto cylMesh{ IglLoader::MeshLoader2("cyl_igl","data/zcylinder.obj") };
    auto snakeMesh{ IglLoader::MeshLoader2("cyl_igl","data/snake1.obj") };
    //auto cubeMesh{ IglLoader::MeshLoader2("cube_igl","data/cube_old.obj") };
    sphere1 = Model::Create("sphere", sphereMesh, material);
    //Map
    walls.push_back(Model::Create("wall0",Mesh::Cube(), bricksMaterial));
    walls[0]->Scale(80, Axis::X);
    walls[0]->Translate(29, Axis::Y);
    walls.push_back(Model::Create("wall1", Mesh::Cube(), bricksMaterial));
    walls[1]->Scale(80, Axis::X);
    walls[1]->Translate(-29, Axis::Y);
    walls.push_back(Model::Create("wall2", Mesh::Cube(), bricksMaterial));
    walls[2]->Scale(58, Axis::Y);
    walls[2]->Translate(-40, Axis::X);
    walls.push_back(Model::Create("wall3", Mesh::Cube(), bricksMaterial));
    walls[3]->Scale(58, Axis::Y);
    walls[3]->Translate(40, Axis::X);
    wallsTree.resize(walls.size());
    wallsTree[0].init(scaledVertices(walls[0]->GetMesh()->data[0].vertices, {80,1,1}), walls[0]->GetMesh()->data[0].faces);
    wallsTree[1].init(scaledVertices(walls[1]->GetMesh()->data[0].vertices, { 80,1,1 }),walls[1]->GetMesh()->data[0].faces);
    wallsTree[2].init(scaledVertices(walls[2]->GetMesh()->data[0].vertices, { 1,58,1 }),walls[2]->GetMesh()->data[0].faces);
    wallsTree[3].init(scaledVertices(walls[3]->GetMesh()->data[0].vertices, { 1,58,1 }),walls[3]->GetMesh()->data[0].faces);
    floor = Model::Create("wall4", Mesh::Cube(), grassMaterial);
    floor->Scale(80, Axis::X);
    floor->Scale(58, Axis::Y);
    floor->Translate(Eigen::Vector3f(0, 0, -1));
    float scaleFactor = 1;
    cyls.push_back(Model::Create("cyl", cylMesh, material));
    cyls[0]->Scale(scaleFactor, Axis::Z);
    cyls[0]->SetCenter(Eigen::Vector3f(0, 0, -0.8f * scaleFactor));
    root->AddChild(cyls[0]);
    cylsTree.resize(linksnum);
    cylsTree[0].init(cyls[0]->GetMesh()->data[0].vertices, cyls[0]->GetMesh()->data[0].faces);
    cyls[0]->showFaces = true;
    cyls[0]->showWireframe = false;
    cyls[0]->showTextures = false;
    if (withSkinning)
        cyls[0]->isHidden = true;
    //cyls[0]->isHidden = true;
    //skinning
    C = Eigen::MatrixXd(linksnum + 1, 3);
    BE = Eigen::MatrixXi(linksnum, 2);
    C.row(0) << 0, 0, -1.6 * (linksnum / 2.0f);
    BE.row(0) << 0, 1;
    for (int i = 1; i < linksnum; i++)
    {
        cyls.push_back(Model::Create("cyl", cylMesh, material));
        cyls[i]->Translate(1.6f * scaleFactor, Axis::Z);
        
        if (i == linksnum - 1)
            cyls[i]->Translate(1.6f * (linksnum - 2) * scaleFactor, Axis::Z);
        cyls[i]->SetCenter(Eigen::Vector3f(0, 0, -0.8f * scaleFactor));
        if (i != linksnum - 1) {
            cyls[i - 1]->AddChild(cyls[i]);
        }
        else {
            root->AddChild(cyls[i]);
        }
        cylsTree[i].init(cyls[i]->GetMesh()->data[0].vertices, cyls[i]->GetMesh()->data[0].faces);
        cyls[i]->showFaces = true;
        cyls[i]->showWireframe = false;
        if (withSkinning && i != linksnum - 1)
            cyls[i]->isHidden = true;
        C.row(i) << 0, 0, 1.6 * (i - (linksnum / 2.0f));
        BE.row(i) << i, i + 1;
    }
    C.row(linksnum) << 0, 0, 1.6 * ((linksnum / 2.0f));
    root->RotateByDegree(-90.0f, Axis::X);
    //root2->RotateByDegree(-90.0f, Axis::X);
    snake = Model::Create("snake", snakeMesh, snakeMaterial);
    root->AddChild(snake);

    snake->Translate(1.6 * linksnum / 2, Axis::Z);
    V = scaledVertices(snakeMesh->data[0].vertices, { 1, 1, (float)linksnum });
    F = snakeMesh->data[0].faces;
    U = V;
    Eigen::MatrixXd vN;
    igl::per_vertex_normals(U, snake->GetMesh()->data[0].faces, vN);
    Mesh nextPose("snake", U, F, vN, snake->GetMesh()->data[0].textureCoords);
    snake->SetMeshList({ std::make_shared<Mesh>(nextPose) });
    igl::directed_edge_parents(BE, P);
    CalculateWeights();
    cyls[0]->Translate({ 0,0,0.8f * scaleFactor });
    root->RotateByDegree(0, Eigen::Vector3f(1, 0, 0));
    sphereTrees.resize(sphereNum);
    for(int i=0;i<sphereNum;i++){
        if(i == sphereNum-1)
            spheres.push_back(Model::Create("sphere", sphereMesh, goldMaterial));
        else
            spheres.push_back(Model::Create("sphere", sphereMesh, material));
        sphereTrees[i].init(spheres[i]->GetMesh()->data[0].vertices, spheres[i]->GetMesh()->data[0].faces);
        root2->AddChild(spheres[i]);
        spheres[i]->Translate({GenerateRandomNumber(-38,38),GenerateRandomNumber(-27,27),0});
        atenSpheres.push_back(false);
    }


    sphereTree.init(sphere1->GetMesh()->data[0].vertices, sphere1->GetMesh()->data[0].faces);
    auto morphFunc = [](Model* model, cg3d::Visitor* visitor) {
        return model->meshIndex;//(model->GetMeshList())[0]->data.size()-1;
    };
    if (!withSkinning)
        snake->isHidden = true;
    root->Translate(-20, Axis::Y);
    sphere1->showWireframe = true;
    //camera->RotateByDegree(-90, Axis::X);
    camera->Translate(50, Axis::Z);
    cyls[linksnum - 1]->AddChild(camList[0]);
    camList[0]->RotateByDegree(155, Axis::X);
    //camList[0]->RotateByDegree(180, Axis::Z);
    camList[0]->Translate(-5.0f, Axis::Y);
    camList[0]->Translate(-8.0f, Axis::Z);
    root2->AddChild(walls[0]);
    root2->AddChild(walls[1]);
    root2->AddChild(walls[2]);
    root2->AddChild(walls[3]);
    root2->AddChild(floor);
    root2->Translate(-20, Axis::Z);
    root->Translate(-20, Axis::Z);
    time = std::chrono::steady_clock::now();
}


void BasicScene::Level3(int firstTime) {
    if (!firstTime) {
        for (int i = 0; i < bombs.size(); i++) {
            root2->RemoveChild(bombs[i]);
        }
        bombs.clear();
        bombsTree.clear();
    }
    cyls[linksnum - 1]->RemoveChild(camList[0]);
    root->RemoveChild(cyls[0]);
    root->RemoveChild(snake);
    root->RemoveChild(cyls[linksnum - 1]);
    cyls.clear();
    for (int i = 0; i < spheres.size(); i++) {
        root2->RemoveChild(spheres[i]);
    }
    spheres.clear();
    sphereTrees.clear();
    atenSpheres.clear();
    for (int i = 0; i < walls.size(); i++) {
        root2->RemoveChild(walls[i]);
    }
    walls.clear();
    viewport->camera = Camera::Create("camera", camera->fov, camera->ratio, mynear, myfar);
    camera.~shared_ptr();
    camera = viewport->camera;
    camList[0].~shared_ptr();
    camList.clear();
    an.resize(linksnum);
    for (int z = 0; z < an.size(); z++) {
        an[z] = Eigen::Quaterniond(Eigen::Matrix3d::Identity());
    }
    AddChild(root = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    AddChild(root2 = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    camList.push_back(Camera::Create("camera2", camera->fov, camera->ratio, mynear, myfar));
    auto program = std::make_shared<Program>("shaders/phongShader");
    auto program1 = std::make_shared<Program>("shaders/pickingShader");
    auto program2 = std::make_shared<Program>("shaders/basicShader");

    auto material = std::make_shared<Material>("material", program);
    auto bricksMaterial = std::make_shared<Material>("material", program2);
    auto goldMaterial = std::make_shared<Material>("material", program2);
    auto grassMaterial = std::make_shared<Material>("material", program2);
    auto snakeMaterial = std::make_shared<Material>("material", program2);
    auto bombMaterial = std::make_shared<Material>("material", program2);

    material->AddTexture(0, "textures/box0.bmp", 2);
    bricksMaterial->AddTexture(0, "textures/bricks.jpg", 2);
    goldMaterial->AddTexture(0, "textures/gold.jpg", 2);
    grassMaterial->AddTexture(0, "textures/grass.bmp", 2);
    snakeMaterial->AddTexture(0, "textures/snake1.png", 2);
    bombMaterial->AddTexture(0, "textures/carbon.jpg", 2);

    auto sphereMesh{ IglLoader::MeshLoader2("sphere_igl", "data/sphere.obj") };
    auto cylMesh{ IglLoader::MeshLoader2("cyl_igl","data/zcylinder.obj") };
    auto snakeMesh{ IglLoader::MeshLoader2("cyl_igl","data/snake1.obj") };
    //auto cubeMesh{ IglLoader::MeshLoader2("cube_igl","data/cube_old.obj") };
    sphere1 = Model::Create("sphere", sphereMesh, material);
    //Map
    walls.push_back(Model::Create("wall0", Mesh::Cube(), bricksMaterial));
    walls[0]->Scale(80, Axis::X);
    walls[0]->Translate(29, Axis::Y);
    walls.push_back(Model::Create("wall1", Mesh::Cube(), bricksMaterial));
    walls[1]->Scale(80, Axis::X);
    walls[1]->Translate(-29, Axis::Y);
    walls.push_back(Model::Create("wall2", Mesh::Cube(), bricksMaterial));
    walls[2]->Scale(58, Axis::Y);
    walls[2]->Translate(-40, Axis::X);
    walls.push_back(Model::Create("wall3", Mesh::Cube(), bricksMaterial));
    walls[3]->Scale(58, Axis::Y);
    walls[3]->Translate(40, Axis::X);
    walls.push_back(Model::Create("wall5", Mesh::Cube(), bricksMaterial));
    walls[4]->Scale(30, Axis::Y);
    walls[4]->Scale(0.5f, Axis::X);
    walls.push_back(Model::Create("wall6", Mesh::Cube(), bricksMaterial));
    walls[5]->Scale(30, Axis::X);
    walls[5]->Scale(0.5f, Axis::Y);
    wallsTree.resize(walls.size());
    wallsTree[0].init(scaledVertices(walls[0]->GetMesh()->data[0].vertices, { 80,1,1 }), walls[0]->GetMesh()->data[0].faces);
    wallsTree[1].init(scaledVertices(walls[1]->GetMesh()->data[0].vertices, { 80,1,1 }), walls[1]->GetMesh()->data[0].faces);
    wallsTree[2].init(scaledVertices(walls[2]->GetMesh()->data[0].vertices, { 1,58,1 }), walls[2]->GetMesh()->data[0].faces);
    wallsTree[3].init(scaledVertices(walls[3]->GetMesh()->data[0].vertices, { 1,58,1 }), walls[3]->GetMesh()->data[0].faces);
    wallsTree[4].init(scaledVertices(walls[4]->GetMesh()->data[0].vertices, { 0.5,30,1 }), walls[4]->GetMesh()->data[0].faces);
    wallsTree[5].init(scaledVertices(walls[5]->GetMesh()->data[0].vertices, { 30,0.5,1 }), walls[5]->GetMesh()->data[0].faces);
    floor = Model::Create("wall4", Mesh::Cube(), grassMaterial);
    floor->Scale(80, Axis::X);
    floor->Scale(58, Axis::Y);
    floor->Translate(Eigen::Vector3f(0, 0, -1));
    float scaleFactor = 1;
    cyls.push_back(Model::Create("cyl", cylMesh, material));
    cyls[0]->Scale(scaleFactor, Axis::Z);
    cyls[0]->SetCenter(Eigen::Vector3f(0, 0, -0.8f * scaleFactor));
    root->AddChild(cyls[0]);
    cylsTree.resize(linksnum);
    cylsTree[0].init(cyls[0]->GetMesh()->data[0].vertices, cyls[0]->GetMesh()->data[0].faces);
    cyls[0]->showFaces = true;
    cyls[0]->showWireframe = false;
    cyls[0]->showTextures = false;
    if (withSkinning)
        cyls[0]->isHidden = true;
    //cyls[0]->isHidden = true;
    //skinning
    C = Eigen::MatrixXd(linksnum + 1, 3);
    BE = Eigen::MatrixXi(linksnum, 2);
    C.row(0) << 0, 0, -1.6 * (linksnum / 2.0f);
    BE.row(0) << 0, 1;
    for (int i = 1; i < linksnum; i++)
    {
        cyls.push_back(Model::Create("cyl", cylMesh, material));
        cyls[i]->Translate(1.6f * scaleFactor, Axis::Z);
        
        if (i == linksnum - 1)
            cyls[i]->Translate(1.6f * (linksnum - 2) * scaleFactor, Axis::Z);
        cyls[i]->SetCenter(Eigen::Vector3f(0, 0, -0.8f * scaleFactor));
        if (i != linksnum - 1) {
            cyls[i - 1]->AddChild(cyls[i]);
        }
        else {
            root->AddChild(cyls[i]);
        }
        cylsTree[i].init(cyls[i]->GetMesh()->data[0].vertices, cyls[i]->GetMesh()->data[0].faces);
        cyls[i]->showFaces = true;
        cyls[i]->showWireframe = false;
        if (withSkinning && i != linksnum - 1)
            cyls[i]->isHidden = true;
        C.row(i) << 0, 0, 1.6 * (i - (linksnum / 2.0f));
        BE.row(i) << i, i + 1;
    }
    C.row(linksnum) << 0, 0, 1.6 * ((linksnum / 2.0f));
    root->RotateByDegree(-90.0f, Axis::X);
    //root2->RotateByDegree(-90.0f, Axis::X);
    snake = Model::Create("snake", snakeMesh, snakeMaterial);
    root->AddChild(snake);

    snake->Translate(1.6 * linksnum / 2, Axis::Z);
    V = scaledVertices(snakeMesh->data[0].vertices, { 1, 1, (float)linksnum });
    F = snakeMesh->data[0].faces;
    U = V;
    Eigen::MatrixXd vN;
    igl::per_vertex_normals(U, snake->GetMesh()->data[0].faces, vN);
    Mesh nextPose("snake", U, F, vN, snake->GetMesh()->data[0].textureCoords);
    snake->SetMeshList({ std::make_shared<Mesh>(nextPose) });
    igl::directed_edge_parents(BE, P);
    CalculateWeights();
    cyls[0]->Translate({ 0,0,0.8f * scaleFactor });
    root->RotateByDegree(0, Eigen::Vector3f(1, 0, 0));
    sphereTrees.resize(sphereNum);
    for (int i = 0; i < sphereNum; i++) {
        if (i == sphereNum - 1)
            spheres.push_back(Model::Create("sphere", sphereMesh, goldMaterial));
        else
            spheres.push_back(Model::Create("sphere", sphereMesh, material));
        sphereTrees[i].init(spheres[i]->GetMesh()->data[0].vertices, spheres[i]->GetMesh()->data[0].faces);
        root2->AddChild(spheres[i]);
        spheres[i]->Translate({ GenerateRandomNumber(-38,38),GenerateRandomNumber(-27,27),0 });
        atenSpheres.push_back(false);
    }    

    sphereTree.init(sphere1->GetMesh()->data[0].vertices, sphere1->GetMesh()->data[0].faces);
    auto morphFunc = [](Model* model, cg3d::Visitor* visitor) {
        return model->meshIndex;//(model->GetMeshList())[0]->data.size()-1;
    };
    bombs.push_back(Model::Create("b1", sphereMesh, bombMaterial));
    bombs.push_back(Model::Create("b2", sphereMesh, bombMaterial));
    bombs.push_back(Model::Create("b3", sphereMesh, bombMaterial));
    bombs.push_back(Model::Create("b4", sphereMesh, bombMaterial));
    bombsTree.resize(4);
    for (int i = 0; i < 4; i++) {
        bombsTree[i].init(bombs[i]->GetMesh()->data[0].vertices, bombs[i]->GetMesh()->data[0].faces);
        root2->AddChild(bombs[i]);
    }
    bombs[0]->Translate({40,14,0});
    bombs[1]->Translate({40,-14,0});
    bombs[2]->Translate({-40,14,0});
    bombs[3]->Translate({-40,-14,0});
    if (!withSkinning)
        snake->isHidden = true;
    root->Translate(-20, Axis::Y);
    sphere1->showWireframe = true;
    //camera->RotateByDegree(-90, Axis::X);
    camera->Translate(50, Axis::Z);
    cyls[linksnum - 1]->AddChild(camList[0]);
    camList[0]->RotateByDegree(155, Axis::X);
    //camList[0]->RotateByDegree(180, Axis::Z);
    camList[0]->Translate(-5.0f, Axis::Y);
    camList[0]->Translate(-8.0f, Axis::Z);
    root2->AddChild(walls[0]);
    root2->AddChild(walls[1]);
    root2->AddChild(walls[2]);
    root2->AddChild(walls[3]);
    root2->AddChild(walls[4]);
    root2->AddChild(walls[5]);
    root2->AddChild(floor);
    root2->Translate(-20, Axis::Z);
    root2->Translate(1, Axis::X);
    root->Translate(-20, Axis::Z);
    time = std::chrono::steady_clock::now();
    std::cout << "HI HOW ARE YOU" << std::endl;
}