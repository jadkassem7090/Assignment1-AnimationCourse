#include "BasicScene.h"
#include <utility>
#include "ObjLoader.h"
#include "IglMeshLoader.h"


using namespace cg3d;

void BasicScene::Init(float fov, int width, int height, float near, float far)
{
    camera = Camera::Create("camera", fov, float(width) / float(height), near, far);
    auto program = std::make_shared<Program>("shaders/basicShader");
    auto material = std::make_shared<Material>("material", program); // empty material
    auto daylight{ std::make_shared<Material>("daylight", "shaders/cubemapShader") };
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{ Model::Create("background", Mesh::Cube(), daylight) };

    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();

    material->AddTexture(0, "textures/box0.bmp", 2);
    auto sphereMesh{ IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj") };
    auto cubeMesh{ IglLoader::MeshFromFiles("cube_igl", "data/cube.off") };

    //Constructing the model
    sphere1 = Model::Create("sphere", sphereMesh, material);
    cube = Model::Create("cube", cubeMesh, material);
    
    camera->Translate(20, Axis::Z);
    // object setup
    sphere1->Scale(3.0f);
    sphere1->showWireframe = true;
    sphere1->Translate({ 8, 0, 0 });
    AddChild(sphere1);
    cube->Scale(3.0f);
    cube->showWireframe = true;
    cube->Translate({ -8, 0, 0 });
    AddChild(cube);
}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    sphere1->Rotate(0.001f, Axis::XYZ);
    cube->Rotate(0.001f, Axis::XYZ);
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
            }
            break;
        case GLFW_KEY_LEFT:
            camera->RotateInSystem(system, 0.1f, Axis::Y);
            break;
        case GLFW_KEY_RIGHT:
            camera->RotateInSystem(system, -0.1f, Axis::Y);
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
                if (oneF.getMyEmp() == true) {
                    if (newcollapse(pickedModel->GetMesh())) {
                        pickedModel->meshIndex = pickedModel->GetMesh()->data.size() - 1;
                    }
                }
                else {
                    if (collapse(pickedModel->GetMesh())) {
                        pickedModel->meshIndex = pickedModel->GetMesh()->data.size() - 1;
                    }
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
bool BasicScene::collapse(std::shared_ptr<cg3d::Mesh> mesh) {
    cout << "collapse " << mesh->dataS.Q.size() << endl;
    const int max_iter = std::ceil(0.1 * mesh->dataS.Q.size());
    bool collapsed = mesh->dataS.simplify(max_iter);
    if (collapsed) {
        Eigen::MatrixXd VN;
        igl::per_vertex_normals(mesh->dataS.V, mesh->dataS.F, VN);
        Eigen::MatrixXd TC = Eigen::MatrixXd::Zero(mesh->dataS.V.rows(), 2);
        mesh->data.push_back(MeshData{ mesh->dataS.V, mesh->dataS.F, VN, TC });
        cout << "Mesh Vector: " << mesh->data.size() << endl;
        pickedModel->SetMeshList({ pickedModel->GetMesh() });
    }
    return collapsed;
}
bool BasicScene::newcollapse(std::shared_ptr<cg3d::Mesh> mesh) {
    cout << "newcollapse " << mesh->dataS.myQ.size() << endl;
    const int max_iter = std::ceil(0.1 * mesh->dataS.myQ.size());
    bool collapsed = mesh->dataS.newsimplify(max_iter);
    if (collapsed) {
        Eigen::MatrixXd VN;
        igl::per_vertex_normals(mesh->dataS.V, mesh->dataS.F, VN);
        Eigen::MatrixXd TC = Eigen::MatrixXd::Zero(mesh->dataS.V.rows(), 2);
        mesh->data.push_back(MeshData{ mesh->dataS.V, mesh->dataS.F, VN, TC });
        cout << "Mesh Vector: " << mesh->data.size() << endl;
        pickedModel->SetMeshList({ pickedModel->GetMesh() });
    }
    return collapsed;
}
