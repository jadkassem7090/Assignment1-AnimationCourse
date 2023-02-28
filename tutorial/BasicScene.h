#pragma once
#include "AutoMorphingModel.h"
#include "Scene.h"
#include "SceneWithImGui.h"
#include <AABB.h>
#include <memory>
#include <chrono>
#include <utility>


typedef std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > RotationList;
class BasicScene : public cg3d::SceneWithImGui
{
public:
    explicit BasicScene(std::string name, cg3d::Display* display) : SceneWithImGui(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    void MouseCallback(cg3d::Viewport* viewport, int x, int y, int button, int action, int mods, int buttonState[]) override;
    void ScrollCallback(cg3d::Viewport* viewport, int x, int y, int xoffset, int yoffset, bool dragging, int buttonState[]) override;
    void CursorPosCallback(cg3d::Viewport* viewport, int x, int y, bool dragging, int* buttonState)  override;
    void KeyCallback(cg3d::Viewport* viewport, int x, int y, int key, int scancode, int action, int mods) override;
    void AddViewportCallback(cg3d::Viewport* _viewport) override;
    Eigen::Vector3f GetArmTip();
    void nextLink();
    void CCD(Eigen::Vector3f Goal, int i);
    Eigen::Vector3f getTipLinkPosition(int index);
    void FabricAlgo(Eigen::Vector3f Goal, int index);
    void Help(int i, Eigen::Vector3f D);
    Eigen::Vector3f getGoalPosition();
    Eigen::Vector3f getLinkPosition(int index);
    void TextCentered(std::string text, float margin = 0);
    void cursorCentered(std::string text, float margin = 0);
    float GenerateRandomNumber(int a, int b);
    bool CheckCollision(igl::AABB<Eigen::MatrixXd, 3>* tree1, igl::AABB<Eigen::MatrixXd, 3>* tree2, std::shared_ptr<cg3d::Model> model1, std::shared_ptr<cg3d::Model> model2);
    bool isThereIntersection(Eigen::AlignedBox<double, 3>& box1, Eigen::AlignedBox<double, 3>& box2, std::shared_ptr<cg3d::Model> model1, std::shared_ptr<cg3d::Model> model2);
    void checkCollisionBetweenCylsAndShpere();
    void checkCollisionBetweenCylsAndBombs();
    void checkCollisionBetweenCylsAndWalls();
    Eigen::MatrixXd scaledVertices(const Eigen::MatrixXd& vertices, const Eigen::Vector3d& scale);
    std::vector<float> getWeightsandIndecies(Eigen::Vector3f v);
    void ApplySkinning();
    void CalculateWeights();
    void switchCamera();
    void Level1(bool firstTime);
    void Level2();
    void Level3(int firstTime);
    enum GameState {
        level1,
        level2,
        level3,
        paused,
        startmenu,
        betweenLevels,
        settings,
        die
    };
private:
    std::vector<std::shared_ptr<cg3d::Camera>> camList;
    std::vector<bool> atenSpheres;
    GameState currentState = startmenu;
    GameState previousState = startmenu;
    int score = 0;
    int thisLevelScore = 0;
    int life = 0;
    void BuildImGui() override;
    cg3d::Viewport* viewport = nullptr;
    std::shared_ptr<Movable> root, root2;
    std::shared_ptr<cg3d::Model> sphere1, cube, snake,rock, floor;
    std::shared_ptr<cg3d::AutoMorphingModel> autoCube;
    std::vector<std::shared_ptr<cg3d::Model>> cyls, spheres, walls, bombs;
    std::vector<igl::AABB<Eigen::MatrixXd, 3>> cylsTree,sphereTrees, wallsTree,bombsTree;
    igl::AABB<Eigen::MatrixXd, 3> sphereTree;
    int pickedIndex = 0;
    RotationList an;
    int tipIndex = 0;
    int sphereNum = 6;
    int frameC = 0;
    int linksnum = 5;
    std::chrono::steady_clock::time_point time;
    int timeBeforePause = 120;
    int lap = 120;
    float mynear = 0;
    float myfar = 0;
    float delta = 0.05f;
    float pi = 3.14159265358979323846;
    bool runCCD = true;
    float dwidth, dheight;
    bool arrived = false;
    bool music = true;
    bool effects = true;
    bool aten = false;
    bool withSkinning = false;
    bool direc = 1;
    //Skinning
    Eigen::MatrixXi F, EI, BE;
    Eigen::VectorXi EQ, P;
    Eigen::MatrixXd V, C, eV, W, U;
};
