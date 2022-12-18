#pragma once

#include "Scene.h"
#include "AutoMorphingModel.h"
#include "igl/AABB.h"
#include <utility>
#include <iostream>
#include "igl/per_vertex_normals.h"

using namespace cg3d;
class BasicScene : public cg3d::Scene
{
public:
    explicit BasicScene(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    void KeyCallback(Viewport* viewport, int x, int y, int key, int scancode, int action, int mods);
    bool CheckCollision(igl::AABB<Eigen::MatrixXd, 3>* tree1, igl::AABB<Eigen::MatrixXd, 3>* tree2);
    bool isThereIntersection(Eigen::AlignedBox<double, 3>& box1, Eigen::AlignedBox<double, 3>& box2);
    void makeBouningBox(Eigen::AlignedBox<double, 3>& aligned_box, int num);

    //simplify
    bool collapse(std::shared_ptr<cg3d::Mesh> mesh);


private:
    std::shared_ptr<Movable> root;
    std::shared_ptr<cg3d::Model> obj1, obj2, boundingBox1, boundingBox2, hit1, hit2, object1, object2;
    std::shared_ptr<AutoMorphingModel> autoSphere, autoCube, autoObj1, autoObj2;
    float speed = 0;
    igl::AABB<Eigen::MatrixXd, 3> Tree1, Tree2;
    bool tellMeCollapsed = true;
    std::shared_ptr<Program> program;
};
