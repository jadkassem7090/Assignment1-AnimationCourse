#pragma once

#include "Scene.h"
#include <utility>
#include "../../build2/tutorial/mysimplification/oneField.h"
#include "../../build2/tutorial/mysimplification/myDataStructure.h"

using namespace cg3d;
class BasicScene : public cg3d::Scene
{
public:
    explicit BasicScene(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    void KeyCallback(Viewport* viewport, int x, int y, int key, int scancode, int action, int mods);
    bool collapse(std::shared_ptr<cg3d::Mesh> mesh);
    bool newcollapse(std::shared_ptr<cg3d::Mesh> mesh);
private:
    std::shared_ptr<cg3d::Model> cube,cyl,autoCube, sphere1,autoSphere, autoCyl;
    oneField oneF = oneField();
};
