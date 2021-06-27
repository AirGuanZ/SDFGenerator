#include <agz-utils/mesh.h>

#include "camera.h"
#include "sdf.h"
#include "sdf_renderer.h"

class DirectX11SDFDemo : public Demo
{
public:

    using Demo::Demo;

private:

    SDF sdf_;

    SDFRenderer sdfRenderer_;

    Camera camera_;

    std::string objFilename_;
    int         signRayCount_ = 3;
    int         sdfRes_       = 64;

    SDFGenerator sdfGen_;

    int   maxTraceSteps_ = 128;
    float absThreshold_  = 0.01f;

    ImGui::FileBrowser fileBrowser_;

    void initialize() override
    {
        window_->setMaximized();

        sdfRenderer_.initilalize();

        objFilename_ = "./asset/bunny.obj";
        loadMesh();

        camera_.setPosition(Float3(-4, 0, 0));
        camera_.setPerspective(60.0f, 0.1f, 100.0f);

        fileBrowser_.SetTitle("Select Obj");

        mouse_->setCursorLock(
            true, window_->getClientWidth() / 2, window_->getClientHeight() / 2);
        mouse_->showCursor(false);
        window_->doEvents();
    }

    void frame() override
    {
        if(keyboard_->isDown(KEY_ESCAPE))
            window_->setCloseFlag(true);

        if(keyboard_->isDown(KEY_LCTRL))
        {
            mouse_->showCursor(!mouse_->isVisible());
            mouse_->setCursorLock(
                !mouse_->isLocked(), mouse_->getLockX(), mouse_->getLockY());
        }

        if(ImGui::Begin("Settings", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::InputInt("Max Steps", &maxTraceSteps_);
            ImGui::InputFloat("Abs Threshold", &absThreshold_);

            if(ImGui::Button("Select Scene"))
                fileBrowser_.Open();

            ImGui::InputInt("SDF Res", &sdfRes_);
            sdfRes_ = (std::max)(sdfRes_, 1);

            ImGui::InputInt("Sign Ray Count", &signRayCount_);
            signRayCount_ = (std::max)(signRayCount_, 1);

            if(ImGui::Button("Regenerate"))
                loadMesh();
        }
        ImGui::End();

        fileBrowser_.Display();
        if(fileBrowser_.HasSelected())
        {
            objFilename_ = fileBrowser_.GetSelected().string();
            fileBrowser_.ClearSelected();

            loadMesh();
        }
        
        camera_.setWOverH(window_->getClientWOverH());
        if(!mouse_->isVisible())
        {
            camera_.update({
                .front      = keyboard_->isPressed('W'),
                .left       = keyboard_->isPressed('A'),
                .right      = keyboard_->isPressed('D'),
                .back       = keyboard_->isPressed('S'),
                .up         = keyboard_->isPressed(KEY_SPACE),
                .down       = keyboard_->isPressed(KEY_LSHIFT),
                .cursorRelX = static_cast<float>(mouse_->getRelativePositionX()),
                .cursorRelY = static_cast<float>(mouse_->getRelativePositionY())
            });
        }
        camera_.recalculateMatrics();

        window_->clearDefaultDepth(1);
        window_->clearDefaultRenderTarget({ 0, 1, 1, 0 });

        sdfRenderer_.setCamera(camera_);
        sdfRenderer_.setSDF(sdf_.lower, sdf_.upper, sdf_.srv);
        sdfRenderer_.setTracer(maxTraceSteps_, absThreshold_);
        sdfRenderer_.render();
    }

    void loadMesh()
    {
        const auto tris = agz::mesh::load_from_file(objFilename_);

        agz::math::aabb3f bbox = { Float3(InfFlt), Float3(-InfFlt) };
        for(auto &t : tris)
        {
            for(auto &v : t.vertices)
                bbox |= v.position;
        }

        const float maxExtent = (bbox.high - bbox.low).max_elem();
        const Float3 offset = -0.5f * (bbox.high + bbox.low);
        const float scale = 2 / maxExtent;
        
        std::vector<Float3> vertices, normals;
        for(auto &t : tris)
        {
            for(auto &v : t.vertices)
            {
                vertices.push_back(scale * (v.position + offset));
                normals.push_back(v.normal);
            }
        }

        sdfGen_.setSignRayCount(signRayCount_);
        sdf_ = sdfGen_.generateGPU(
            vertices.data(), normals.data(), tris.size(),
            Float3(-1.2f), Float3(1.2f), Int3(sdfRes_));
    }
};

int main()
{
    DirectX11SDFDemo(WindowDesc{
        .clientSize     = { 640, 480 },
        .title          = L"DirectX11-SDF",
        .disableTimeout = true,
        .sampleCount    = 4
    }).run();
}
