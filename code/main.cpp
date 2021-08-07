#include <aOpenGL.h>
#include <iostream>



class MyApp : public agl::App
{
public:
    agl::spModel            model;
    agl::Motion             motion_a;
    agl::Motion             motion_b;
    std::vector<agl::Pose>  motion_a_poses;
    std::vector<agl::Pose>  motion_b_poses;
    Vec3 cam_offset;
    int a_nof;
    int b_nof;

    void start() override
    {
        const char* model_path      = "../data/fbx/ybot/model/ybot.fbx";
        const char* motion_path_a   = "../data/fbx/ybot/motion/Walking.fbx";
        const char* motion_path_b   = "../data/fbx/ybot/motion/Sneak Walk.fbx";

        agl::FBX model_fbx(model_path);
        agl::FBX motion_a_fbx(motion_path_a);
        agl::FBX motion_b_fbx(motion_path_b);

        model       = model_fbx.model();
        motion_a    = motion_a_fbx.motion(model).at(0);
        motion_b    = motion_b_fbx.motion(model).at(0);
        a_nof       = motion_a.poses.size();
        b_nof       = motion_b.poses.size();

        cam_offset = 2.0f * Vec3(0.0f, 3.0f, 3.0f);
    

        // Print motion name -------------------------------------------- //
        {
            std::cout << motion_path_a << " imported" << std::endl;
            std::cout << "\tname : " << motion_a.name << std::endl;
            std::cout << "\tnumber of frames : " << motion_a.poses.size() << std::endl;
            std::cout << std::endl;

            std::cout << motion_path_b << " imported" << std::endl;
            std::cout << "\tname : " << motion_b.name << std::endl;
            std::cout << "\tnumber of frames : " << motion_b.poses.size() << std::endl;
        }

    }

    int frame = 0;
    void update() override
    {
        if(frame >= 0 && frame < a_nof)
            model->set_pose(motion_a.poses.at(frame));
        else if(frame >= a_nof && frame < a_nof + b_nof)
            model->set_pose(motion_b.poses.at(frame-a_nof));
        
        // cam_follow_root(model->root()->world_pos(), cam_offset);

        // Update frame count
        frame = (frame + 1) % (a_nof + b_nof);
        // frame = (frame + 1) % (a_nof);
    }

    void render() override
    {
        agl::Render::plane()
            ->scale(15.0f)
            ->floor_grid(true)
            ->color(0.2f, 0.2f, 0.2f)
            ->draw();

        agl::Render::model(model)->draw();
    }

    void key_callback(char key, int action) override
    {
        if(action != GLFW_PRESS)
            return;
        
        if(key == '1')
            this->capture(true);
        if(key == '2')
            this->capture(false);
    }

    void cam_follow_root(Vec3 root_pos, Vec3 cam_offset)
    {
        Vec3 focus = root_pos;
        focus.y() = 1.0f;

        Vec3 pos = root_pos;
        pos = pos + cam_offset;
        pos.y() = 2.0f;

        camera().set_position(pos);
        camera().set_focus(focus);

    }
    
};

int main(int argc, char* argv[])
{
    MyApp app;
    agl::AppManager::start(&app);
    return 0;
} 