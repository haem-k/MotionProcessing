#include <aOpenGL.h>
#include <iostream>

static std::vector<agl::Pose> stitch(
    const std::vector<agl::Pose>& poses_a,
    const std::vector<agl::Pose>& poses_b)
{
    std::vector<agl::Pose> new_poses = poses_a;

    Quat a_last_root_orient = poses_a.back().local_rotations.at(0);
    // std::cout << last_root_orient << std::endl;
    Vec3 a_last_root_pos = poses_a.back().root_position;
    std::cout << "a_last_root_pos" << std::endl;
    std::cout << a_last_root_pos << std::endl;
    a_last_root_pos.y() = 0.0f;

    Quat inverse = poses_b.at(0).local_rotations.at(0).inverse();
    Vec3 b_first_root = poses_b.at(0).root_position;
    std::cout << "b_first_root" << std::endl;
    std::cout << b_first_root << std::endl;


    for(int i = 0; i < poses_b.size(); i++)
    {
        // Compute local transform
        agl::Pose new_pose = poses_b.at(i);
        new_pose.root_position = a_last_root_orient * inverse * new_pose.root_position;
        new_pose.root_position += a_last_root_pos;
        new_pose.local_rotations.at(0) = a_last_root_orient * inverse * new_pose.local_rotations.at(0);


        new_poses.push_back(new_pose);
    }
    return new_poses;
}

class MyApp : public agl::App
{
public:
    agl::spModel            model_1a;
    agl::spModel            model_1b;
    agl::spModel            model_2a;
    agl::spModel            model_2b;

    agl::Motion             motion_1a;
    agl::Motion             motion_1b;
    agl::Motion             motion_2a;
    agl::Motion             motion_2b;

    Vec3 pos1a;
    Vec3 pos1b;
    Vec3 pos2a;
    Vec3 pos2b;

    Vec3 cam_offset;
    int a_nof;
    int b_nof;
    int stitched_nof;

    void start() override
    {
        const char* model_path      = "../data/fbx/ybot/model/ybot.fbx";
        const char* motion_path_1a   = "../data/fbx/ybot/motion/Walking.fbx";
        const char* motion_path_1b   = "../data/fbx/ybot/motion/Sneak Walk.fbx";
        const char* motion_path_2a   = "../data/fbx/ybot/motion/Running To Turn.fbx";
        const char* motion_path_2b   = "../data/fbx/ybot/motion/Running.fbx";

        agl::FBX model_fbx(model_path);
        agl::FBX motion_1a_fbx(motion_path_1a);
        agl::FBX motion_1b_fbx(motion_path_1b);
        agl::FBX motion_2a_fbx(motion_path_2a);
        agl::FBX motion_2b_fbx(motion_path_2b);

        model_1a       = model_fbx.model();
        model_1b       = model_fbx.model();
        model_2a       = model_fbx.model();
        model_2b       = model_fbx.model();

        motion_1a    = motion_1a_fbx.motion(model_1a).at(0);
        motion_1b    = motion_1b_fbx.motion(model_1b).at(0);
        motion_2a    = motion_2a_fbx.motion(model_2a).at(0);
        motion_2b    = motion_2b_fbx.motion(model_2b).at(0);


        pos1a = motion_1a.poses.at(0).root_position + Vec3(-1.0f, 0, 0);
        pos1b = motion_1b.poses.at(0).root_position + Vec3(-1.0f, 0, 0);
        pos2a = motion_2a.poses.at(0).root_position + Vec3(1.0f, 0, 0);
        pos2b = motion_2b.poses.at(0).root_position + Vec3(1.0f, 0, 0);

        // a_nof       = motion_a.poses.size();
        // b_nof       = motion_b.poses.size();

        cam_offset = 2.0f * Vec3(0.0f, 3.0f, 3.0f);
    

        // Print motion name -------------------------------------------- //
        // {
        //     std::cout << motion_path_a << " imported" << std::endl;
        //     std::cout << "\tname : " << motion_a.name << std::endl;
        //     std::cout << "\tnumber of frames : " << motion_a.poses.size() << std::endl;
        //     std::cout << std::endl;

        //     std::cout << motion_path_b << " imported" << std::endl;
        //     std::cout << "\tname : " << motion_b.name << std::endl;
        //     std::cout << "\tnumber of frames : " << motion_b.poses.size() << std::endl;
        // }


    }

    int frame = 0;
    void update() override
    {

        
        // if(frame >= 0 && frame < a_nof)
        //     model->set_pose(motion_a.poses.at(frame));
        // else if(frame >= a_nof && frame < a_nof + b_nof)
        //     model->set_pose(motion_b.poses.at(frame-a_nof));
        
        // frame = (frame + 1) % (a_nof + b_nof);
        
        
        agl::Pose firstPose = motion_1a.poses.at(0);
        firstPose.root_position = pos1a; 
        model_1a->set_pose(firstPose);

        firstPose = motion_1b.poses.at(0);
        firstPose.root_position = pos1b; 
        model_1b->set_pose(firstPose);

        firstPose = motion_2a.poses.at(0);
        firstPose.root_position = pos2a; 
        model_2a->set_pose(firstPose);

        firstPose = motion_2b.poses.at(0);
        firstPose.root_position = pos2b; 
        model_2b->set_pose(firstPose);

        // frame = (frame + 1) % stitched_nof;
        
        // cam_follow_root(Vec3(0, 0, 0), cam_offset);


    }

    void render() override
    {
        agl::Render::plane()
            ->scale(15.0f)
            ->floor_grid(true)
            ->color(0.2f, 0.2f, 0.2f)
            ->draw();


        agl::Render::sphere()
            ->position(pos1a)
            ->color(1, 0, 0)
            ->scale(0.1f)
            ->draw();

        agl::Render::sphere()
            ->position(pos1b)
            ->color(0, 1, 0)
            ->scale(0.1f)
            ->draw();

        agl::Render::sphere()
            ->position(pos2a)
            ->color(0, 0, 1)
            ->scale(0.1f)
            ->draw();

        agl::Render::sphere()
            ->position(pos2b)
            ->color(1, 1, 0)
            ->scale(0.1f)
            ->draw();

        agl::Render::model(model_1a)->draw();
        agl::Render::model(model_1b)->draw();
        agl::Render::model(model_2a)->draw();
        agl::Render::model(model_2b)->draw();
    }

    void render_xray() override
    {
        agl::Render::sphere()
            ->position(pos1a)
            ->color(1, 0, 0)
            ->scale(0.1f)
            ->draw();

        agl::Render::sphere()
            ->position(pos1b)
            ->color(0, 1, 0)
            ->scale(0.1f)
            ->draw();

        agl::Render::sphere()
            ->position(pos2a)
            ->color(0, 0, 1)
            ->scale(0.1f)
            ->draw();

        agl::Render::sphere()
            ->position(pos2b)
            ->color(1, 1, 0)
            ->scale(0.1f)
            ->draw();

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