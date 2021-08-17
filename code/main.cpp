#include <aOpenGL.h>
#include <iostream>

static std::vector<agl::Pose> stitch_proj(
    const std::vector<agl::Pose>& poses_a,
    const std::vector<agl::Pose>& poses_b)
{
    std::vector<agl::Pose> new_poses = poses_a;

    agl::Pose aLast = poses_a.back();
    agl::Pose bFirst = poses_b.at(0);

    Vec3 a_last_pos = aLast.root_position;
    a_last_pos.y() = 0;
    Vec3 b_0_pos = bFirst.root_position;
    b_0_pos.y() = 0;

    // Project root - aLast
    Mat3 a_last_proj = Mat3().Identity();
    Vec3 y = Vec3(0, 1, 0);
    Vec3 z = aLast.local_rotations.at(0) * Vec3(0, 0, 1);
    z.y() = 0.0f;
    z.normalize();
    Vec3 x = y.cross(z);
    a_last_proj.col(0) = x;
    a_last_proj.col(1) = y;
    a_last_proj.col(2) = z;

    // Project root - bFirst
    Mat3 b_first_proj = Mat3().Identity();
    z = bFirst.local_rotations.at(0) * Vec3(0, 0, 1);   // rotation
    z.y() = 0.0f;
    z.normalize();
    x = y.cross(z);
    b_first_proj.col(0) = x;
    b_first_proj.col(1) = y;
    b_first_proj.col(2) = z;

    // Rotation between the projected roots
    Mat3 align_rotation = a_last_proj * b_first_proj.inverse();


    // rotate the pose
    for(int i = 0; i < poses_b.size(); i++)
    {
        agl::Pose new_pose = poses_b.at(i);

        Vec3 b_i_pos = poses_b.at(i).root_position;
        Vec3 dp = b_i_pos - b_0_pos;
        
        Vec3 new_root_pos = align_rotation * dp + a_last_pos;
        Mat3 new_root_orient = align_rotation * poses_b.at(i).local_rotations.at(0).matrix();

        // compute rotation
        new_pose.root_position = new_root_pos;
        new_pose.local_rotations.at(0) = Quat(new_root_orient);
        
        new_poses.push_back(new_pose);
    }

    return new_poses;
}


static std::vector<agl::Pose> stitch(
    const std::vector<agl::Pose>& poses_a,
    const std::vector<agl::Pose>& poses_b)
{
    std::vector<agl::Pose> new_poses = poses_a;

    Quat a_last_orient = poses_a.back().local_rotations.at(0);
    Vec3 a_last_pos = poses_a.back().root_position;
    a_last_pos.y() = 0.0f;

    Quat b_first_inverse = poses_b.at(0).local_rotations.at(0).inverse();
    Vec3 b_first_pos = poses_b.at(0).root_position;

    for(int i = 0; i < poses_b.size(); i++)
    {
        agl::Pose new_pose = poses_b.at(i);

        Vec3 b_i_pos = poses_b.at(i).root_position;
        Vec3 dp = (b_i_pos - b_first_pos);
        
        // Vec3 new_pos = (b_first_inverse * dp) + a_last_pos;
        // // Set original y values 
        // new_pos.y() = b_i_pos.y(); 
        // new_pose.root_position = new_pos;
        // new_pose.local_rotations.at(0) = b_first_inverse * new_pose.local_rotations.at(0);


        // Only z part
        Mat3 b_first_orient = poses_b.at(0).local_rotations.at(0).inverse().matrix();
        Mat3 new_inverse = Mat3().Identity();
        new_inverse.col(2) = poses_b.at(0).local_rotations.at(0).matrix().col(2);
        Quat new_inverse_quat = Quat(new_inverse);

        Mat3 a_last_orient_mat = a_last_orient.matrix();
        Mat3 new_a_orient = Mat3().Identity();
        new_a_orient.col(2) = a_last_orient_mat.col(2);

        Vec3 new_pos = new_a_orient * (new_inverse_quat * dp) + a_last_pos;
        new_pos.y() = b_i_pos.y(); 
        new_pose.root_position = new_pos;
        new_pose.local_rotations.at(0) = new_a_orient * new_inverse_quat * new_pose.local_rotations.at(0);



        new_poses.push_back(new_pose);
    }
    return new_poses;
}

class MyApp : public agl::App
{
public:
    agl::spModel            model;
    agl::Motion             motion_a;
    agl::Motion             motion_b;
    std::vector<agl::Pose>  motion_a_poses;
    std::vector<agl::Pose>  motion_b_poses;
    std::vector<agl::Pose>  stitched;
    std::vector<agl::Pose>  proj_stitched;
    Vec3 cam_offset;
    int a_nof;
    int b_nof;
    int stitched_nof;

    void start() override
    {
        const char* model_path      = "../data/fbx/ybot/model/ybot.fbx";
        const char* motion_path_a   = "../data/fbx/ybot/motion/Walking.fbx";
        const char* motion_path_b   = "../data/fbx/ybot/motion/Sneak Walk.fbx";

        // const char* motion_path_a   = "../data/fbx/ybot/motion/Running To Turn.fbx";
        // const char* motion_path_b   = "../data/fbx/ybot/motion/Running.fbx";

        agl::FBX model_fbx(model_path);
        agl::FBX motion_a_fbx(motion_path_a);
        agl::FBX motion_b_fbx(motion_path_b);

        model       = model_fbx.model();
        motion_a    = motion_a_fbx.motion(model).at(0);
        motion_b    = motion_b_fbx.motion(model).at(0);
        a_nof       = motion_a.poses.size();
        b_nof       = motion_b.poses.size();

        cam_offset = 2.0f * Vec3(0.0f, 3.0f, 3.0f);
    
        // stitched = stitch(motion_a.poses, motion_b.poses);
        // stitched_nof = stitched.size();

        proj_stitched = stitch_proj(motion_a.poses, motion_b.poses);
        stitched_nof = proj_stitched.size();
        

    }

    bool stop = false;
    int frame = 0;
    void update() override
    {
        if(stop)
            return;


        // // Play single motion A
        // model->set_pose(motion_a.poses.at(frame));
        // frame = (frame + 1) % (a_nof);

        // // Play single motion B
        // model->set_pose(motion_b.poses.at(frame));
        // frame = (frame + 1) % (b_nof);

        // // Play two motions
        // if(frame >= 0 && frame < a_nof)
        //     model->set_pose(motion_a.poses.at(frame));
        // else if(frame >= a_nof && frame < a_nof + b_nof)
        //     model->set_pose(motion_b.poses.at(frame-a_nof));
        // frame = (frame + 1) % (a_nof + b_nof);
        

        // Play stitched motion
        model->set_pose(proj_stitched.at(frame));
        frame = (frame + 1) % stitched_nof;


    }

    void render() override
    {
        agl::Render::plane()
            ->scale(15.0f)
            ->floor_grid(true)
            ->color(0.2f, 0.2f, 0.2f)
            ->draw();

        agl::Render::sphere()
            ->scale(0.3f)
            ->color(1.0f, 0, 0)
            ->position(0, 0, 0)
            ->draw();

        agl::Render::model(model)->draw();
    }

    void key_callback(char key, int action) override
    {
        if(action != GLFW_PRESS)
            return;
        if(key == 's')
            stop = !stop;
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