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
        
        // compute rotation
        Vec3 new_root_pos = align_rotation * dp + a_last_pos;
        Mat3 new_root_orient = align_rotation * poses_b.at(i).local_rotations.at(0).matrix();

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
        
        Vec3 new_pos = a_last_orient * (b_first_inverse * dp) + a_last_pos;
        // Set original y values 
        new_pos.y() = b_i_pos.y(); 
        new_pose.root_position = new_pos;
        new_pose.local_rotations.at(0) = a_last_orient * b_first_inverse * new_pose.local_rotations.at(0);


        new_poses.push_back(new_pose);
    }
    return new_poses;
}

class MyApp : public agl::App
{
public:
    agl::spModel            model;
    agl::spModel            compare_model_a;
    agl::spModel            compare_model_b;
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

    // Visualize line from root
    float line_length = 5.0f;
    Vec3 line_end = Vec3(0, 0, 1);
    Vec3 line_start;
    Mat4 a_line_trf;

    Mat4 b_line_trf;

    

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

        model               = model_fbx.model();
        compare_model_a       = model_fbx.model();
        compare_model_b       = model_fbx.model();
        motion_a            = motion_a_fbx.motion(model).at(0);
        motion_b            = motion_b_fbx.motion(model).at(0);
        a_nof               = motion_a.poses.size();
        b_nof               = motion_b.poses.size();
        cam_offset = 2.0f * Vec3(0.0f, 3.0f, 3.0f);
    
        // stitched = stitch(motion_a.poses, motion_b.poses);
        // stitched_nof = stitched.size();

        proj_stitched = stitch_proj(motion_a.poses, motion_b.poses);
        stitched_nof = proj_stitched.size();
        

        // Get z axis of root - motion a
        line_start = proj_stitched.at(a_nof - 1).root_position;
        line_end = proj_stitched.at(a_nof - 1).local_rotations.at(0) * Vec3(0, 0, 1) + line_start;
        a_line_trf = get_line_transform(line_start, line_end);
        // Translate to see it better
        Vec3 new_pos = a_line_trf.col(3).head<3>();
        new_pos.x() -= 1.0f;
        a_line_trf.col(3).head<3>() = new_pos;

        // Get z axis of root - motion b
        line_start = proj_stitched.at(a_nof).root_position;
        line_end = proj_stitched.at(a_nof).local_rotations.at(0) * Vec3(0, 0, 1) + line_start;
        b_line_trf = get_line_transform(line_start, line_end);


    }

    bool stop = false;
    int frame = 0;
    void update() override
    {
        if(stop)
            return;
        

        // Play stitched motion
        model->set_pose(proj_stitched.at(frame));

        // Visualize last frame of motion a
        if(frame >= a_nof - 1)
        {
            agl::Pose compare_pose = proj_stitched.at(a_nof-1);
            // translate for better visuals
            compare_pose.root_position.x() -= 1.0f;
            compare_model_a->set_pose(compare_pose);
        }

        // Visualize first frame of motion b
        if(frame >= a_nof)
        {
            compare_model_b->set_pose(proj_stitched.at(a_nof));
        }

        frame = (frame + 1) % stitched_nof;


    }

    Mat4 get_line_transform(Vec3 start_pos, Vec3 end_pos)
    {
        Mat4 line_trf = Mat4().Identity();
        
        // line_length = sqrt(pow(start_pos.x() - end_pos.x(), 2) + pow(start_pos.y() - end_pos.y(), 2) + pow(start_pos.z() - end_pos.z(), 2));
        line_trf.col(3).head<3>() = (start_pos + end_pos)/2;

        Vec3 line_z = end_pos - start_pos;
        line_z.normalize();
        Vec3 line_y = Vec3(0, 1, 0).cross(line_z);
        Vec3 line_x = line_y.cross(line_z);
        line_trf.col(0).head<3>() = line_x;
        line_trf.col(1).head<3>() = line_y;
        line_trf.col(2).head<3>() = line_z;
        
        return line_trf;
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
        if(frame >= a_nof - 1)
        {
            agl::Render::model(compare_model_a)->draw();
            agl::Render::cube()
                ->scale(0.05f, 0.05f, 1.0f)
                ->color(0, 1, 0)
                ->transform(a_line_trf)
                ->draw();
        }

        if(frame >= a_nof)
        {
            agl::Render::model(compare_model_b)->draw();
            agl::Render::cube()
                ->scale(0.05f, 0.05f, 1.0f)
                ->color(0, 1, 0)
                ->transform(b_line_trf)
                ->draw();
        }

        
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
    
};

int main(int argc, char* argv[])
{
    MyApp app;
    agl::AppManager::start(&app);
    return 0;
} 