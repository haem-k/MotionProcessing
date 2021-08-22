#include <aOpenGL.h>
#include <iostream>
#include <cmath>

class RootRelative : public agl::App
{
public:
    agl::spModel model;
    std::vector<agl::Motion> motions;
    std::vector<Mat4> root_relative_jointTrf;
    int nof;
    int noj;

    void start() override
    {
        const char* model_path = "../../aOpenGL/data/fbx/kmodel/model/kmodel.fbx";
        const char* motion_path = "../../aOpenGL/data/fbx/kmodel/motion/ubi_sprint1_subject2.fbx";

        agl::FBX model_fbx(model_path);
        model = model_fbx.model();

        agl::FBX motion_fbx(motion_path);
        motions = motion_fbx.motion(model);
        
        // Get motion and get pose from 'frame'
        const auto& motion = motions.at(0);

        nof = (int)motion.poses.size();
        noj = (int)model->joints().size();

    }

    int frame = 0;
    void update() override
    {
        // Get motion and get pose from 'frame'
        const auto& motion = motions.at(0);
        const auto& pose = motion.poses.at(frame);

        // Set pose to the model
        model->set_pose(pose);
        root_relative_jointTrf = get_root_relative_jointTrf(model->joints());


        // Update frame count
        frame = (frame+1)%nof;
    }

    std::vector<Mat4> get_root_relative_jointTrf(std::vector<agl::spJoint> model_joints)
    {
        std::vector<Mat4> root_relative_jointTrf;
        int noj = model_joints.size();
        Mat4 root_trf = model_joints.at(0)->world_trf();
        Mat4 root_trf_inverse = root_trf.inverse();

        // Push_back root first
        root_relative_jointTrf.push_back(root_trf);

        for(int j = 0; j < noj; j++)
        {
            Mat4 joint_trf = model_joints.at(j)->world_trf();
            // Multiply inverse of root 
            joint_trf = root_trf_inverse * joint_trf;
            root_relative_jointTrf.push_back(joint_trf);
        }

        return root_relative_jointTrf;
    }



    void render() override
    {
        agl::Render::plane()
            ->scale(15.0f)
            ->floor_grid(true)
            ->color(0.2f, 0.2f, 0.2f)
            ->draw();

        for(int i = 0; i < root_relative_jointTrf.size(); i++)
        {
            agl::Render::sphere()
                ->transform(root_relative_jointTrf.at(i))
                ->scale(0.05f)
                ->color(1.0f, 0, 0)
                ->draw();
        }

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
    
};

int main(int argc, char* argv[])
{
    RootRelative app;
    agl::AppManager::start(&app);
    return 0;
} 