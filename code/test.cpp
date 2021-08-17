#include <aOpenGL.h>
#include <iostream>
#include <cmath>

class Test : public agl::App
{
public:
    Vec3 start_pos = Vec3(0, 0, 0);
    Vec3 end_pos = Vec3(1, 1, 1);
    Vec3 target_pos = Vec3(-1, 2, -1);

    Mat4 line_trf;
    Mat4 proj_line_trf;
    Mat4 target_line_trf;
    Mat4 proj_target_line_trf;

    Mat4 rotate_line_trf = Mat4().Identity();
    Mat4 result_line_trf;
    float line_length;

    void start() override
    {
        line_trf = draw_line(start_pos, end_pos);
        std::cout << "Initial line_trf" << std::endl;
        std::cout << line_trf << std::endl;
        std::cout << std::endl;

        target_line_trf = draw_line(start_pos, target_pos);
        std::cout << "Target line_trf" << std::endl;
        std::cout << target_line_trf << std::endl;
        std::cout << std::endl;

        // Project vectors onto xz plane
        end_pos.y() = 0.0f;
        proj_line_trf = draw_line(start_pos, end_pos);

        target_pos.y() = 0.0f;
        proj_target_line_trf = draw_line(start_pos, target_pos);

        // Get transformation between projected vectors
        rotate_line_trf = proj_line_trf.inverse();
        rotate_line_trf = proj_target_line_trf * rotate_line_trf;
        // 'displacement' transform -> not going to translate, only rotation
        rotate_line_trf.col(3).head<3>() = Vec3(0, 0, 0);

        // rotate original vector with rotation computed w/ projected vectors
        result_line_trf = rotate_line_trf * line_trf;

    }

    int frame = 0;
    void update() override
    {

    }

    void render() override
    {
        agl::Render::plane()
            ->scale(15.0f)
            ->floor_grid(true)
            ->color(0.2f, 0.2f, 0.2f)
            ->draw();


        agl::Render::cube()
            ->scale(0.1f, 0.1f, line_length)
            ->transform(line_trf)
            ->color(0, 1, 0)
            ->draw();

        agl::Render::cube()
            ->scale(0.1f, 0.1f, line_length)
            ->transform(proj_line_trf)
            ->color(0, 1, 0)
            ->draw();

        agl::Render::cube()
            ->scale(0.1f, 0.1f, line_length)
            ->transform(target_line_trf)
            ->color(0, 0, 1)
            ->draw();

        agl::Render::cube()
            ->scale(0.1f, 0.1f, line_length)
            ->transform(proj_target_line_trf)
            ->color(0, 0, 1)
            ->draw();

        agl::Render::cube()
            ->scale(0.1f, 0.1f, line_length)
            ->transform(result_line_trf)
            ->color(0, 1, 0.2)
            ->draw();

        agl::Render::sphere()
            ->position(start_pos)
            ->color(1, 0, 0)
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

    Mat4 draw_line(Vec3 start_pos, Vec3 end_pos)
    {
        Mat4 line_trf = Mat4().Identity();
        
        line_length = sqrt(pow(start_pos.x() - end_pos.x(), 2) + pow(start_pos.y() - end_pos.y(), 2) + pow(start_pos.z() - end_pos.z(), 2));
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
    Test app;
    agl::AppManager::start(&app);
    return 0;
} 