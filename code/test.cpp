#include <aOpenGL.h>
#include <iostream>
#include <cmath>

class Test : public agl::App
{
public:
    Vec3 start_pos = Vec3(0, 0, 0);
    Vec3 end_pos = Vec3(1, 1, 1);
    Mat4 line_trf;
    float z_scale;

    void start() override
    {
        line_trf = draw_line(start_pos, end_pos);

        
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
            ->scale(0.1f, 0.1f, z_scale)
            ->transform(line_trf)
            ->color(0, 1, 0)
            ->draw();


        agl::Render::sphere()
            ->position(end_pos)
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
        
        z_scale = sqrt(pow(start_pos.x() - end_pos.x(), 2) + pow(start_pos.y() - end_pos.y(), 2) + pow(start_pos.z() - end_pos.z(), 2));
        std::cout << z_scale << std::endl;
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