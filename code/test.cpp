#include <aOpenGL.h>
#include <iostream>

class MyApp : public agl::App
{
public:
    void start() override
    {
        
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



        agl::Render::sphere()
            ->position(Vec3(-1.0f, 0, 0))
            ->color(0, 1, 0)
            ->scale(0.1f)
            ->draw();

        agl::Render::sphere()
            ->position(Vec3(1.0f, 0, 0))
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