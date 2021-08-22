#include <aOpenGL.h>
#include <iostream>
#include <cmath>

class RootRelative : public agl::App
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