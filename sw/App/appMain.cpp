#include "appMain.hpp"

#define LOG_MODULE "appMain"
#include "log.h"

namespace app
{

// using namespace ww::peripheral;
// using namespace ww::accessor;
// using namespace ww::device;
// using namespace ww;
// using namespace ww::os;
// using namespace ww::graph;
// using namespace ww::comm;
// using namespace ww::comm::test;

class App
{

  public:
    App();
    void setup();
    void loop();

  private:
    //Application components
};

App::App()
    {};
void App::setup()
{
    //TODO: Application setup code here.
};

void App::loop()
{
    //TODO: Application main loop code here, if not use RTOS.
};
} // namespace app

static app::App app1 = app::App();

void setup(void)
{
    app1.setup();
}

void loop(void)
{
    app1.loop();
}
