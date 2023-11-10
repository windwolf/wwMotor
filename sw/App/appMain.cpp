#include "appMain.hpp"

#include "logger.hpp"
LOGGER("appMain")
namespace app {

// using namespace wibot::peripheral;
// using namespace wibot::accessor;
// using namespace wibot::device;
// using namespace ww;
// using namespace wibot::os;
// using namespace wibot::graph;
// using namespace wibot::comm;
// using namespace wibot::comm::test;

class App {
   public:
    App();
    void setup();
    void loop();

   private:
    // Application components
};

App::App(){};
void App::setup(){
    // TODO: Application setup code here.
};

void App::loop(){
    // TODO: Application main loop code here, if not use RTOS.
};
}  // namespace app

static app::App app1 = app::App();

void setup(void) {
    app1.setup();
}

void loop(void) {
    app1.loop();
}
