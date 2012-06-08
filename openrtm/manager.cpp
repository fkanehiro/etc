#include <rtm/Manager.h>

int main(int argc, char *argv[])
{
    RTC::Manager* manager;
    manager = RTC::Manager::init(argc, argv);
    manager->init(argc, argv);
    manager->activateManager();
    manager->runManager(true);

    sleep(1);

    manager->shutdown();

    return 0;
}
