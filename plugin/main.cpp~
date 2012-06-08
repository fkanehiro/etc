#include <iostream>
#include <dlfcn.h>
#include <typeinfo>
#include "parent.h"
#include "port.h"

int main(int argc, char *argv[])
{
    
    typedef parent *(*factory)();

    void *handleA = dlopen("./pluginA.so",RTLD_LAZY|RTLD_GLOBAL);
    if (!handleA) std::cerr << dlerror() << std::endl;
    factory factoryA = (factory) dlsym(handleA, "create_instance");
    parent *instanceA =factoryA();

    void *handleB = dlopen("./pluginB.so",RTLD_LAZY|RTLD_GLOBAL);
    if (!handleB) std::cerr << dlerror() << std::endl;
    factory factoryB = (factory) dlsym(handleB, "create_instance");
    parent *instanceB =factoryB();

    instanceA->purevirtual(instanceB);
    instanceB->purevirtual(instanceA);

    shared_data_base *out = instanceA->get_shared();
    shared_data_base *in = instanceB->get_shared();
    out->share(in);

    instanceB->print();

    delete instanceA;
    delete instanceB;

    dlclose(handleA);
    dlclose(handleB);

    return 0;
}
