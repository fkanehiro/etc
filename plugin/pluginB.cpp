#include "childB.h"

extern "C"{
parent *create_instance()
{
    return new childB;
}
}

