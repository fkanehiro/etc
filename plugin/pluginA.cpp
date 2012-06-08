#include "childA.h"

extern "C"{
parent *create_instance()
{
    return new childA;
}
}

