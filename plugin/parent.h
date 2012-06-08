#ifndef __PARENT_H__
#define __PARENT_H__
#include "port.h"
#include <cstdlib>

class parent
{
public:
    virtual ~parent() {}
    virtual void purevirtual(parent *i_parent) = 0;
    virtual shared_data_base *get_shared()=0;
    virtual void print() = 0;
};
#endif
