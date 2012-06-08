#include "parent.h"
#include "port.h"

class childB : public parent
{
public:
    childB() : m_sdata(m_ptr), m_ptr(NULL) {}
    void purevirtual(parent *i_parent);
    shared_data_base *get_shared() { return &m_sdata; }
    void print();

    shared_data<int *> m_sdata;
    int *m_ptr;
};
