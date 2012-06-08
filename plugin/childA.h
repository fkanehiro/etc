#include "parent.h"
#include "port.h"

class childA : public parent
{
public:
    childA() : m_sdata(m_data), m_data(10) {}
    void purevirtual(parent *i_parent);
    shared_data_base *get_shared() { return &m_sdata; }
    void print();

    shared_data<int> m_sdata;
    int m_data;
};
