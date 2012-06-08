#include "childA.h"
#include <iostream>

void childA::purevirtual(parent *i_parent)
{
    std::cout << "childA::purevirtual" << std::endl;
}

void childA::print()
{
    std::cout << "childA::print() : " << m_data << std::endl;
}
