#include <iostream>
#include <boost/shared_ptr.hpp>

class Obj
{
public:
    ~Obj() { std::cout << "destructor" << std::endl; }
};

int main(int argc, char *argv[])
{
    boost::shared_ptr<Obj> o(new Obj());

    return 0;
}
