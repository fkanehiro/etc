#include <iostream>
#include <string>
#include <boost/python.hpp>
#include <rtm/Manager.h>
using namespace std;

// てきとーな関数
string add_hello( string s = "default")
{
    std::cout << "add_hello is called" << std::endl;
    int argc=1;
    char *argv[] = {"dummy"};
    RTC::Manager::init(argc, argv);

	return "Hello, " + s;
}

int square( int n )
{
	return n * n;
}

using namespace boost::python;

class PyArray{
public:
    ~PyArray() { std::cout << "destructor" << std::endl; }
    bool construct(PyObject* op) { // op must be a unicode object
        _buff = (char *)(PyUnicode_AS_DATA(op));
        _size = PyUnicode_GET_SIZE(op);
        return true;
    };
    void setStr(std::string str){
        std::cout << "str = " << str << std::endl;
    }
    int getSize() { return _size; }
    void setSize(int s) { _size = s; }
private:
    char *_buff;
    int _size;
};

// モジュールの初期化ルーチン:モジュール名=my_sample
BOOST_PYTHON_MODULE( my_sample )
{
	// C++のadd_hello関数を、greetという名前でpython用に公開
	boost::python::def( "greet", add_hello );

	// C++のsquare関数を、squareという名前でpython用に公開
	boost::python::def( "square", square );

        class_<PyArray>("Array")
            .def("construct", &PyArray::construct)
            .def("setStr", &PyArray::setStr)
            .add_property("size", &PyArray::getSize, &PyArray::setSize)
            ;
}
