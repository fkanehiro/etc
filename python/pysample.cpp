#include <vector>
#include <new>
#include <algorithm>
#include <boost/python.hpp>
#include <boost/foreach.hpp>
#include <boost/range/value_type.hpp>

class foo {
public:
    typedef std::vector<int> int_vector;

public:
    int_vector const& get_list() const {
        return v_;
    }

    void add(int_vector const& that) {
        std::copy(that.begin(), that.end(), std::back_inserter(v_));
    }

    void add(int v) {
        v_.push_back(v);
    }

private:
    int_vector v_;
};

template<typename T_>
class vector_to_pylist_converter {
public:
    typedef T_ native_type;

    static PyObject* convert(native_type const& v) {
        namespace py = boost::python;
        py::list retval;
        BOOST_FOREACH(typename boost::range_value<native_type>::type i, v)
        {
            retval.append(py::object(i));
        }
        return py::incref(retval.ptr());
    }
};

template<typename T_>
class pylist_to_vector_converter {
public:
    typedef T_ native_type;

    static void* convertible(PyObject* pyo) {
        if (!PySequence_Check(pyo))
            return 0;

        return pyo;
    }

    static void construct(PyObject* pyo, boost::python::converter::rvalue_from_python_stage1_data* data)
    {
        namespace py = boost::python;
        native_type* storage = new(reinterpret_cast<py::converter::rvalue_from_python_storage<native_type>*>(data)->storage.bytes) native_type();
        for (py::ssize_t i = 0, l = PySequence_Size(pyo); i < l; ++i) {
            storage->push_back(
                py::extract<typename boost::range_value<native_type>::type>(
                    PySequence_GetItem(pyo, i)));
        }
        data->convertible = storage;
    }
};

BOOST_PYTHON_MODULE(pysample)
{
    using namespace boost::python;

    class_<foo>("foo")
        .def("get_list", &foo::get_list, return_value_policy<copy_const_reference>())
        .def("add", (void(foo::*)(int))&foo::add)
        .def("add", (void(foo::*)(foo::int_vector const&))&foo::add)
        ;
    to_python_converter<foo::int_vector, vector_to_pylist_converter<foo::int_vector> >();
    converter::registry::push_back(
        &pylist_to_vector_converter<foo::int_vector>::convertible,
        &pylist_to_vector_converter<foo::int_vector>::construct,
        boost::python::type_id<foo::int_vector>());
}
