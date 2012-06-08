#include <iostream>
#include <hrpModel/ModelLoaderUtil.h>

int main(int argc, char *argv[])
{
    CORBA::ORB_var orb = CORBA::ORB::_nil();
  
    try {

	orb = CORBA::ORB_init(argc, argv);
	
        OpenHRP::ModelLoader_var ml = hrp::getModelLoader(orb);

        ml->_non_existent();
        std::cout << "alive" << std::endl;
    }
    catch (CORBA::SystemException& ex) {
        std::cout << "zombie" << std::endl;
    }
    catch (const std::string& error){
        std::cerr << error << std::endl;
    }

    try {
	orb->destroy();
    }
    catch(...){

    }
    
    return 0;
}
