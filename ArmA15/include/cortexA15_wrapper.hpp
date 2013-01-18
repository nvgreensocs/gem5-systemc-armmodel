#ifndef CORTEXA15_WRAPPER_HPP_
#define CORTEXA15_WRAPPER_HPP_

#include "cortexA15.hpp"

sc_core::sc_object* create_cortexA15 (const char * instance_name)
{
    CortexA15* cortexA15_instance = new CortexA15(instance_name,"cfg_port", "ace_master","debug_master");
    return (sc_core::sc_object*)(cortexA15_instance);
};

void destroy_cortexA15(sc_core::sc_object* instance){
    delete ((CortexA15*)instance);
};

#endif /*CORTEXA15_WRAPPER_HPP_*/
