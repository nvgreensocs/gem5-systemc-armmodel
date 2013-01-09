/*
*  Copyright 2012 by Texas Instruments Incorporated.
*  All rights reserved. Property of Texas Instruments Incorporated.
*  Restricted rights to use, duplicate or disclose this code are
*  granted through contract.
*/


/*CS******************************************************************

Project :  Nerios

Author	:	A. Romaña

Date	:	Q4 2011

File	:	gem5_pollevent_queue.hpp

Purpose	:	Class definition for the GEM5 poll event queue

Description:



******************************************************************CE*/

#ifndef _GEM5_POLLEVENT_QUEUE_HPP_
#define _GEM5_POLLEVENT_QUEUE_HPP_

/*************** standard files inclusion ***************/

#include <systemc>
#include <iostream>

/*************** pre-processor options definition ***********/

/*************** application files inclusion ***************/

/*************** macros definition ***************/

/*************** C types definition ***************/

/*************** constants definition ***************/

/*************** pre-processor autocheck ***************/

/*************** C++ types definition ***************/

class GEM5_pollevent_queue : public sc_core::sc_module
{
public:

	sc_core::sc_event gem5_sc_event;

	
	SC_HAS_PROCESS(GEM5_pollevent_queue);
	GEM5_pollevent_queue(sc_core::sc_module_name nm);

	void serviceOne();
	void start_of_simulation();
};

/*************** global variables and arrays declaration ***************/

/*************** prototypes declaration ***************/

#endif/*_GEM5_POLLEVENT_QUEUE_HPP_*/
