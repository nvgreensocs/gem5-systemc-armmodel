/*
*  Copyright 2012 by Texas Instruments Incorporated.
*  All rights reserved. Property of Texas Instruments Incorporated.
*  Restricted rights to use, duplicate or disclose this code are
*  granted through contract.
*/


/*CS******************************************************************

Project :  Nerios

Author	:	A. Roma�a

Date	:	Q4 2011

File	:	gem5_event_queue.hpp

Purpose	:	Class definition for the GEM5 event queue

Description:



******************************************************************CE*/

#ifndef _GEM5_EVENT_QUEUE_HPP_
#define _GEM5_EVENT_QUEUE_HPP_

/*************** standard files inclusion ***************/

#include <systemc>
#include <iostream>

/*************** pre-processor options definition ***********/

/*************** application files inclusion ***************/

#include "arch/arm/system.hh"
#include "mem/physical.hh"
#include "sim/root.hh"
#include "sim/core.hh"
#include "params/DerivO3CPU.hh"

/*************** macros definition ***************/

/*************** C types definition ***************/

/*************** constants definition ***************/

/*************** pre-processor autocheck ***************/

/*************** C++ types definition ***************/

class GEM5_event_queue : public sc_core::sc_module
{
public:

	sc_core::sc_event_queue gem5_sc_event_queue;

	
	SC_HAS_PROCESS(GEM5_event_queue);
	GEM5_event_queue(sc_core::sc_module_name nm);


	void serviceOne();

	uint64_t cp_timestamp;

};

/*************** global variables and arrays declaration ***************/

/*************** prototypes declaration ***************/

#endif/*_GEM5_EVENT_QUEUE_HPP_*/
