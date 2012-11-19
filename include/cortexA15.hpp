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

File	:	cortexA15.hpp

Purpose	:	Class definition for the cortex A15 model

Description:



******************************************************************CE*/

#ifndef _CORTEXA15_HPP_
#define _CORTEXA15_HPP_

/*************** standard files inclusion ***************/

#include <systemc>
#include <iostream>
#include <Python.h>

/*************** pre-processor options definition ***********/

/*************** application files inclusion ***************/

#include "arch/arm/system.hh"
#include "arch/arm/linux/system.hh"
#include "arch/arm/interrupts.hh"
#include "arch/arm/nativetrace.hh"
#include "mem/physical.hh"
#include "mem/bus.hh"
#include "mem/cache/base.hh"
#include "sim/root.hh"
#include "sim/core.hh"
#include "sim/init.hh"
#include "cpu/intr_control.hh"
#include "cpu/o3/cpu.hh"
#include "cpu/o3/impl.hh"
#include "cpu/simple/atomic.hh"
#include "base/socket.hh"

#include "params/ArmTableWalker.hh"
#include "params/RealViewCtrl.hh"
#include "params/AtomicSimpleCPU.hh"
#include "params/DerivO3CPU.hh"
#include "params/IntrControl.hh"
#include "params/Terminal.hh"
#include "params/RealView.hh"
#include "params/AmbaFake.hh"
#include "params/Bridge.hh"
#include "params/Bus.hh"
#include "params/IsaFake.hh"
#include "params/A9SCU.hh"
#include "params/Gic.hh"
#include "params/CpuLocalTimer.hh"
#include "params/IdeController.hh"
#include "params/RawDiskImage.hh"
#include "params/CowDiskImage.hh"
#include "params/VncServer.hh"
#include "params/Sp804.hh"
#include "params/Pl050.hh"
#include "params/Pl011.hh"
#include "params/Pl111.hh"
#include "params/BaseCache.hh"
#include "params/StridePrefetcher.hh"
#include "python/swig/pyobject.hh"

#include <greencontrol/config.h>

#include "sc_tlm_config_interface.h"
#include "gem5_event_queue.hpp"
#include "bridge_classic_to_ambatlm2.hpp"
#include "nerios_arm_system.hpp"

/*************** macros definition ***************/

/*************** C types definition ***************/

/*************** constants definition ***************/

#define MAX_CORES  4

/*************** pre-processor autocheck ***************/

/*************** C++ types definition ***************/

class CortexA15 : public sc_core::sc_module
{

private:

	std::vector<SimObject *> objects;
	//std::queue<Nerios_config*> _pending_requests;
	
	ArmSystem * cortexA15;
	LinuxArmSystem * linuxCortexA15;
	
	gs::cnf::cnf_api *m_Api;
	
	gs::gs_param<std::string> _debug_flags_cfg;
	gs::gs_param<bool>        _disable_listeners_flags_cfg;
	gs::gs_param<uint32_t>    _num_cores_cfg;
	gs::gs_param<uint32_t>    _l2_cachesize_cfg;
	gs::gs_param<uint32_t>    _cluster_id_cfg;
	gs::gs_param<bool>        _gem5_uart_addr_cfg;
	gs::gs_param<std::string> _system_cfg;
	gs::gs_param<std::string> _linux_boot_loader_cfg;
	gs::gs_param<std::string> _linux_kernel_cfg;
	gs::gs_param<std::string> _linux_disk_image_cfg;
	gs::gs_param<std::string> _cpt_dir_cfg;
	
	sc_core::sc_time clk1;

	BaseO3CPU * cpu[MAX_CORES];
	AtomicSimpleCPU * dummy_cpu[MAX_CORES];
	ArmISA::TLB * dtlb[MAX_CORES];
	ArmISA::TLB * itlb[MAX_CORES];
	Trace::ExeTracer * tracer[MAX_CORES];
	ArmISA::Interrupts * interrupts[MAX_CORES];
	BaseCache * icache[MAX_CORES];
	BaseCache * dcache[MAX_CORES];
	BaseCache * xtb_walker_cache[MAX_CORES];
	Bus * xtb_walker_cache_bus[MAX_CORES];

	sc_core::sc_event   switch_cpus_evt;

public:

	// ports
	
	//sc_core::sc_port<Nerios_slave_config_if, 0, sc_core::SC_ZERO_OR_MORE_BOUND> nerios_cfg_port;
	
	SC_HAS_PROCESS(CortexA15);
	CortexA15(sc_core::sc_module_name nm, std::string cfg_port_name, std::string ace_master_name,std::string debug_master_name);
	
	//cpus database
	static std::map<std::string,CortexA15*> CA15;
	std::map<std::string,FullO3CPU<O3CPUImpl> *> cpus;
	Nerios_config*   _current_request;
	uint32_t         _num_cores;
	static uint32_t  _global_num_cores;
	static bool      _mem_is_serialized;

	void empty_requests();
	void enable_debug(std::string debug_string);
	void switch_cpus();
	void checkpoint(const std::string &cpt_dir);
	
	BridgeClassicToAMBATLM2<128> * toACEbridge;
   void running_the_kernel(void);

protected:

	void before_end_of_elaboration();
	void end_of_elaboration();
	void start_of_simulation();
	void service_config_bus();
	
};

/*************** global variables and arrays declaration ***************/

/*************** prototypes declaration ***************/

#endif/*_CORTEXA15_HPP_*/
