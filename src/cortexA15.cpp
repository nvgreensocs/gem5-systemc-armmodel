/*********************************************************************
 *  Copyright 2012 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 ********************************************************************/

/*CS******************************************************************

Project :	CIV TLM

Author	:       Alexandre Romana

Date	:	Q4 2011

File	:	cortexA15.cpp

Purpose	:	Implementation unit for the cortex A15

Description:

VNC access is commented out - uncomment to re-enable

******************************************************************CE*/


/*************** standard files inclusion ***************/

#include <cmath>
#include <csignal>

/*************** pre-processor options definition ***********/

/*************** application files inclusion ***************/

#include "cortexA15.hpp"
#include "base/loader/object_file.hh"
#include "sim/stat_control.hh"
#include "sim/sim_events.hh"
#include "base/loader/symtab.hh"

/*************** macros definition ***************/

#define UART_BADDR 0x10100000
#define USE_A15

/*************** types definition ***************/

namespace Debug {
	extern CompoundFlag O3CPUAll;
	extern CompoundFlag Exec;
	extern CompoundFlag ExecAll;
	extern SimpleFlag Loader;
	extern SimpleFlag Fetch;
	extern SimpleFlag Decode;
	extern SimpleFlag Rename;
	extern SimpleFlag Uart;
	extern SimpleFlag Terminal;
	extern SimpleFlag TerminalVerbose;
	extern SimpleFlag BusAddrRanges;
	extern SimpleFlag Bus;
	extern SimpleFlag BusBridge;
	extern SimpleFlag Cache;
    extern SimpleFlag CachePort;
    extern SimpleFlag CacheRepl;
    extern SimpleFlag HWPrefetch;
    extern SimpleFlag LSQ;
    extern SimpleFlag LSQUnit;
    extern SimpleFlag IEW;
    extern SimpleFlag IQ;
    extern SimpleFlag O3CPU;
    extern SimpleFlag Checkpoint;
    extern SimpleFlag TLB;
    extern SimpleFlag TLBVerbose;
    extern SimpleFlag SimpleCPU;
    extern SimpleFlag MemDepUnit;
    extern SimpleFlag Config;
    extern SimpleFlag LLSC;
}

/*************** constants definition ***************/

/*************** pre-processor autocheck ***************/

/*************** global variables and arrays definition ***************/

extern GEM5_event_queue gem5_event_queue;

/*************** local variables and arrays definition ***************/

std::map<std::string,CortexA15*> CortexA15::CA15;
uint32_t      CortexA15::_global_num_cores=0;
bool          CortexA15::_mem_is_serialized=false;

/*************** local routines definition ***************/

//#define DEBUG_EN_PIPELINE
#ifdef DEBUG_EN_PIPELINE
#define DBG_PIPE_MSG( msg ) cout << "EagleNest : " << sc_time_stamp()	  \
        << "(" << sc_get_curr_simcontext()->delta_count() \
        << "): " << msg << endl;
#else
#define DBG_PIPE_MSG( msg )
#endif

/// Exit signal handler.
void
nerios_exitNowHandler(int sigtype)
{
	std::cout << "Received user interrupt, exiting..." << std::endl;
	sc_core::sc_stop();
}

/// Abort signal handler.
void
nerios_abortHandler(int sigtype)
{
    std::cerr << "Program aborted at cycle " << curTick() << std::endl;
    sc_core::sc_stop();
}

/*************** funtions definition  ***************/

CortexA15::CortexA15(sc_core::sc_module_name nm, std::string cfg_port_name, std::string ace_master_name,std::string debug_master_name):
sc_core::sc_module(nm),
//nerios_cfg_port(cfg_port_name.c_str()),
_debug_flags_cfg("debug_flags",""),
_l2_cachesize_cfg("l2_cachesize_kB",4096),
_num_cores_cfg("num_cores",1),
_disable_listeners_flags_cfg("disable_listeners",false),
_cluster_id_cfg("cluster_id",0),
_system_cfg("system_config","bareMetal"),
_linux_boot_loader_cfg("linux_boot_loader","./scripts/binaries/boot.arm"),
_linux_kernel_cfg("linux_kernel","./scripts/binaries/vmlinux.arm.smp.fb.2.6.38.8"),
_linux_disk_image_cfg("linux_disk_image","./scripts/binaries/linux-arm-ael.img"),
_cpt_dir_cfg("linux_checkpoint_dir","./scripts/binaries/cpt.56896248877000"),
_gem5_uart_addr_cfg("gem5_uart_addr",false)
{

	m_Api = gs::cnf::GCnf_Api::getApiInstance(this);
	//_current_request=NULL;
		
	setClockFrequency(LL(1000000000000));
	
    toACEbridge = new BridgeClassicToAMBATLM2<128>((std::string(name())+".toACEbridge").c_str(),ace_master_name,debug_master_name);

	curTick(0);
	
	if (_disable_listeners_flags_cfg)
		ListenSocket::disableAll();

	// Initialize m5 special signal handling:
	
    // Floating point exceptions may happen on misspeculated paths, so
    // ignore them
    signal(SIGFPE, SIG_IGN);

	// Exit cleanly on Interrupt (Ctrl-C)
    signal(SIGINT, nerios_exitNowHandler);

    // Print out cycle number on abort
    signal(SIGABRT, nerios_abortHandler);
};

void CortexA15::service_config_bus()
{
	/*Nerios_config* _new_config=NULL;
	for (int i = 0; i < nerios_cfg_port.size(); i++)
	{
		bool more = true;
		while (more)
		{
			more = nerios_cfg_port[i]->read(_new_config);
			if (_new_config != NULL)
			{
				_pending_requests.push(_new_config);
				_new_config=NULL;
			}
		}
	}
	empty_requests();*/
};

void CortexA15::enable_debug(std::string debug_string)
{
	std::vector<std::string> debug_flags;
	std::stringstream ss(debug_string);
	std::string item;
	while(std::getline(ss, item, ',')) 
	{
		if (item != "")
			debug_flags.push_back(item);
	}
	for (uint32_t i=0;i<debug_flags.size();i++)
	{
		Trace::enabled=true;
		if(debug_flags.at(i) == "O3CPUAll" )
		{
			(Debug::O3CPUAll).enable();
		}
		else if (debug_flags.at(i) == "Exec" )
		{
			(Debug::Exec).enable();
		}
		else if (debug_flags.at(i) == "ExecAll" )
		{
			(Debug::ExecAll).enable();
		}
		else if (debug_flags.at(i) == "Fetch" )
		{
			(Debug::Fetch).enable();
		}
		else if (debug_flags.at(i) == "Decode" )
		{
			(Debug::Decode).enable();
		}
		else if (debug_flags.at(i) == "Rename" )
		{
			(Debug::Rename).enable();
		}
		else if(debug_flags.at(i) == "Loader" )
		{
			(Debug::Loader).enable();
		}
		else if(debug_flags.at(i) == "Uart" )
		{
			(Debug::Uart).enable();
		}
		else if(debug_flags.at(i) == "Terminal" )
		{
			(Debug::Terminal).enable();
		}
		else if(debug_flags.at(i) == "TerminalVerbose" )
		{
			(Debug::TerminalVerbose).enable();
		}
		else if(debug_flags.at(i) == "BusAddrRanges" )
		{
			(Debug::BusAddrRanges).enable();
		}
		else if(debug_flags.at(i) == "Bus" )
		{
			(Debug::Bus).enable();
		}
		else if(debug_flags.at(i) == "BusBridge" )
		{
			(Debug::BusBridge).enable();
		}
		else if(debug_flags.at(i) == "Cache" )
		{
			(Debug::Cache).enable();
		}
		else if(debug_flags.at(i) == "CachePort" )
		{
			(Debug::CachePort).enable();
		}
		else if(debug_flags.at(i) == "CacheRepl" )
		{
			(Debug::CacheRepl).enable();
		}
		else if(debug_flags.at(i) == "HWPrefetch" )
		{
			(Debug::HWPrefetch).enable();
		}
		else if(debug_flags.at(i) == "LSQ" )
		{
			(Debug::LSQ).enable();
		}
		else if(debug_flags.at(i) == "LSQUnit" )
		{
			(Debug::LSQUnit).enable();
		}
		else if(debug_flags.at(i) == "IEW" )
		{
			(Debug::IEW).enable();
		}
		else if(debug_flags.at(i) == "IQ" )
		{
			(Debug::IQ).enable();
		}
		else if(debug_flags.at(i) == "O3CPU" )
		{
			(Debug::O3CPU).enable();
		}
		else if(debug_flags.at(i) == "Checkpoint" )
		{
			(Debug::Checkpoint).enable();
		}
		else if(debug_flags.at(i) == "TLB" )
		{
			(Debug::TLB).enable();
		}
		else if(debug_flags.at(i) == "SimpleCPU" )
		{
			(Debug::SimpleCPU).enable();
		}
		else if(debug_flags.at(i) == "MemDepUnit" )
		{
			(Debug::SimpleCPU).enable();
		}
		else if (debug_flags.at(i) == "Config")
		{
			(Debug::Config).enable();
		}
		else if(debug_flags.at(i) == "TLBVerbose" )
		{
			(Debug::TLBVerbose).enable();
		}
		else if(debug_flags.at(i) == "LLSC" )
		{
			(Debug::LLSC).enable();
		}
		else
		{
			std::cout << "ERROR: unrecognized debug flag for A15: " << debug_flags.at(i) << std::endl;
			sc_core::sc_stop();
			return;				
		}
	}
}

void CortexA15::empty_requests()
{
	//if ((_current_request == NULL) && (_pending_requests.size() > 0) && (_system_cfg == "bareMetal") )
	{
		curTick(sc_core::sc_time_stamp().value());
		
	//	_current_request=_pending_requests.front();
	//	_pending_requests.pop();
		// registering state
		//Vsu::state_collector.add_state(_current_request->task_name,sc_core::sc_time_stamp().value(),RUNNING,name());

		// enable debug
		std::string debug_gem5="";
		m_Api->getValue<std::string> ("Tasks.test.Task1.debug",debug_gem5);
		enable_debug(debug_gem5);

		uint32_t frq_mhz = uint32_t(ceil(1000000/clk1.value()));
		std::cout << name() << " running at " << frq_mhz << " MHz" << std::endl;
		ArmSystemParams* sysPars= const_cast<ArmSystemParams*>(cortexA15->params());
		//if (m_Api->existsParam("Tasks." + _current_request->task_name + ".kernel"))
		if (m_Api->existsParam("Tasks.test.Task1.kernel"))
		{
			sysPars->kernel=m_Api->getValue("Tasks.test.Task1.kernel");
			std::cout << "STARTING kernel " << cortexA15->params()->kernel << " on " << name() << " all cores" << std::endl;

            // Get the kernel code
			cortexA15->kernel = createObjectFile(sysPars->kernel);

            if (cortexA15->kernel == NULL)
                fatal("Could not load kernel file %s", sysPars->kernel);

            // setup entry points
            cortexA15->kernelStart = cortexA15->kernel->textBase();
            cortexA15->kernelEnd = cortexA15->kernel->bssBase() + cortexA15->kernel->bssSize();
            cortexA15->kernelEntry = cortexA15->kernel->entryPoint();

            // load symbols
            if (!cortexA15->kernel->loadGlobalSymbols(cortexA15->kernelSymtab))
                fatal("could not load kernel symbols\n");

            if (!cortexA15->kernel->loadLocalSymbols(cortexA15->kernelSymtab))
                fatal("could not load kernel local symbols\n");

            if (!cortexA15->kernel->loadGlobalSymbols(debugSymbolTable))
                fatal("could not load kernel symbols\n");

            if (!cortexA15->kernel->loadLocalSymbols(debugSymbolTable))
                fatal("could not load kernel local symbols\n");


			// launching the engine
			cortexA15->initState();
		}
		/*else if (m_Api->existsParam("Tasks." + _current_request->task_name + ".kernel_list.1"))
		{
			std::vector<string> kernel_list = m_Api->getParamList("Tasks." + _current_request->task_name + ".kernel_list");
			std::vector<string> core_list = m_Api->getParamList("Tasks." + _current_request->task_name + ".core_list");
			assert(kernel_list.size() == core_list.size());
			Addr kernel0Entry;
			for( int i = 0; i < kernel_list.size(); i++)
			{
				sysPars->kernel=m_Api->getValue(kernel_list[i]);
                      
		                // Get the kernel code
		                cortexA15->kernel = createObjectFile(sysPars->kernel);

		                if (cortexA15->kernel == NULL)
		                    fatal("Could not load kernel file %s", sysPars->kernel);

		                // setup entry points
		                cortexA15->kernelStart = cortexA15->kernel->textBase();
		                cortexA15->kernelEnd = cortexA15->kernel->bssBase() + cortexA15->kernel->bssSize();
		                cortexA15->kernelEntry = cortexA15->kernel->entryPoint();

		                // load symbols
		                if (!cortexA15->kernel->loadGlobalSymbols(cortexA15->kernelSymtab))
		                    fatal("could not load kernel symbols\n");

		                if (!cortexA15->kernel->loadLocalSymbols(cortexA15->kernelSymtab))
		                    fatal("could not load kernel local symbols\n");

		                if (!cortexA15->kernel->loadGlobalSymbols(debugSymbolTable))
                                    fatal("could not load kernel symbols\n");

		                if (!cortexA15->kernel->loadLocalSymbols(debugSymbolTable))
                                    fatal("could not load kernel local symbols\n");
				
				cortexA15->initState();
				std::cout << "STARTING kernel " << cortexA15->params()->kernel << " on " << name() << " core " << m_Api->getValue<uint32_t>(core_list[i]) << std::endl;
				if (m_Api->getValue<uint32_t>(core_list[i]) == 0)
					kernel0Entry = cortexA15->getKernelEntry();
				else
					cortexA15->threadContexts[m_Api->getValue<uint32_t>(core_list[i])]->pcState(cortexA15->getKernelEntry());
			}
			cortexA15->threadContexts[0]->pcState(kernel0Entry);
		}*/
		for (int i = 0; i < cortexA15->threadContexts.size(); i++)
		{
			cortexA15->threadContexts[i]->setIntReg(6, _global_num_cores);
			cortexA15->threadContexts[i]->setIntReg(8, frq_mhz);
		}
		for (uint32_t i=0;i<objects.size();i++)
			objects[i]->initState();
		for (uint32_t i=0;i<_num_cores;i++)
			cpu[i]->initState();

		cortexA15->resetStats();
		for (uint32_t i=0;i<objects.size();i++)
			objects[i]->resetStats();
		for (uint32_t i=0;i<_num_cores;i++)
			cpu[i]->resetStats();

		cortexA15->startup();
		for (uint32_t i=0;i<objects.size();i++)
			objects[i]->startup();
		for (uint32_t i=0;i<_num_cores;i++)
			cpu[i]->startup();
  	}
};

void CortexA15::before_end_of_elaboration()
{
	_num_cores=_num_cores_cfg;
	_global_num_cores += _num_cores;
	Range<long long unsigned int> rb("0x0:+0xFFFFFFFFFFFFFFFF");
	
	if (CA15.empty())
	{
		RootParams * rootParams= new RootParams();
		rootParams->name="root";
		rootParams->full_system=true;
		rootParams->time_sync_period = 	LL(100000000000);
		rootParams->time_sync_enable = false;
		rootParams->time_sync_spin_threshold = 100000000;
		Root * root = rootParams->create();
		objects.push_back(root);
	}

	CortexA15::CA15[std::string(name())]=this;

	MANDATORY_GS_PARAM(sc_core::sc_time,"clk1_arm",clk1);

	//////////////////////////
	// construct arm system //
	//////////////////////////

    IdeDisk * ideDisk = NULL;
    Pl111 * clcd = NULL;
    Pl050 * kmi0 = NULL;
    Pl050 * kmi1 = NULL;
    VncServer * vncserver = NULL;
	
	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
	{
		/*PhysicalMemoryParams* nvmParams = new PhysicalMemoryParams();
		nvmParams->name=std::string(name())+".nvmem";
		nvmParams->latency=30000;
		nvmParams->zero=true;
		nvmParams->latency_var=0;
		nvmParams->range;
		nvmParams->file;
		nvmParams->null=false;
		nvmParams->range=Range<long long unsigned int>("0x80000000:+0x3FFFFFF");*/


		LinuxArmSystemParams* sysParam = new LinuxArmSystemParams();
		sysParam->name=std::string(name());
		sysParam->boot_loader=_linux_boot_loader_cfg;
//		sysParam->boot_loader_mem=toACEbridge;// 64MB @ 2GB address
		sysParam->boot_osflags="earlyprintk console=ttyAMA0 lpj=19988480 norandmaps rw loglevel=8 mem=128MB root=/dev/sda1";
		sysParam->flags_addr=0x10000030;
		sysParam->gic_cpu_addr=0x1f000100;
		sysParam->init_param=0;
		sysParam->kernel=_linux_kernel_cfg;
		sysParam->load_addr_mask=0xFFFFFFF;
		sysParam->machine_type=Enums::RealView_PBX;
		sysParam->mem_mode=Enums::atomic;
		//sysParam->mem_mode=Enums::timing;
		sysParam->memories.push_back(toACEbridge);
		sysParam->midr_regval=890224640;
		sysParam->num_work_ids=16;
		sysParam->atags_addr=256;
//		sysParam->physmem=toACEbridge;
		sysParam->readfile="";
		sysParam->symbolfile="";
		sysParam->work_begin_ckpt_count=0;
		sysParam->work_begin_cpu_id_exit=-1;
		sysParam->work_begin_exit_count=0;
		sysParam->work_cpus_ckpt_count=0;
		sysParam->work_end_ckpt_count=0;
		sysParam->work_end_exit_count=0;
		sysParam->work_item_id=-1;
		linuxCortexA15 = sysParam->create();
		
		RawDiskImageParams* rawDiskImageParams = new RawDiskImageParams();
		rawDiskImageParams->name=std::string(name())+".cf0.image.child";
		rawDiskImageParams->image_file=_linux_disk_image_cfg;
		rawDiskImageParams->read_only=true; // FIXME? Could change that?
		RawDiskImage * rawDiskImage = rawDiskImageParams->create();
  		objects.push_back((SimObject*)rawDiskImage);

		CowDiskImageParams* cowDiskImageParams = new CowDiskImageParams();
		cowDiskImageParams->name=std::string(name())+".cf0.image";
		cowDiskImageParams->child=(DiskImage*)rawDiskImage;
		cowDiskImageParams->image_file="";
		cowDiskImageParams->read_only=false;
		cowDiskImageParams->table_size=65536;
		CowDiskImage * cowDiskImage = cowDiskImageParams->create();
  		objects.push_back((SimObject*)cowDiskImage);
		
		IdeDiskParams* ideDiskParams = new IdeDiskParams();
		ideDiskParams->name=std::string(name())+".cf0";
		ideDiskParams->driveID=Enums::master;
		ideDiskParams->delay=1000*clk1.value();//1000000; FIXME should be lower!
		ideDiskParams->image=(DiskImage*)cowDiskImage;
		ideDisk = ideDiskParams->create();
	  	objects.push_back((SimObject*)ideDisk);

	}
	else
	{
		ArmSystemParams* sysParam = new ArmSystemParams();
		sysParam->name=std::string(name())+"";
		sysParam->kernel="";
		sysParam->work_begin_exit_count = 0;
		sysParam->mem_mode = Enums::timing;
		sysParam->work_end_ckpt_count = 0;
		sysParam->init_param = 0;
		sysParam->memories.push_back(toACEbridge);
		sysParam->load_addr_mask = 0xFFFFFFFF;
		sysParam->symbolfile = "";
//		sysParam->physmem=toACEbridge;
		sysParam->work_cpus_ckpt_count = 0;
		sysParam->readfile = "";
		sysParam->work_begin_ckpt_count = 0;
		sysParam->work_end_exit_count = 0;
		sysParam->boot_osflags="a";
		sysParam->work_item_id = -1;
		sysParam->work_begin_cpu_id_exit = -1;
		sysParam->gic_cpu_addr = 0, 
//		sysParam->boot_loader_mem = 0x0;
		sysParam->midr_regval = 890224640;
		sysParam->boot_loader = "";
		sysParam->flags_addr = 0;
		sysParam->num_work_ids=16;
		cortexA15 = sysParam->create();
	}

	BaseCacheParams * baseCacheParams = new BaseCacheParams();
	baseCacheParams->name=std::string(name())+".iocache";
	baseCacheParams->addr_ranges.push_back(Range<long long unsigned int>("0x0:+0xfffffff"));// Range<long long unsigned int>("0x0:+0x7ffffff"); Moving to 256MB linux RAM moves LCD frame buffer by 128MB to 0xF0000000
	baseCacheParams->assoc = 8;
	baseCacheParams->block_size = 64;
	baseCacheParams->forward_snoops = false;
	baseCacheParams->hash_delay = 1;
	baseCacheParams->is_top_level = true;
	baseCacheParams->latency = 10*clk1.value();//10000;
	baseCacheParams->max_miss_count = 0;
	baseCacheParams->mshrs = 20;
	baseCacheParams->prefetch_on_access = false;
	baseCacheParams->prefetcher = NULL;
	baseCacheParams->prioritizeRequests = false;
	baseCacheParams->repl = NULL;
	baseCacheParams->size = 1024;
	baseCacheParams->subblock_size = 0;
	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
		baseCacheParams->system=linuxCortexA15;
	else
		baseCacheParams->system = cortexA15;
	baseCacheParams->tgts_per_mshr = 12;
	baseCacheParams->trace_addr = 0;
	baseCacheParams->two_queue = false;
	baseCacheParams->write_buffers = 8;
  	BaseCache * iocache = baseCacheParams->create();
  	objects.push_back((SimObject*)iocache);

#ifdef USE_A15

  	BaseCache * l2;
  	if (_system_cfg != "linux")
  	{
  		StridePrefetcherParams* stridePrefetcherParams = new StridePrefetcherParams();
		stridePrefetcherParams->name=std::string(name())+".l2.prefetcher";
		stridePrefetcherParams->cross_pages=false;
		stridePrefetcherParams->data_accesses_only=false;
		stridePrefetcherParams->degree=8;
		stridePrefetcherParams->serial_squash=false;
		stridePrefetcherParams->size=100;
		stridePrefetcherParams->use_master_id=true;
		stridePrefetcherParams->latency = clk1.value();//1000;
		if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
			stridePrefetcherParams->sys=linuxCortexA15;
		else
			stridePrefetcherParams->sys = cortexA15;
		StridePrefetcher * l2_prefetcher = stridePrefetcherParams->create();
		objects.push_back((SimObject*)l2_prefetcher);

		//  Assume below for Tetris L2 hit latency:

		//          Dual         Quad
		//256K       10           12
		//512K       10           12
		//1M         12           14
		//2M         12           14
		//4M         14           16

		baseCacheParams = new BaseCacheParams();
		baseCacheParams->name=std::string(name())+".l2";
		baseCacheParams->addr_ranges.push_back(rb);
		baseCacheParams->assoc = 8;
		baseCacheParams->block_size = 64;
		baseCacheParams->forward_snoops = true;
		baseCacheParams->hash_delay = 1;
		baseCacheParams->is_top_level = false;
		uint32_t l2_cachesize = _l2_cachesize_cfg;
		if (l2_cachesize <= 512)
		{
			if (_num_cores <= 2)
				baseCacheParams->latency = 10*clk1.value();//14000;
			else
				baseCacheParams->latency = 12*clk1.value();//14000;
		}
		else if (l2_cachesize <= 2048)
		{
			if (_num_cores <= 2)
				baseCacheParams->latency = 12*clk1.value();//14000;
			else
				baseCacheParams->latency = 14*clk1.value();//14000;
		}
		else
		{
			if (_num_cores <= 2)
				baseCacheParams->latency = 14*clk1.value();//14000;
			else
				baseCacheParams->latency = 16*clk1.value();//14000;
		}
		baseCacheParams->max_miss_count = 0;
		baseCacheParams->mshrs = 16;
		baseCacheParams->prefetch_on_access = true;
		baseCacheParams->prioritizeRequests = false;
		baseCacheParams->repl = NULL;
		baseCacheParams->size = _l2_cachesize_cfg*1024;
		baseCacheParams->subblock_size = 0;
		baseCacheParams->tgts_per_mshr = 8;
		baseCacheParams->trace_addr = 0;
		baseCacheParams->two_queue = false;
		baseCacheParams->write_buffers = 8;
		baseCacheParams->prefetcher=(BasePrefetcher*)l2_prefetcher;
		if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
			baseCacheParams->system=linuxCortexA15;
		else
			baseCacheParams->system = cortexA15;
		l2 = baseCacheParams->create();
		objects.push_back((SimObject*)l2);
  	}

#else

	baseCacheParams = new BaseCacheParams();
	baseCacheParams->name=std::string(name())+".l2";
	baseCacheParams->write_buffers = 8;
	baseCacheParams->is_top_level = false;
	baseCacheParams->block_size = 64;
	baseCacheParams->prefetch_latency = 100*clk1.value();//1000;
	baseCacheParams->size = _l2_cachesize_cfg*1024;
	baseCacheParams->latency = 16*clk1.value();//14000;
	baseCacheParams->addr_ranges.push_back(rb);
	baseCacheParams->num_cpus = _num_cores;
	baseCacheParams->trace_addr = 0;
	baseCacheParams->max_miss_count = 0;
	baseCacheParams->mshrs = 20;
	baseCacheParams->forward_snoops = true;
	baseCacheParams->tgts_per_mshr = 12;
	baseCacheParams->repl = 0x0;
	baseCacheParams->assoc = 8;
	baseCacheParams->prefetcher = NULL;
	baseCacheParams->prioritizeRequests = false;
	baseCacheParams->hash_delay = 1;
	baseCacheParams->subblock_size = 0;
	baseCacheParams->prefetcher_size = 100;
	baseCacheParams->two_queue = false;
  	BaseCache * l2 = baseCacheParams->create();
  	objects.push_back((SimObject*)l2);

#endif

	IntrControlParams * intCtrlParams = new IntrControlParams();
	intCtrlParams->name=std::string(name())+".intrctrl";
	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
		intCtrlParams->sys=linuxCortexA15;
	else
		intCtrlParams->sys=cortexA15;
	IntrControl * intc = intCtrlParams->create();
	objects.push_back((SimObject*)intc);

    RealViewParams * rvParams = new RealViewParams();
    rvParams->name=std::string(name())+".realview";
	rvParams->max_mem_size=256*1024*1024;
	rvParams->mem_start_addr=0;
    rvParams->intrctrl = intc;
    rvParams->pci_cfg_base = 0;
	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
		rvParams->system=linuxCortexA15;
	else
		rvParams->system = cortexA15;
 	RealView * rv = rvParams->create();
 	objects.push_back((SimObject*)rv);

	BusParams * busParams = new BusParams();
	busParams->name=std::string(name())+".membus";
	busParams->clock = clk1.value();//1000;
  	busParams->header_cycles = 1;
  	busParams->width = 64; // bytes //TODO: move to 16!!!
  	busParams->use_default_range = false;
  	busParams->block_size = 64;
  	busParams->bus_id = 1;// unused?
  	busParams->port_master_connection_count=6;
  	if (_system_cfg == "linux")
  		busParams->port_slave_connection_count=4*_num_cores+1;
  	else
  		busParams->port_slave_connection_count=2;
  	busParams->port_default_connection_count=1;
  	Bus * membus = busParams->create();
	objects.push_back((SimObject*)membus);

    busParams = new BusParams();
	busParams->name=std::string(name())+".iobus";
	busParams->clock = clk1.value();//1000;
  	busParams->header_cycles = 1;
  	busParams->width = 64;// bytes
  	busParams->use_default_range = false;
  	busParams->block_size = 64;
  	busParams->bus_id = 0; // unused?
	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
	{
		busParams->port_master_connection_count=26;
	  	busParams->port_slave_connection_count=3;
	}
	else
	{
		busParams->port_master_connection_count=22;
	  	busParams->port_slave_connection_count=2;
	}
  	busParams->port_default_connection_count=0;
  	Bus * iobus = busParams->create();
	objects.push_back((SimObject*)iobus);
	
	Bus * tol2bus;
	if (_system_cfg != "linux")
	{
		busParams = new BusParams();
		busParams->name=std::string(name())+".tol2bus";
		busParams->clock = clk1.value();//1000;
		busParams->header_cycles = 1;
		busParams->width = 64; // bytes
		busParams->use_default_range = false;
		busParams->block_size = 64;
		busParams->bus_id = 0;// unused?
		busParams->port_master_connection_count=1;
		busParams->port_slave_connection_count=3*_num_cores;
		busParams->port_default_connection_count=0;
		tol2bus = busParams->create();
		objects.push_back((SimObject*)tol2bus);
	}
	
	IsaFakeParams * isafkParams = new IsaFakeParams();
	isafkParams->name=std::string(name())+".membus.badaddr_responder";
	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
		isafkParams->system=linuxCortexA15;
	else
		isafkParams->system=cortexA15;
    isafkParams->pio_addr = 0;
    isafkParams->pio_latency = clk1.value();//1000;
    isafkParams->ret_data8 = 0xFF;
    isafkParams->update_data = false;
    isafkParams->warn_access = "warn"; 
    isafkParams->ret_bad_addr = true;
    isafkParams->ret_data64 = ULL(18446744073709551615); 
    isafkParams->fake_mem = false;
    isafkParams->pio_size = 8;
    isafkParams->ret_data32 = 0xFFFFFFFF;
    isafkParams->ret_data16 = 0xFFFF;
    IsaFake * badaddr_responder = isafkParams->create();
	objects.push_back((SimObject*)badaddr_responder);

	/*isafkParams = new IsaFakeParams();
	isafkParams->name=std::string(name())+".iobus.badaddr_responder";
	isafkParams->platform=(Platform*)rv;
	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
		isafkParams->system=linuxCortexA15;
	else
		isafkParams->system=cortexA15;
    isafkParams->pio_addr = 0;
    isafkParams->pio_latency = clk1.value();//1000;
    isafkParams->ret_data8 = 0xFF;
    isafkParams->update_data = false;
    isafkParams->warn_access = "warn";
    isafkParams->ret_bad_addr = true;
    isafkParams->ret_data64 = ULL(18446744073709551615);
    isafkParams->fake_mem = false;
    isafkParams->pio_size = 8;
    isafkParams->ret_data32 = 0xFFFFFFFF;
    isafkParams->ret_data16 = 0xFFFF;
    IsaFake * badaddr_responder2 = isafkParams->create();
	objects.push_back((SimObject*)badaddr_responder2);*/

	//isafkParams = new IsaFakeParams();
	//isafkParams->name=std::string(name())+".realview.flash_fake";
	//isafkParams->platform=(Platform*)rv;
	//isafkParams->system=cortexA15;
    //isafkParams->pio_addr = 0x40000000;
    //isafkParams->pio_latency = 1000;
    //isafkParams->ret_data8 = 0xFF;
    //isafkParams->update_data = false;
    //isafkParams->warn_access = ""; 
    //isafkParams->ret_bad_addr = false;
    //isafkParams->ret_data64 = ULL(18446744073709551615); 
    //isafkParams->fake_mem = true;
    //isafkParams->pio_size = 8;
    //isafkParams->ret_data32 = 0xFFFFFFFF;
    //isafkParams->ret_data16 = 0xFFFF;
    //IsaFake * flash_fake = isafkParams->create();
	//objects.push_back((SimObject*)flash_fake);

	isafkParams = new IsaFakeParams();
	isafkParams->name=std::string(name())+".realview.l2x0_fake";
	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
		isafkParams->system=linuxCortexA15;
	else
		isafkParams->system=cortexA15;
    isafkParams->pio_addr = 0x1F002000;
    isafkParams->pio_latency = clk1.value();//1000;
    isafkParams->ret_data8 = 0xFF;
    isafkParams->update_data = false;
    isafkParams->warn_access = ""; 
    isafkParams->ret_bad_addr = false;
    isafkParams->ret_data64 = ULL(18446744073709551615); 
    isafkParams->fake_mem = false;
    isafkParams->pio_size = 4095;
    isafkParams->ret_data32 = 0xFFFFFFFF;
    isafkParams->ret_data16 = 0xFFFF;
    IsaFake * l2x0_fake = isafkParams->create();
	objects.push_back((SimObject*)l2x0_fake);
	
	BridgeParams * bridgeParams = new BridgeParams();
	bridgeParams->name=std::string(name())+".bridge";
	bridgeParams->write_ack = false;
  	bridgeParams->resp_size = 16;
  	Range<long long unsigned int> rb1("0x10000000:+0xEFFFFFF");
  	bridgeParams->ranges.push_back(rb1);
  	bridgeParams->delay = 50*clk1.value();//50000;
  	bridgeParams->req_size = 16;
  	bridgeParams->nack_delay = 4*clk1.value();//4000;
	Bridge * bridge = bridgeParams->create();
	objects.push_back((SimObject*)bridge);
	
	A9SCUParams * a9scuParams = new A9SCUParams();
	a9scuParams->name=std::string(name())+".realview.a9scu";
	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
		a9scuParams->system=linuxCortexA15;
	else
		a9scuParams->system = cortexA15;
    a9scuParams->pio_addr = 0x1f000000;
    a9scuParams->pio_latency = clk1.value();//1000;
	A9SCU * a9scu = a9scuParams->create();
	objects.push_back((SimObject*)a9scu);
	
	AmbaFakeParams * ambaFakeParams = new AmbaFakeParams();
	ambaFakeParams->name=std::string(name())+".realview.mmc_fake";
	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
		ambaFakeParams->system=linuxCortexA15;
	else
		ambaFakeParams->system = cortexA15;
	ambaFakeParams->pio_addr = 0x10005000;
	ambaFakeParams->pio_latency = clk1.value();//1000;
	ambaFakeParams->amba_id = 0;
	ambaFakeParams->ignore_access = false;
	AmbaFake * mmc_fake = ambaFakeParams->create();
	objects.push_back((SimObject*)mmc_fake);
	
	ambaFakeParams = new AmbaFakeParams();
	ambaFakeParams->name=std::string(name())+".realview.watchdog_fake";

	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
		ambaFakeParams->system=linuxCortexA15;
	else
		ambaFakeParams->system = cortexA15;
	ambaFakeParams->pio_addr = 0x10010000;
	ambaFakeParams->pio_latency = clk1.value();//1000;
	ambaFakeParams->amba_id = 0;
	ambaFakeParams->ignore_access = false;
	AmbaFake * watchdog_fake = ambaFakeParams->create();
	objects.push_back((SimObject*)watchdog_fake);
	
	ambaFakeParams = new AmbaFakeParams();
	ambaFakeParams->name=std::string(name())+".realview.gpio0_fake";

	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
		ambaFakeParams->system=linuxCortexA15;
	else
		ambaFakeParams->system = cortexA15;
	ambaFakeParams->pio_addr = 0x10013000;
	ambaFakeParams->pio_latency = clk1.value();//1000;
	ambaFakeParams->amba_id = 0;
	ambaFakeParams->ignore_access = false;
	AmbaFake * gpio0_fake = ambaFakeParams->create();
	objects.push_back((SimObject*)gpio0_fake);
	
	ambaFakeParams = new AmbaFakeParams();
	ambaFakeParams->name=std::string(name())+".realview.gpio1_fake";

	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
		ambaFakeParams->system=linuxCortexA15;
	else
		ambaFakeParams->system = cortexA15;
	ambaFakeParams->pio_addr = 0x10014000;
	ambaFakeParams->pio_latency = clk1.value();//1000;
	ambaFakeParams->amba_id = 0;
	ambaFakeParams->ignore_access = false;
	AmbaFake * gpio1_fake = ambaFakeParams->create();
	objects.push_back((SimObject*)gpio1_fake);
	
	ambaFakeParams = new AmbaFakeParams();
	ambaFakeParams->name=std::string(name())+".realview.gpio2_fake";

	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
		ambaFakeParams->system=linuxCortexA15;
	else
		ambaFakeParams->system = cortexA15;
	ambaFakeParams->pio_addr = 0x10015000;
	ambaFakeParams->pio_latency = clk1.value();//1000;
	ambaFakeParams->amba_id = 0;
	ambaFakeParams->ignore_access = false;
	AmbaFake * gpio2_fake = ambaFakeParams->create();
	objects.push_back((SimObject*)gpio2_fake);
	
	ambaFakeParams = new AmbaFakeParams();
	ambaFakeParams->name=std::string(name())+".realview.dmac_fake";

	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
		ambaFakeParams->system=linuxCortexA15;
	else
		ambaFakeParams->system = cortexA15;
	ambaFakeParams->pio_addr = 0x10030000;
	ambaFakeParams->pio_latency = clk1.value();//1000;
	ambaFakeParams->amba_id = 0;
	ambaFakeParams->ignore_access = false;
	AmbaFake * dmac_fake = ambaFakeParams->create();
	objects.push_back((SimObject*)dmac_fake);

	ambaFakeParams = new AmbaFakeParams();
	ambaFakeParams->name=std::string(name())+".realview.aaci_fake";

	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
		ambaFakeParams->system=linuxCortexA15;
	else
		ambaFakeParams->system = cortexA15;
	ambaFakeParams->pio_addr = 0x10004000;
	ambaFakeParams->pio_latency = clk1.value();//1000;
	ambaFakeParams->amba_id = 0;
	ambaFakeParams->ignore_access = false;
	AmbaFake * aaci_fake = ambaFakeParams->create();
	objects.push_back((SimObject*)aaci_fake);

	ambaFakeParams = new AmbaFakeParams();
	ambaFakeParams->name=std::string(name())+".realview.ssp_fake";

	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
		ambaFakeParams->system=linuxCortexA15;
	else
		ambaFakeParams->system = cortexA15;
	ambaFakeParams->pio_addr = 0x1000d000;
	ambaFakeParams->pio_latency = clk1.value();//1000;
	ambaFakeParams->amba_id = 0;
	ambaFakeParams->ignore_access = false;
	AmbaFake * ssp_fake = ambaFakeParams->create();
	objects.push_back((SimObject*)ssp_fake);
	
	ambaFakeParams = new AmbaFakeParams();
	ambaFakeParams->name=std::string(name())+".realview.rtc_fake";

	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
		ambaFakeParams->system=linuxCortexA15;
	else
		ambaFakeParams->system = cortexA15;
	ambaFakeParams->pio_addr = 0x10017000;
	ambaFakeParams->pio_latency = clk1.value();//1000;
	ambaFakeParams->amba_id = 0;
	ambaFakeParams->ignore_access = false;
	AmbaFake * rtc_fake = ambaFakeParams->create();
	objects.push_back((SimObject*)rtc_fake);
	
	ambaFakeParams = new AmbaFakeParams();
	ambaFakeParams->name=std::string(name())+".realview.smc_fake";

	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
		ambaFakeParams->system=linuxCortexA15;
	else
		ambaFakeParams->system = cortexA15;
	ambaFakeParams->pio_addr = 0x100e1000;
	ambaFakeParams->pio_latency = clk1.value();//1000;
	ambaFakeParams->amba_id = 0;
	ambaFakeParams->ignore_access = false;
	AmbaFake * smc_fake = ambaFakeParams->create();
	objects.push_back((SimObject*)smc_fake);
	
	ambaFakeParams = new AmbaFakeParams();
	ambaFakeParams->name=std::string(name())+".realview.sp810_fake";

	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
		ambaFakeParams->system=linuxCortexA15;
	else
		ambaFakeParams->system = cortexA15;
	ambaFakeParams->pio_addr = 0x10001000;
	ambaFakeParams->pio_latency = clk1.value();//1000;
	ambaFakeParams->amba_id = 0;
	ambaFakeParams->ignore_access = true;
	AmbaFake * sp810_fake = ambaFakeParams->create();
	objects.push_back((SimObject*)sp810_fake);

	ambaFakeParams = new AmbaFakeParams();
	ambaFakeParams->name=std::string(name())+".realview.sci_fake";

	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
		ambaFakeParams->system=linuxCortexA15;
	else
		ambaFakeParams->system = cortexA15;
	ambaFakeParams->pio_addr = 0x1000e000;
	ambaFakeParams->pio_latency = clk1.value();//1000;
	ambaFakeParams->amba_id = 0;
	ambaFakeParams->ignore_access = false;
	AmbaFake * sci_fake = ambaFakeParams->create();
	objects.push_back((SimObject*)sci_fake);

	GicParams * gicParams = new GicParams();
	gicParams->name=std::string(name())+".realview.gic";
	gicParams->platform=(Platform*)rv;
	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
		gicParams->system=linuxCortexA15;
	else
		gicParams->system = cortexA15;
    gicParams->it_lines = 128;
    gicParams->dist_addr = 0x1f001000;
    gicParams->cpu_pio_delay = 10*clk1.value();//10000;
    gicParams->dist_pio_delay = 10*clk1.value();//10000;
    gicParams->cpu_addr = 0x1f000100;
    gicParams->int_latency = 10*clk1.value();//10000;
	Gic * gic = gicParams->create();
	objects.push_back((SimObject*)gic);
	
	CpuLocalTimerParams * cpuLocalTimerParams = new CpuLocalTimerParams();
	cpuLocalTimerParams->name=std::string(name())+".realview.local_cpu_timer";
	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
		cpuLocalTimerParams->system=linuxCortexA15;
	else
		cpuLocalTimerParams->system = cortexA15;
	cpuLocalTimerParams->pio_addr = 0x1f000600;
    cpuLocalTimerParams->pio_latency = clk1.value();//1000;
  	cpuLocalTimerParams->gic = gic;
	cpuLocalTimerParams->int_num_timer = 29;
	cpuLocalTimerParams->int_num_watchdog = 30;
	cpuLocalTimerParams->clock = clk1.value();//1000;
	CpuLocalTimer * local_cpu_timer = cpuLocalTimerParams->create();
	objects.push_back((SimObject*)local_cpu_timer);

	IdeControllerParams * ideControllerParams = new IdeControllerParams();
	ideControllerParams->name=std::string(name())+".realview.cf_ctrl";
	ideControllerParams->platform=(Platform*)rv;
	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
	{
		ideControllerParams->system=linuxCortexA15;
		ideControllerParams->disks.push_back(ideDisk);
	}
	else
		ideControllerParams->system = cortexA15;
	ideControllerParams->min_backoff_delay = 4*clk1.value();//4000;
	ideControllerParams->max_backoff_delay = 10000*clk1.value();//10000000;
    ideControllerParams->VendorID = 32902, 
    ideControllerParams->InterruptPin = 1;
    ideControllerParams->HeaderType = 0;
    ideControllerParams->pci_dev = 7;
    ideControllerParams->BAR5Size = 0;
    ideControllerParams->SubsystemID = 0;
    ideControllerParams->CardbusCIS = 0;
    ideControllerParams->MinimumGrant = 0;
    ideControllerParams->Revision = 0;
    ideControllerParams->Status = 640;
    ideControllerParams->MaximumLatency = 0;
    ideControllerParams->pio_latency = clk1.value();//1000;
    ideControllerParams->LatencyTimer = 0;
    ideControllerParams->BAR0LegacyIO = true;
    ideControllerParams->BAR1LegacyIO = true;
    ideControllerParams->BAR2LegacyIO = false;
    ideControllerParams->BAR4LegacyIO = false; 
    ideControllerParams->BAR5LegacyIO = false; 
    ideControllerParams->SubsystemVendorID = 0;
    ideControllerParams->Command = 1;
    ideControllerParams->DeviceID = 28945;
    ideControllerParams->ProgIF = 133;
    ideControllerParams->SubClassCode = 1;
    ideControllerParams->pci_func = 0;
    ideControllerParams->BAR0 = 0x18000000;
    ideControllerParams->BAR1 = 0x18000100;
    ideControllerParams->BAR2 = 1;
    ideControllerParams->BAR3 = 1;
    ideControllerParams->BAR4 = 1; 
    ideControllerParams->BAR5 = 1; 
    ideControllerParams->ClassCode = 1;
    ideControllerParams->BIST = 0; 
    ideControllerParams->CacheLineSize = 0;
    ideControllerParams->BAR0Size = 16;
    ideControllerParams->BAR1Size = 1;
    ideControllerParams->BAR2Size = 8;
    ideControllerParams->BAR3Size = 4;
    ideControllerParams->BAR4Size = 16;
    ideControllerParams->ExpansionROM = 0;
    ideControllerParams->BAR3LegacyIO = false;
    ideControllerParams->config_latency = 20*clk1.value();//20000;
    ideControllerParams->pci_bus = 2;
    ideControllerParams->InterruptLine = 31;
    ideControllerParams->io_shift = 1;
    ideControllerParams->ctrl_offset = 2;
	IdeController * cf_ctrl = ideControllerParams->create();
	objects.push_back((SimObject*)cf_ctrl);

	Sp804Params * sp804Params = new Sp804Params();
	sp804Params->name=std::string(name())+".realview.timer1";
	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
		sp804Params->system=linuxCortexA15;
	else
		sp804Params->system = cortexA15;
    sp804Params->pio_addr = 0x10012000;
    sp804Params->pio_latency = clk1.value();//1000;
    sp804Params->amba_id = 1316868;
    sp804Params->clock0 = 1000000;
    sp804Params->clock1 = 1000000;
    sp804Params->int_num0 = 37; 
    sp804Params->int_num1 = 37;
    sp804Params->gic = gic;
	Sp804 * timer1 = sp804Params->create();
	objects.push_back((SimObject*)timer1);

	sp804Params = new Sp804Params();
	sp804Params->name=std::string(name())+".realview.timer0";
	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
		sp804Params->system=linuxCortexA15;
	else
		sp804Params->system = cortexA15;
    sp804Params->pio_addr = 0x10011000;
    sp804Params->pio_latency = clk1.value();//1000;
    sp804Params->amba_id = 1316868;
    sp804Params->clock0 = 1000000;
    sp804Params->clock1 = 1000000;
    sp804Params->int_num0 = 36; 
    sp804Params->int_num1 = 36;
    sp804Params->gic = gic;
	Sp804 * timer0 = sp804Params->create();
	objects.push_back((SimObject*)timer0);
	
	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
	{
		VncServerParams * vncServerParams = new VncServerParams();
		vncServerParams->name=std::string(name())+".vncserver";
		vncServerParams->number = 0;
		vncServerParams->port = 5900;
		vncserver = vncServerParams->create();
		objects.push_back((SimObject*)vncserver);

		Pl050Params * pl050Params = new Pl050Params();
		pl050Params->name=std::string(name())+".realview.kmi1";
		pl050Params->system = linuxCortexA15;
		pl050Params->pio_addr = 0x10007000;
		pl050Params->pio_latency = 1000;
		pl050Params->amba_id = 1314896;
		pl050Params->int_num = 53;
		pl050Params->gic = gic;
		pl050Params->int_delay = 1000*clk1.value();//1000000;
		pl050Params->is_mouse = true;
		pl050Params->vnc = vncserver;
		kmi1 = pl050Params->create();
		objects.push_back((SimObject*)kmi1);

		pl050Params = new Pl050Params();
		pl050Params->name=std::string(name())+".realview.kmi0";
		pl050Params->system = linuxCortexA15;
		pl050Params->pio_addr = 0x10006000;
		pl050Params->pio_latency = 1000;
		pl050Params->amba_id = 1314896;
		pl050Params->int_num = 52;
		pl050Params->gic = gic;
		pl050Params->int_delay = 1000*clk1.value();//1000000;
		pl050Params->is_mouse = false;
		pl050Params->vnc = vncserver;
		kmi0 = pl050Params->create();
		objects.push_back((SimObject*)kmi0);

		Pl111Params * pl111Params = new Pl111Params();
		pl111Params->name=std::string(name())+".realview.clcd";
		pl111Params->system = linuxCortexA15;
		pl111Params->min_backoff_delay = 4*clk1.value();//4000;
		pl111Params->max_backoff_delay = 10000*clk1.value();//10000000;
		pl111Params->int_num = 55;
		pl111Params->pio_addr = 0x10020000;
		pl111Params->pio_latency = 10*clk1.value();//10000;
		pl111Params->amba_id = 1315089,
		pl111Params->gic = gic;
		pl111Params->vnc = vncserver;
		pl111Params->clock = 41667;
		clcd = pl111Params->create();
		objects.push_back((SimObject*)clcd);
	}
	
	RealViewCtrlParams * realViewCtrlParams = new RealViewCtrlParams();
	realViewCtrlParams->name=std::string(name())+".realview.realview_io";
	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
		realViewCtrlParams->system=linuxCortexA15;
	else
		realViewCtrlParams->system = cortexA15;
    realViewCtrlParams->pio_addr = 0x10000000;
    realViewCtrlParams->pio_latency = clk1.value();//1000;
  	realViewCtrlParams->proc_id0 = 201326592; 
  	realViewCtrlParams->proc_id1 = 201327138;
  	realViewCtrlParams->idreg = 0;
	RealViewCtrl * realview_io = realViewCtrlParams->create();
	objects.push_back((SimObject*)realview_io);
	
	objects.push_back((SimObject*)toACEbridge);
	
	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
		connectPorts((SimObject*)linuxCortexA15,"system_port",1,(SimObject*)toACEbridge,"func",1);
	else
		connectPorts((SimObject*)cortexA15,"system_port",1,(SimObject*)toACEbridge,"func",1);
		

	////////////////////////////////////////
	// construct num core dependent stuff //
	////////////////////////////////////////
	
	TerminalParams * termParams;
	Pl011Params * pl011Params;
	DerivO3CPUParams * cpuParams;
	ArmTableWalkerParams * tableWalkerParams;
	ArmTLBParams * tlbParams;
	ExeTracerParams * traceParams;
	OpDescParams * opdescParams;
	ArmInterruptsParams * intParams;

	Terminal* term[MAX_CORES];
	Pl011 * uart[MAX_CORES];
	ArmISA::TableWalker * dtb_walker[MAX_CORES];
	ArmISA::TableWalker * itb_walker[MAX_CORES];
	ArmISA::TableWalker * linux_dtb_walker[MAX_CORES];
	ArmISA::TableWalker * linux_itb_walker[MAX_CORES];
	FUDesc * O3v7a_Load[MAX_CORES];
	FUDesc * O3v7a_Store[MAX_CORES];
	OpDesc * opdesc;
#ifdef USE_A15
	FUDesc * O3v7a_CX[MAX_CORES];
	FUDesc * O3v7a_MX[MAX_CORES];
	FUDesc * O3v7a_IX[MAX_CORES];
	FUPool * O3v7a_FUP[MAX_CORES];
#else
	FUDesc * FUList0[MAX_CORES];
	FUDesc * FUList1[MAX_CORES];
	FUDesc * FUList2[MAX_CORES];
	FUDesc * FUList3[MAX_CORES];
	FUDesc * FUList4[MAX_CORES];
	FUDesc * FUList5[MAX_CORES];
	FUDesc * FUList6[MAX_CORES];
	FUDesc * FUList7[MAX_CORES];
	FUDesc * FUList8[MAX_CORES];
#endif
	FUPool * fuPool[MAX_CORES];
	
	for (uint32_t n=0;n<_num_cores;n++)
	{
		// core number
		std::stringstream ss;
		ss.str("");
		if (_num_cores > 1)
			ss << n;
		
		// Terminals
	
		termParams = new TerminalParams();
		if ((_system_cfg == "linuxRestore") && (n == 0))
			termParams->name=std::string(name())+".terminal";
		else if (n == 0)
			termParams->name=std::string(name())+".terminal_0";
		else
			termParams->name=std::string(name())+".terminal_"+ss.str();
		termParams->port = 3456+n;
    	termParams->intr_control = intc;
    	termParams->number = n;
    	termParams->output = true;
    	term[n] = termParams->create();
    	objects.push_back((SimObject*)term[n]);
    	
    	// Uart
		    	
		pl011Params = new Pl011Params();
		if ((_system_cfg == "linuxRestore") && (n == 0))
			pl011Params->name=std::string(name())+".realview.uart";
		else
			pl011Params->name=std::string(name())+".realview.uart"+ss.str();
		pl011Params->platform=(Platform*)rv;
		if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
			pl011Params->system=linuxCortexA15;
		else
			pl011Params->system = cortexA15;
		if (_gem5_uart_addr_cfg)
			pl011Params->pio_addr = 0x10009000+n*0x1000;
		else
			pl011Params->pio_addr = UART_BADDR+n*0x100000;//0x10009000+n*0x1000;
		pl011Params->pio_latency = clk1.value();//1000;
		pl011Params->terminal = term[n];
		pl011Params->int_num = 44+n;
		pl011Params->gic = gic;
		pl011Params->end_on_eot = true;
		pl011Params->int_delay = 100*clk1.value();//100000;
		uart[n] = pl011Params->create();
		objects.push_back((SimObject*)uart[n]);
		
		// TLBs
		if (_system_cfg != "linux")
		{
			tableWalkerParams = new ArmTableWalkerParams();
			if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
				tableWalkerParams->name=std::string(name())+".cpu"+ss.str()+".dtb.walker_";
			else
				tableWalkerParams->name=std::string(name())+".cpu"+ss.str()+".dtb.walker";
			tableWalkerParams->max_backoff = 100000;
			if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
				tableWalkerParams->sys=linuxCortexA15;
			else
				tableWalkerParams->sys = cortexA15;
			tableWalkerParams->min_backoff = 0;
			tableWalkerParams->port_port_connection_count=1;
			dtb_walker[n] = tableWalkerParams->create();
			objects.push_back((SimObject*)dtb_walker[n]);

			tlbParams = new ArmTLBParams();
			if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
				tlbParams->name=std::string(name())+".cpu"+ss.str()+".dtb_";
			else
				tlbParams->name=std::string(name())+".cpu"+ss.str()+".dtb";
			tlbParams->size=64; // 32 read + 32 write
			tlbParams->walker=dtb_walker[n];
			dtlb[n] = tlbParams->create();
			objects.push_back((SimObject*)dtlb[n]);

			tableWalkerParams = new ArmTableWalkerParams();
			if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
				tableWalkerParams->name=std::string(name())+".cpu"+ss.str()+".itb.walker_";
			else
				tableWalkerParams->name=std::string(name())+".cpu"+ss.str()+".itb.walker";
			tableWalkerParams->max_backoff = 100000;
			if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
				tableWalkerParams->sys=linuxCortexA15;
			else
				tableWalkerParams->sys = cortexA15;
			tableWalkerParams->min_backoff = 0;
			tableWalkerParams->port_port_connection_count=1;
			itb_walker[n] = tableWalkerParams->create();
			objects.push_back((SimObject*)itb_walker[n]);

			tlbParams = new ArmTLBParams();
			if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
				tlbParams->name=std::string(name())+".cpu"+ss.str()+".itb_";
			else
				tlbParams->name=std::string(name())+".cpu"+ss.str()+".itb";
			tlbParams->size=32;
			tlbParams->walker=itb_walker[n];
			itlb[n] = tlbParams->create();
			objects.push_back((SimObject*)itlb[n]);

			baseCacheParams = new BaseCacheParams();
			baseCacheParams->name=std::string(name())+".cpu"+ss.str()+".xtb_walker_cache";
			baseCacheParams->addr_ranges.push_back(rb);
			baseCacheParams->assoc = 4;
			baseCacheParams->block_size = 64;
			baseCacheParams->forward_snoops = false;
			baseCacheParams->hash_delay = 1;
			baseCacheParams->is_top_level = true;
			baseCacheParams->latency = 4*clk1.value();//4000;
			baseCacheParams->max_miss_count = 0;
			baseCacheParams->mshrs = 6;
			baseCacheParams->prefetch_on_access = false;
			baseCacheParams->prefetcher = NULL;
			baseCacheParams->prioritizeRequests = false;
			baseCacheParams->repl = NULL;
			baseCacheParams->size = 8*1024; // 512 entries, 4 ways, 32bit descriptors
			baseCacheParams->subblock_size = 0;
			if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
				baseCacheParams->system=linuxCortexA15;
			else
				baseCacheParams->system = cortexA15;
			baseCacheParams->tgts_per_mshr = 8;
			baseCacheParams->trace_addr = 0;
			baseCacheParams->two_queue = false;
			baseCacheParams->write_buffers = 16; // ????
			xtb_walker_cache[n] = baseCacheParams->create();
			objects.push_back((SimObject*)xtb_walker_cache[n]);
		}
		
		// tracer
		
		traceParams = new ExeTracerParams();
		traceParams->name=std::string(name())+".cpu"+ss.str()+".tracer";
		tracer[n] = traceParams->create();
		objects.push_back((SimObject*)tracer[n]);

		// interrupts

		intParams = new ArmInterruptsParams();
		intParams->name=std::string(name())+".cpu"+ss.str()+".interrupts";
		interrupts[n] = intParams->create();
		objects.push_back((SimObject*)interrupts[n]);
			

		// Instructions

#ifdef USE_A15

		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList0.opList";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 2; 
		opdescParams->opClass = Enums::MemRead;
		FUDescParams* fudescParams = new FUDescParams();
		fudescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList0";
		fudescParams->count=1;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		O3v7a_Load[n] = fudescParams->create();
		objects.push_back((SimObject*)O3v7a_Load[n]);

		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList1.opList";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 2; 
		opdescParams->opClass = Enums::MemWrite;
		fudescParams = new FUDescParams();
		fudescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList1";
		fudescParams->count=1;
		fudescParams->opList.clear();
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		O3v7a_Store[n] = fudescParams->create();
		objects.push_back((SimObject*)O3v7a_Store[n]);

		fudescParams = new FUDescParams();
		fudescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2";
		fudescParams->count=2;
		fudescParams->opList.clear();
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList00";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 4; 
		opdescParams->opClass = Enums::SimdAdd;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList01";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 4; 
		opdescParams->opClass = Enums::SimdAddAcc;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList02";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 4; 
		opdescParams->opClass = Enums::SimdAlu;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList03";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 4; 
		opdescParams->opClass = Enums::SimdCmp;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList04";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 3; 
		opdescParams->opClass = Enums::SimdCvt;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList05";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 3; 
		opdescParams->opClass = Enums::SimdMisc;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList06";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 5; 
		opdescParams->opClass = Enums::SimdMult;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList07";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 5; 
		opdescParams->opClass = Enums::SimdMultAcc;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList08";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 3; 
		opdescParams->opClass = Enums::SimdShift;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList09";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 3; 
		opdescParams->opClass = Enums::SimdShiftAcc;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList10";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 9; 
		opdescParams->opClass = Enums::SimdSqrt;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList11";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 5; 
		opdescParams->opClass = Enums::SimdFloatAdd;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList12";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 5; 
		opdescParams->opClass = Enums::SimdFloatAlu;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList13";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 3; 
		opdescParams->opClass = Enums::SimdFloatCmp;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList14";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 3; 
		opdescParams->opClass = Enums::SimdFloatCvt;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList15";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 3; 
		opdescParams->opClass = Enums::SimdFloatDiv;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList16";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 3; 
		opdescParams->opClass = Enums::SimdFloatMisc;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList17";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 3; 
		opdescParams->opClass = Enums::SimdFloatMult;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList18";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 1; 
		opdescParams->opClass = Enums::SimdFloatMultAcc;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList19";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 9; 
		opdescParams->opClass = Enums::SimdFloatSqrt;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList20";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 5; 
		opdescParams->opClass = Enums::FloatAdd;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList21";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 5; 
		opdescParams->opClass = Enums::FloatCmp;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList22";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 5; 
		opdescParams->opClass = Enums::FloatCvt;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList23";
		opdescParams->issueLat = 9;
		opdescParams->opLat = 9; 
		opdescParams->opClass = Enums::FloatDiv;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList24";
		opdescParams->issueLat = 33;
		opdescParams->opLat = 33; 
		opdescParams->opClass = Enums::FloatSqrt;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList25";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 4; 
		opdescParams->opClass = Enums::FloatMult;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		O3v7a_CX[n] = fudescParams->create();
		objects.push_back((SimObject*)O3v7a_CX[n]);

		fudescParams = new FUDescParams();
		fudescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList3";
		fudescParams->count=1;
		fudescParams->opList.clear();
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList3.opList0";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 3; 
		opdescParams->opClass = Enums::IntMult;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList3.opList1";
		opdescParams->issueLat = 12;
		opdescParams->opLat = 12; 
		opdescParams->opClass = Enums::IntDiv;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList3.opList2";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 3; 
		opdescParams->opClass = Enums::IprAccess;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		O3v7a_MX[n] = fudescParams->create();
		objects.push_back((SimObject*)O3v7a_MX[n]);

		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList4.opList";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 1; 
		opdescParams->opClass = Enums::IntAlu;
		fudescParams = new FUDescParams();
		fudescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList4";
		fudescParams->count=2;
		fudescParams->opList.clear();
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		O3v7a_IX[n] = fudescParams->create();
		objects.push_back((SimObject*)O3v7a_IX[n]);

		FUPoolParams * fupoolParams = new FUPoolParams();
		fupoolParams->name=std::string(name())+".cpu"+ss.str()+".fuPool";
		fupoolParams->FUList.push_back(O3v7a_Load[n]);
		fupoolParams->FUList.push_back(O3v7a_Store[n]);
		fupoolParams->FUList.push_back(O3v7a_CX[n]);
		fupoolParams->FUList.push_back(O3v7a_MX[n]);
		fupoolParams->FUList.push_back(O3v7a_IX[n]);
		O3v7a_FUP[n] = fupoolParams->create();
		objects.push_back((SimObject*)O3v7a_FUP[n]);

#else

		FUDescParams* fudescParams = new FUDescParams();
		fudescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList0";
		fudescParams->count=6;

		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList0.opList";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 1; 
		opdescParams->opClass = Enums::IntAlu;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		
		FUList0[n] = fudescParams->create();
		objects.push_back((SimObject*)FUList0[n]);

		fudescParams = new FUDescParams();
		fudescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList1";
		fudescParams->count=2;

		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList1.opList0";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 3; 
		opdescParams->opClass = Enums::IntMult;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);

		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList1.opList1";
		opdescParams->issueLat = 19;
		opdescParams->opLat = 20; 
		opdescParams->opClass = Enums::IntDiv;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		
		FUList1[n] = fudescParams->create();
		objects.push_back((SimObject*)FUList1[n]);		

		fudescParams = new FUDescParams();
		fudescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2";
		fudescParams->count=4;

		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList0";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 2; 
		opdescParams->opClass = Enums::FloatAdd;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList1";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 2; 
		opdescParams->opClass = Enums::FloatCmp;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList2.opList2";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 2; 
		opdescParams->opClass = Enums::FloatCvt;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		
		FUList2[n] = fudescParams->create();
		objects.push_back((SimObject*)FUList2[n]);
		
		fudescParams = new FUDescParams();
		fudescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList3";
		fudescParams->count=2;
		
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList3.opList0";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 4; 
		opdescParams->opClass = Enums::FloatMult;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList3.opList1";
		opdescParams->issueLat = 12;
		opdescParams->opLat = 12; 
		opdescParams->opClass = Enums::FloatDiv;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList3.opList2";
		opdescParams->issueLat = 24;
		opdescParams->opLat = 24; 
		opdescParams->opClass = Enums::FloatSqrt;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		
		FUList3[n] = fudescParams->create();
		objects.push_back((SimObject*)FUList3[n]);

		fudescParams = new FUDescParams();
		fudescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList4";
		fudescParams->count=0;
		
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList4.opList";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 1; 
		opdescParams->opClass = Enums::MemRead;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		
		FUList4[n] = fudescParams->create();
		objects.push_back((SimObject*)FUList4[n]);
		fudescParams->opList.push_back(opdesc);

		fudescParams = new FUDescParams();
		fudescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList5";
		fudescParams->count=4;
		
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList5.opList00";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 1; 
		opdescParams->opClass = Enums::SimdAdd;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList5.opList01";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 1; 
		opdescParams->opClass = Enums::SimdAddAcc;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList5.opList02";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 1; 
		opdescParams->opClass = Enums::SimdAlu;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList5.opList03";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 1; 
		opdescParams->opClass = Enums::SimdCmp;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList5.opList04";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 1; 
		opdescParams->opClass = Enums::SimdCvt;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList5.opList05";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 1; 
		opdescParams->opClass = Enums::SimdMisc;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList5.opList06";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 1; 
		opdescParams->opClass = Enums::SimdMult;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList5.opList07";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 1; 
		opdescParams->opClass = Enums::SimdMultAcc;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList5.opList08";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 1; 
		opdescParams->opClass = Enums::SimdShift;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList5.opList09";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 1; 
		opdescParams->opClass = Enums::SimdShiftAcc;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList5.opList10";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 1; 
		opdescParams->opClass = Enums::SimdSqrt;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList5.opList11";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 1; 
		opdescParams->opClass = Enums::SimdFloatAdd;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList5.opList12";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 1; 
		opdescParams->opClass = Enums::SimdFloatAlu;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList5.opList13";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 1; 
		opdescParams->opClass = Enums::SimdFloatCmp;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList5.opList14";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 1; 
		opdescParams->opClass = Enums::SimdFloatCvt;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList5.opList15";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 1; 
		opdescParams->opClass = Enums::SimdFloatDiv;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList5.opList16";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 1; 
		opdescParams->opClass = Enums::SimdFloatMisc;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList5.opList17";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 1; 
		opdescParams->opClass = Enums::SimdFloatMult;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList5.opList18";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 1; 
		opdescParams->opClass = Enums::SimdFloatMultAcc;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList5.opList19";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 1; 
		opdescParams->opClass = Enums::SimdFloatSqrt;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);
		
		FUList5[n] = fudescParams->create();
		objects.push_back((SimObject*)FUList5[n]);
		fudescParams->opList.push_back(opdesc);

		fudescParams = new FUDescParams();
		fudescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList6";
		fudescParams->count=0;
		
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList6.opList";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 1; 
		opdescParams->opClass = Enums::MemWrite;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);

		FUList6[n] = fudescParams->create();
		objects.push_back((SimObject*)FUList6[n]);
		fudescParams->opList.push_back(opdesc);

		fudescParams = new FUDescParams();
		fudescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList7";
		fudescParams->count=4;
		
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList7.opList0";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 1; 
		opdescParams->opClass = Enums::MemRead;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);

		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList7.opList1";
		opdescParams->issueLat = 1;
		opdescParams->opLat = 1; 
		opdescParams->opClass = Enums::MemWrite;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);

		FUList7[n] = fudescParams->create();
		objects.push_back((SimObject*)FUList7[n]);
		fudescParams->opList.push_back(opdesc);

		fudescParams = new FUDescParams();
		fudescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList8";
		fudescParams->count=1;
		
		opdescParams= new OpDescParams();
		opdescParams->name=std::string(name())+".cpu"+ss.str()+".fuPool.FUList8.opList";
		opdescParams->issueLat = 3;
		opdescParams->opLat = 3; 
		opdescParams->opClass = Enums::IprAccess;
		opdesc=opdescParams->create();
		objects.push_back((SimObject*)opdesc);
		fudescParams->opList.push_back(opdesc);

		FUList8[n] = fudescParams->create();
		objects.push_back((SimObject*)FUList8[n]);
		fudescParams->opList.push_back(opdesc);
		
		FUPoolParams * fupoolParams = new FUPoolParams();
		fupoolParams->name=std::string(name())+".cpu"+ss.str()+".fuPool";
		fupoolParams->FUList.push_back(FUList0[n]);
		fupoolParams->FUList.push_back(FUList1[n]);
		fupoolParams->FUList.push_back(FUList2[n]);
		fupoolParams->FUList.push_back(FUList3[n]);
		fupoolParams->FUList.push_back(FUList4[n]);
		fupoolParams->FUList.push_back(FUList5[n]);
		fupoolParams->FUList.push_back(FUList6[n]);
		fupoolParams->FUList.push_back(FUList7[n]);
		fupoolParams->FUList.push_back(FUList8[n]);
		fuPool[n] = fupoolParams->create();
		objects.push_back((SimObject*)fuPool[n]);

#endif

		//CPU
		
		
#ifdef USE_A15
		
		cpuParams = new DerivO3CPUParams();
		cpuParams->interrupts=interrupts[n];
		cpuParams->name=std::string(name())+".cpu"+ss.str();
		cpuParams->itb=itlb[n];
		cpuParams->dtb=dtlb[n];
		cpuParams->tracer=tracer[n];
		if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
		{
			cpuParams->system=linuxCortexA15;
			cpuParams->defer_registration=true;
			cpuParams->cpu_id = n+_num_cores;
		}
		else
		{
			cpuParams->system=cortexA15;
			cpuParams->defer_registration = false;
			cpuParams->cpu_id = n;
		}
		cpuParams->clock=clk1.value();//1000;
		cpuParams->do_statistics_insts = true;
		cpuParams->do_quiesce=true;
		cpuParams->renameWidth = 3;
		cpuParams->LSQDepCheckShift = 0;
		cpuParams->activity = 0;
		cpuParams->smtLSQThreshold = 100;
		cpuParams->BTBEntries = 2048; 
		cpuParams->dispatchWidth = 6;
		cpuParams->iewToRenameDelay = 1;
		cpuParams->numROBEntries = 40;
		cpuParams->squashWidth = 8;
		cpuParams->renameToROBDelay = 1;
		cpuParams->SQEntries = 16;
		cpuParams->SSITSize = 1024;
		cpuParams->fetchWidth = 3;
		cpuParams->backComSize = 5;
		cpuParams->smtCommitPolicy = "RoundRobin";
		cpuParams->commitToIEWDelay = 1;
		cpuParams->commitToDecodeDelay = 1; 
		cpuParams->decodeToRenameDelay = 2;
		cpuParams->fetchToDecodeDelay = 3;
		cpuParams->issueWidth = 8;
		cpuParams->LSQCheckLoads = true;
		cpuParams->globalCtrBits = 2;
		cpuParams->commitToRenameDelay = 1;
		cpuParams->choicePredictorSize = 8192;
		cpuParams->cachePorts = 200;
		cpuParams->renameToDecodeDelay = 1;
		cpuParams->smtFetchPolicy = "SingleThread";
		cpuParams->store_set_clear_period = 250*clk1.value();//250000;
		cpuParams->numPhysFloatRegs = 128;
		cpuParams->numPhysIntRegs = 128;
		cpuParams->RASSize = 16;
		cpuParams->wbDepth = 1;
		cpuParams->issueToExecuteDelay = 1; 
		cpuParams->predType = "tournament";
		cpuParams->smtROBThreshold = 100;
		cpuParams->smtNumFetchingThreads = 1;
		cpuParams->wbWidth = 8;
		cpuParams->commitToFetchDelay = 1;
		cpuParams->fetchTrapLatency = 1;
		cpuParams->fuPool = O3v7a_FUP[n]; 
		cpuParams->localHistoryTableSize = 64;
		cpuParams->decodeToFetchDelay = 1; 
		cpuParams->renameToFetchDelay = 1;
		cpuParams->decodeWidth = 3;
		cpuParams->trapLatency = 13;
		cpuParams->smtIQPolicy = "Partitioned";
		cpuParams->globalHistoryBits = 13;
		cpuParams->smtROBPolicy = "Partitioned";
		cpuParams->localPredictorSize = 64;
		cpuParams->choiceCtrBits = 2;
		cpuParams->numRobs = 1;
		cpuParams->localHistoryBits = 6;
		cpuParams->iewToDecodeDelay = 1; 
		cpuParams->smtLSQPolicy = "Partitioned";
		cpuParams->commitWidth = 8;
		cpuParams->globalPredictorSize = 8192;
		cpuParams->localCtrBits = 2;
		cpuParams->forwardComSize = 5;
		cpuParams->BTBTagSize = 18;
		cpuParams->numIQEntries = 32;
		cpuParams->LFSTSize = 1024;
		cpuParams->iewToCommitDelay = 1;
		cpuParams->renameToIEWDelay = 1; 
		cpuParams->iewToFetchDelay = 1;
		cpuParams->LQEntries = 16;
		cpuParams->smtIQThreshold = 100;
		cpuParams->function_trace = false;
		cpuParams->do_checkpoint_insts = true;
		cpuParams->max_loads_all_threads = 0;
		cpuParams->function_trace_start = 0;
		cpuParams->checker = 0x0;
		cpuParams->profile = 0;
		cpuParams->max_insts_all_threads = 0;
		cpuParams->phase = 0;
		cpuParams->progress_interval = 0;
		cpuParams->max_loads_any_thread = 0;
		cpuParams->max_insts_any_thread = 0;
		cpu[n] = (BaseO3CPU*)cpuParams->create();
		((BaseCPU*)cpu[n])->setClusterId(_cluster_id_cfg);
		cpus[cpuParams->name]=(FullO3CPU<O3CPUImpl>*)cpu[n];
		//objects.push_back((SimObject*)cpu[n]);
		
#else

		cpuParams = new DerivO3CPUParams();
		cpuParams->interrupts=interrupts[n];
		cpuParams->name=std::string(name())+".cpu"+ss.str();
		cpuParams->itb=itlb[n];
		cpuParams->dtb=dtlb[n];
		cpuParams->tracer=tracer[n];
		if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
			cpuParams->system=linuxCortexA15;
		else
			cpuParams->system=cortexA15;
		cpuParams->clock=clk1.value();//1000;
		cpuParams->cpu_id = n;
		cpuParams->BTBEntries=4096;
		cpuParams->BTBTagSize=16;
		cpuParams->LFSTSize=1024;
		cpuParams->LQEntries=32;
		cpuParams->LSQCheckLoads=true;
		cpuParams->LSQDepCheckShift=4;
		cpuParams->RASSize=16;
		cpuParams->SQEntries=32;
		cpuParams->SSITSize=1024;
		cpuParams->activity=0;
		cpuParams->backComSize=5;
		cpuParams->cachePorts=200;
		cpuParams->checker=NULL;
		cpuParams->choiceCtrBits=2;
		cpuParams->choicePredictorSize=8192;
		cpuParams->commitToDecodeDelay=1;
		cpuParams->commitToFetchDelay=1;
		cpuParams->commitToIEWDelay=1;
		cpuParams->commitToRenameDelay=1;
		cpuParams->commitWidth=8;
		cpuParams->decodeToFetchDelay=1;
		cpuParams->decodeToRenameDelay=1;
		cpuParams->decodeWidth=8;
		cpuParams->defer_registration=false;
		cpuParams->dispatchWidth=8;
		cpuParams->do_checkpoint_insts=true;
		cpuParams->do_quiesce=true;
		cpuParams->do_statistics_insts=true;
		cpuParams->fetchToDecodeDelay=1;
		cpuParams->fetchTrapLatency=1;
		cpuParams->fetchWidth=8;
		cpuParams->forwardComSize=5;
		cpuParams->fuPool= fuPool[n];
		cpuParams->function_trace=false;
		cpuParams->function_trace_start=0;
		cpuParams->globalCtrBits=2;
		cpuParams->globalHistoryBits=13;
		cpuParams->globalPredictorSize=8192;
		cpuParams->iewToCommitDelay=1;
		cpuParams->iewToDecodeDelay=1;
		cpuParams->iewToFetchDelay=1;
		cpuParams->iewToRenameDelay=1;
		cpuParams->instShiftAmt=2;
		cpuParams->issueToExecuteDelay=1;
		cpuParams->issueWidth=8;
		cpuParams->localCtrBits=2;
		cpuParams->localHistoryBits=11;
		cpuParams->localHistoryTableSize=2048;
		cpuParams->localPredictorSize=2048;
		cpuParams->max_insts_all_threads=0;
		cpuParams->max_insts_any_thread=0;
		cpuParams->max_loads_all_threads=0;
		cpuParams->max_loads_any_thread=0;
		cpuParams->numIQEntries=64;
		cpuParams->numPhysFloatRegs=256;
		cpuParams->numPhysIntRegs=256;
		cpuParams->numROBEntries=192;
		cpuParams->numRobs=1;
		cpuParams->numThreads=1;
		cpuParams->phase=0;
		cpuParams->predType="tournament";
		cpuParams->profile=0;
		cpuParams->progress_interval=0;
		cpuParams->renameToDecodeDelay=1;
		cpuParams->renameToFetchDelay=1;
		cpuParams->renameToIEWDelay=2;
		cpuParams->renameToROBDelay=1;
		cpuParams->renameWidth=8;
		cpuParams->smtCommitPolicy="RoundRobin";
		cpuParams->smtFetchPolicy="SingleThread";
		cpuParams->smtIQPolicy="Partitioned";
		cpuParams->smtIQThreshold=100;
		cpuParams->smtLSQPolicy="Partitioned";
		cpuParams->smtLSQThreshold=100;
		cpuParams->smtNumFetchingThreads=1;
		cpuParams->smtROBPolicy="Partitioned";
		cpuParams->smtROBThreshold=100;
		cpuParams->squashWidth=8;
		cpuParams->store_set_clear_period=250000;
		cpuParams->trapLatency=13;
		cpuParams->wbDepth=1;
		cpuParams->wbWidth=8;
		cpu[n] = cpuParams->create();
		cpus[cpuParams->name]=(FullO3CPU<O3CPUImpl>*)cpu[n];
		//objects.push_back((SimObject*)cpu[n]);


#endif
  		// Caches
  		
#ifdef USE_A15

		if (_system_cfg != "linux")
		{
			baseCacheParams = new BaseCacheParams();
			baseCacheParams->name=std::string(name())+".cpu"+ss.str()+".icache";
			baseCacheParams->addr_ranges.push_back(rb);
			baseCacheParams->assoc = 2;
			baseCacheParams->block_size = 64;
			baseCacheParams->forward_snoops = true;
			baseCacheParams->hash_delay = 1;
			baseCacheParams->is_top_level = true;
			baseCacheParams->latency = clk1.value();//1000;
			baseCacheParams->max_miss_count = 0;
			baseCacheParams->mshrs = 2;
			baseCacheParams->prefetch_on_access = false;
			baseCacheParams->prefetcher = NULL;
			baseCacheParams->prioritizeRequests = false;
			baseCacheParams->repl = NULL;
			baseCacheParams->size = 32768;
			baseCacheParams->subblock_size = 0;
			if (_system_cfg == "linuxRestore")
				baseCacheParams->system=linuxCortexA15;
			else
				baseCacheParams->system = cortexA15;
			baseCacheParams->tgts_per_mshr = 8;
			baseCacheParams->trace_addr = 0;
			baseCacheParams->two_queue = false;
			baseCacheParams->write_buffers = 8;
			icache[n] = baseCacheParams->create();
			objects.push_back((SimObject*)icache[n]);

			baseCacheParams = new BaseCacheParams();
			baseCacheParams->name=std::string(name())+".cpu"+ss.str()+".dcache";
			baseCacheParams->addr_ranges.push_back(rb);
			baseCacheParams->assoc = 2;
			baseCacheParams->block_size = 64;
			baseCacheParams->forward_snoops = true;
			baseCacheParams->hash_delay = 1;
			baseCacheParams->is_top_level = true;
			baseCacheParams->latency = 2*clk1.value();//2000;
			baseCacheParams->max_miss_count = 0;
			baseCacheParams->mshrs = 6;
			baseCacheParams->prefetch_on_access = false;
			baseCacheParams->prefetcher = NULL;
			baseCacheParams->prioritizeRequests = false;
			baseCacheParams->repl = NULL;
			baseCacheParams->size = 32768;
			baseCacheParams->subblock_size = 0;
			if (_system_cfg == "linuxRestore")
				baseCacheParams->system=linuxCortexA15;
			else
				baseCacheParams->system = cortexA15;
			baseCacheParams->tgts_per_mshr = 8;
			baseCacheParams->trace_addr = 0;
			baseCacheParams->two_queue = false;
			baseCacheParams->write_buffers = 16;
			dcache[n] = baseCacheParams->create();
			objects.push_back((SimObject*)dcache[n]);

#else

		baseCacheParams = new BaseCacheParams();
		baseCacheParams->name=std::string(name())+".cpu"+ss.str()+".icache";
		baseCacheParams->write_buffers = 8;
		baseCacheParams->is_top_level = true;
		baseCacheParams->block_size = 64;
		baseCacheParams->prefetch_latency = 10*clk1.value();//10000;
		baseCacheParams->size = 32768;
		baseCacheParams->latency = clk1.value();//1000;
		baseCacheParams->prefetch_past_page = false;
		baseCacheParams->addr_ranges.push_back(rb);
		baseCacheParams->num_cpus = 1;
		baseCacheParams->trace_addr = 0;
		baseCacheParams->max_miss_count = 0;
		baseCacheParams->mshrs = 10;
		baseCacheParams->forward_snoops = true;
		baseCacheParams->prefetch_data_accesses_only = false;
		baseCacheParams->tgts_per_mshr = 20;
		baseCacheParams->repl = 0x0;
		baseCacheParams->assoc = 2;
		baseCacheParams->prefetch_on_access = false;
		baseCacheParams->prioritizeRequests = false;
		baseCacheParams->prefetch_use_cpu_id = true;
		baseCacheParams->prefetch_serial_squash = false;
		baseCacheParams->prefetch_degree = 1;
		baseCacheParams->prefetch_policy = Enums::none;
		baseCacheParams->hash_delay = 1;
		baseCacheParams->subblock_size = 0;
		baseCacheParams->prefetcher_size = 100;
		baseCacheParams->two_queue = false;
		icache[n] = baseCacheParams->create();
		objects.push_back((SimObject*)icache[n]);

		baseCacheParams = new BaseCacheParams();
		baseCacheParams->name=std::string(name())+".cpu"+ss.str()+".dcache";
		baseCacheParams->write_buffers = 8;
		baseCacheParams->is_top_level = true;
		baseCacheParams->block_size = 64;
		baseCacheParams->prefetch_latency = 10*clk1.value();//20000;
		baseCacheParams->size = 65536;
		baseCacheParams->latency = 1*clk1.value();//2000;
		baseCacheParams->prefetch_past_page = false;
		baseCacheParams->addr_ranges.push_back(rb);
		baseCacheParams->num_cpus = 1;
		baseCacheParams->trace_addr = 0;
		baseCacheParams->max_miss_count = 0;
		baseCacheParams->mshrs = 10;
		baseCacheParams->forward_snoops = true;
		baseCacheParams->prefetch_data_accesses_only = false;
		baseCacheParams->tgts_per_mshr = 20;
		baseCacheParams->repl = NULL;
		baseCacheParams->assoc = 2;
		baseCacheParams->prefetch_on_access = false;
		baseCacheParams->prioritizeRequests = false;
		baseCacheParams->prefetch_use_cpu_id = true;
		baseCacheParams->prefetch_serial_squash = false;
		baseCacheParams->prefetch_degree = 1;
		baseCacheParams->prefetch_policy = Enums::none;
		baseCacheParams->hash_delay = 1;
		baseCacheParams->subblock_size = 0;
		baseCacheParams->prefetcher_size = 100;
		baseCacheParams->two_queue = false;
		dcache[n] = baseCacheParams->create();
		objects.push_back((SimObject*)dcache[n]);

#endif

			busParams = new BusParams();
			busParams->name=std::string(name())+".cpu"+ss.str()+".xtb_walker_cache_bus";
			busParams->clock = clk1.value();//1000;
			busParams->header_cycles = 1;
			busParams->width = 64;// bytes
			busParams->use_default_range = false;
			busParams->block_size = 64;
			busParams->bus_id = 0;// unused?
			busParams->port_master_connection_count=1;
			busParams->port_slave_connection_count=2;
			busParams->port_default_connection_count=0;
			xtb_walker_cache_bus[n] = busParams->create();
			objects.push_back((SimObject*)xtb_walker_cache_bus[n]);

		}
	}
	
	
	AmbaFake * uart_fake[MAX_CORES];
	for (uint32_t n=_num_cores;n<MAX_CORES;n++)
	{
		// core number
		std::stringstream ss;
		ss << n;

		//Fake Uart

		ambaFakeParams = new AmbaFakeParams();
		ambaFakeParams->name=std::string(name())+".realview.uart"+ss.str()+"_fake";
		if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
			ambaFakeParams->system=linuxCortexA15;
		else
			ambaFakeParams->system = cortexA15;
		if (_gem5_uart_addr_cfg)
			ambaFakeParams->pio_addr = 0x10009000+n*0x1000;
		else
			ambaFakeParams->pio_addr = UART_BADDR+n*0x100000;//0x10009000+n*0x1000;
		ambaFakeParams->pio_latency = clk1.value();//1000;
		ambaFakeParams->amba_id = 0;
		ambaFakeParams->ignore_access = false;
		uart_fake[n] = ambaFakeParams->create();
		objects.push_back((SimObject*)uart_fake[n]);
	}
	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
	{
		AtomicSimpleCPUParams* atomicSimpleCPUParams;
		// core number
		std::stringstream ss;
		for (uint32_t i=0;i<_num_cores;i++)
		{
			ss.str("");
			if (_num_cores > 1)
				ss << i;
			// TLBs
			ArmTableWalkerParams* tableWalkerParams = new ArmTableWalkerParams();
			tableWalkerParams->name="system.cpu"+ss.str()+".dtb.walker";
			tableWalkerParams->max_backoff = 100000;
			tableWalkerParams->sys=linuxCortexA15;
			tableWalkerParams->min_backoff = 0;
			linux_dtb_walker[i] = tableWalkerParams->create();
			objects.push_back((SimObject*)linux_dtb_walker[i]);

			ArmTLBParams* tlbParams = new ArmTLBParams();
			tlbParams->name="system.cpu"+ss.str()+".dtb";
			tlbParams->size=64;
			tlbParams->walker=linux_dtb_walker[i];
			ArmISA::TLB* dtlb_ = tlbParams->create();
			objects.push_back((SimObject*)dtlb_);

			tableWalkerParams = new ArmTableWalkerParams();
			tableWalkerParams->name="system.cpu"+ss.str()+".itb.walker";
			tableWalkerParams->max_backoff = 100000;
			tableWalkerParams->sys=linuxCortexA15;
			tableWalkerParams->min_backoff = 0;
			linux_itb_walker[i] = tableWalkerParams->create();
			objects.push_back((SimObject*)linux_itb_walker[i]);

			tlbParams = new ArmTLBParams();
			tlbParams->name="system.cpu"+ss.str()+".itb";
			tlbParams->size=64;
			tlbParams->walker=linux_itb_walker[i];
			ArmISA::TLB* itlb_ = tlbParams->create();
			objects.push_back((SimObject*)itlb_);

			atomicSimpleCPUParams = new AtomicSimpleCPUParams();
			atomicSimpleCPUParams->name="system.cpu"+ss.str();
			atomicSimpleCPUParams->checker=NULL;
			atomicSimpleCPUParams->clock=clk1.value();
			atomicSimpleCPUParams->cpu_id=i;
			atomicSimpleCPUParams->defer_registration=false;
			atomicSimpleCPUParams->do_checkpoint_insts=true;
			atomicSimpleCPUParams->do_quiesce=true;
			atomicSimpleCPUParams->do_statistics_insts=true;
			atomicSimpleCPUParams->dtb=dtlb_;
			atomicSimpleCPUParams->function_trace=false;
			atomicSimpleCPUParams->function_trace_start=0;
			atomicSimpleCPUParams->interrupts=interrupts[i];
			atomicSimpleCPUParams->itb=itlb_;
			atomicSimpleCPUParams->max_insts_all_threads=0;
			atomicSimpleCPUParams->max_insts_any_thread=0;
			atomicSimpleCPUParams->max_loads_all_threads=0;
			atomicSimpleCPUParams->max_loads_any_thread=0;
			atomicSimpleCPUParams->numThreads=1;
			atomicSimpleCPUParams->phase=0;
			atomicSimpleCPUParams->profile=0;
			atomicSimpleCPUParams->progress_interval=0;
			atomicSimpleCPUParams->simulate_data_stalls=false;
			atomicSimpleCPUParams->simulate_inst_stalls=false;
			atomicSimpleCPUParams->system=linuxCortexA15;
			atomicSimpleCPUParams->tracer=tracer[i];
			atomicSimpleCPUParams->width=1;
			dummy_cpu[i] = atomicSimpleCPUParams->create();
		}
	}

	
	/////////////////
	// Connections //
	/////////////////
	
	uint32_t tol2bus_mst_idx=0;
	uint32_t tol2bus_slv_idx=0;
	uint32_t iobus_mst_idx=0;
	uint32_t iobus_slv_idx=0;
	uint32_t membus_mst_idx=0;
	uint32_t membus_slv_idx=0;
	uint32_t xtb_walker_cache_bus_mst_idx[MAX_CORES]={0};
	uint32_t xtb_walker_cache_bus_slv_idx[MAX_CORES]={0};
	
	connectPorts((SimObject*)membus,"default",-1,(SimObject*)badaddr_responder,"pio",-1);
	connectPorts((SimObject*)membus,"master",membus_mst_idx++,(SimObject*)toACEbridge,"slave",-1);
	connectPorts((SimObject*)membus,"master",membus_mst_idx++,(SimObject*)gic,"pio",-1);
	connectPorts((SimObject*)membus,"master",membus_mst_idx++,(SimObject*)l2x0_fake,"pio",-1);
	connectPorts((SimObject*)membus,"master",membus_mst_idx++,(SimObject*)a9scu,"pio",-1);
	connectPorts((SimObject*)membus,"master",membus_mst_idx++,(SimObject*)local_cpu_timer,"pio",-1);
	connectPorts((SimObject*)membus,"master",membus_mst_idx++,(SimObject*)bridge,"slave",-1);
	if (_system_cfg != "linux")
		connectPorts((SimObject*)l2,"mem_side",-1,(SimObject*)membus,"slave",membus_slv_idx++);
	connectPorts((SimObject*)iocache,"mem_side",-1,(SimObject*)membus,"slave",membus_slv_idx++);
	
	connectPorts((SimObject*)bridge,"master",-1,(SimObject*)iobus,"slave",iobus_slv_idx++);
	connectPorts((SimObject*)iobus,"master",iobus_mst_idx++,(SimObject*)realview_io,"pio",-1);
	connectPorts((SimObject*)iobus,"master",iobus_mst_idx++,(SimObject*)timer0,"pio",-1);
	connectPorts((SimObject*)iobus,"master",iobus_mst_idx++,(SimObject*)timer1,"pio",-1);
	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
	{
		connectPorts((SimObject*)iobus,"master",iobus_mst_idx++,(SimObject*)clcd,"pio",-1);
		connectPorts((SimObject*)clcd,"dma",-1,(SimObject*)iobus,"slave",iobus_slv_idx++);
		connectPorts((SimObject*)iobus,"master",iobus_mst_idx++,(SimObject*)kmi0,"pio",-1);
		connectPorts((SimObject*)iobus,"master",iobus_mst_idx++,(SimObject*)kmi1,"pio",-1);
	}
	connectPorts((SimObject*)iobus,"master",iobus_mst_idx++,(SimObject*)cf_ctrl,"pio",-1);
	connectPorts((SimObject*)iobus,"master",iobus_mst_idx++,(SimObject*)dmac_fake,"pio",-1);
	for (uint32_t n=0;n<_num_cores;n++)
	{
		connectPorts((SimObject*)iobus,"master",iobus_mst_idx++,(SimObject*)uart[n],"pio",-1);
	}
	for (uint32_t n=_num_cores;n<MAX_CORES;n++)
	{
		connectPorts((SimObject*)iobus,"master",iobus_mst_idx++,(SimObject*)uart_fake[n],"pio",-1);
	}
	connectPorts((SimObject*)iobus,"master",iobus_mst_idx++,(SimObject*)smc_fake,"pio",-1);
	connectPorts((SimObject*)iobus,"master",iobus_mst_idx++,(SimObject*)sp810_fake,"pio",-1);
	connectPorts((SimObject*)iobus,"master",iobus_mst_idx++,(SimObject*)watchdog_fake,"pio",-1);
	connectPorts((SimObject*)iobus,"master",iobus_mst_idx++,(SimObject*)gpio0_fake,"pio",-1);
	connectPorts((SimObject*)iobus,"master",iobus_mst_idx++,(SimObject*)gpio1_fake,"pio",-1);
	connectPorts((SimObject*)iobus,"master",iobus_mst_idx++,(SimObject*)gpio2_fake,"pio",-1);
	connectPorts((SimObject*)iobus,"master",iobus_mst_idx++,(SimObject*)ssp_fake,"pio",-1);
	connectPorts((SimObject*)iobus,"master",iobus_mst_idx++,(SimObject*)sci_fake,"pio",-1);
	connectPorts((SimObject*)iobus,"master",iobus_mst_idx++,(SimObject*)aaci_fake,"pio",-1);
	connectPorts((SimObject*)iobus,"master",iobus_mst_idx++,(SimObject*)mmc_fake,"pio",-1);
	connectPorts((SimObject*)iobus,"master",iobus_mst_idx++,(SimObject*)rtc_fake,"pio",-1);
//	connectPorts((SimObject*)flash_fake,"pio",-1,(SimObject*)iobus,"port",24);
	connectPorts((SimObject*)iobus,"master",iobus_mst_idx++,(SimObject*)cf_ctrl,"config",-1);
	connectPorts((SimObject*)cf_ctrl,"dma",-1,(SimObject*)iobus,"slave",iobus_slv_idx++);
	if (_system_cfg != "linux")
		connectPorts((SimObject*)tol2bus,"master",tol2bus_mst_idx++,(SimObject*)l2,"cpu_side",-1);
	
	for (uint32_t n=0;n<_num_cores;n++)
	{
		if (_system_cfg != "linux")
		{
			connectPorts((SimObject*)icache[n],"mem_side",-1,(SimObject*)tol2bus,"slave",tol2bus_slv_idx++);
			connectPorts((SimObject*)dcache[n],"mem_side",-1,(SimObject*)tol2bus,"slave",tol2bus_slv_idx++);

			if (_system_cfg == "linuxRestore")
			{
				connectPorts((SimObject*)linux_itb_walker[n],"port",-1,(SimObject*)xtb_walker_cache_bus[n],"slave",xtb_walker_cache_bus_slv_idx[n]++);
				connectPorts((SimObject*)linux_dtb_walker[n],"port",-1,(SimObject*)xtb_walker_cache_bus[n],"slave",xtb_walker_cache_bus_slv_idx[n]++);
				connectPorts((SimObject*)dummy_cpu[n],"icache_port",-1,(SimObject*)icache[n],"cpu_side",-1);
				connectPorts((SimObject*)dummy_cpu[n],"dcache_port",-1,(SimObject*)dcache[n],"cpu_side",-1);
			}
			else
			{
				connectPorts((SimObject*)itb_walker[n],"port",-1,(SimObject*)xtb_walker_cache_bus[n],"slave",xtb_walker_cache_bus_slv_idx[n]++);
				connectPorts((SimObject*)dtb_walker[n],"port",-1,(SimObject*)xtb_walker_cache_bus[n],"slave",xtb_walker_cache_bus_slv_idx[n]++);
				connectPorts((SimObject*)cpu[n],"icache_port",-1,(SimObject*)icache[n],"cpu_side",-1);
				connectPorts((SimObject*)cpu[n],"dcache_port",-1,(SimObject*)dcache[n],"cpu_side",-1);
			}
			connectPorts((SimObject*)xtb_walker_cache_bus[n],"master",xtb_walker_cache_bus_mst_idx[n]++,(SimObject*)xtb_walker_cache[n],"cpu_side",-1);
			connectPorts((SimObject*)xtb_walker_cache[n],"mem_side",-1,(SimObject*)tol2bus,"slave",tol2bus_slv_idx++);
		}
		else
		{
			connectPorts((SimObject*)linux_itb_walker[n],"port",-1,(SimObject*)membus,"slave",membus_slv_idx++);
			connectPorts((SimObject*)linux_dtb_walker[n],"port",-1,(SimObject*)membus,"slave",membus_slv_idx++);
			connectPorts((SimObject*)dummy_cpu[n],"icache_port",-1,(SimObject*)membus,"slave",membus_slv_idx++);
			connectPorts((SimObject*)dummy_cpu[n],"dcache_port",-1,(SimObject*)membus,"slave",membus_slv_idx++);
		}

	}	
	
	connectPorts((SimObject*)iobus,"master",iobus_mst_idx++,(SimObject*)iocache,"cpu_side",-1);
//	connectPorts((SimObject*)iobus,"default",-1,(SimObject*)badaddr_responder2,"pio",-1);


	/*SC_METHOD(service_config_bus);
	dont_initialize();
	for (int i = 0; i < nerios_cfg_port.size(); i++)
	{
		sensitive << nerios_cfg_port[i]->value_changed_event();
		nerios_cfg_port[i]->set_slave_name(string(basename()));
	};*/

   SC_THREAD(running_the_kernel);


	enable_debug(_debug_flags_cfg);
	
	// Initialize the global statistics
	Stats::initSimStats();

	if (_system_cfg == "linuxRestore")
	{
		SC_METHOD(switch_cpus);
		sensitive << switch_cpus_evt;
		dont_initialize();
	}
	else if (_system_cfg == "linux")
	{
		linuxCortexA15->init();
		for (uint32_t i=0;i<objects.size();i++)
			objects[i]->init();
		for (uint32_t i=0;i<_num_cores;i++)
			dummy_cpu[i]->init();
	}
	else
	{
		cortexA15->init();
		for (uint32_t i=0;i<objects.size();i++)
			objects[i]->init();
		for (uint32_t i=0;i<_num_cores;i++)
			cpu[i]->init();
	}
	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
	{
		linuxCortexA15->regStats();
		for (uint32_t i=0;i<_num_cores;i++)
			dummy_cpu[i]->regStats();
	}
	else
		cortexA15->regStats();
	for (uint32_t i=0;i<objects.size();i++)
		objects[i]->regStats();
	for (uint32_t i=0;i<_num_cores;i++)
		cpu[i]->regStats();
		
	if ((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))
	{
		linuxCortexA15->regFormulas();
		for (uint32_t i=0;i<_num_cores;i++)
			dummy_cpu[i]->regFormulas();
	}
	else
		cortexA15->regFormulas();
	for (uint32_t i=0;i<objects.size();i++)
		objects[i]->regFormulas();
	for (uint32_t i=0;i<_num_cores;i++)
		cpu[i]->regFormulas();

	// We're done registering statistics.  Enable the stats package now.
	std::list<Stats::Info *> listinfo = Stats::statsList();
	for (std::list<Stats::Info *>::iterator iter2=listinfo.begin();iter2!=listinfo.end();++iter2)
	{
		(*iter2)->enable();
	}
};

void CortexA15::end_of_elaboration()
{
	
};

void CortexA15::switch_cpus()
{
	curTick(sc_core::sc_time_stamp().value());
	if (_system_cfg == "linux")
	{
		for (uint32_t i=0;i<_num_cores;i++)
			cpu[i]->init();
	}
	CountedDrainEvent* countedDrainEvent = new CountedDrainEvent();
	linuxCortexA15->drain(countedDrainEvent);
	for (uint32_t i=0;i<objects.size();i++)
		objects[i]->drain(countedDrainEvent);
	for (uint32_t i=0;i<_num_cores;i++)
		dummy_cpu[i]->drain(countedDrainEvent);

    linuxCortexA15->setMemoryMode(Enums::timing);

    for (uint32_t i=0;i<_num_cores;i++)
    {
    	(reinterpret_cast<BaseCPU*>(dummy_cpu[i]))->switchOut();
		(reinterpret_cast<SimObject*>(cpu[i]))->takeOverFrom(reinterpret_cast<BaseCPU*>(dummy_cpu[i]));
    }

	linuxCortexA15->resume();
	for (uint32_t i=0;i<objects.size();i++)
		objects[i]->resume();
	for (uint32_t i=0;i<_num_cores;i++)
		cpu[i]->resume();
};

void CortexA15::checkpoint(const std::string &cpt_dir)
{
	curTick(sc_core::sc_time_stamp().value());

	CountedDrainEvent* countedDrainEvent = new CountedDrainEvent();
	linuxCortexA15->drain(countedDrainEvent);
	for (uint32_t i=0;i<objects.size();i++)
		objects[i]->drain(countedDrainEvent);
	for (uint32_t i=0;i<_num_cores;i++)
		dummy_cpu[i]->drain(countedDrainEvent);

	_mem_is_serialized=false;
	Serializable::serializeAll(cpt_dir);

	linuxCortexA15->resume();
	for (uint32_t i=0;i<objects.size();i++)
		objects[i]->resume();
	for (uint32_t i=0;i<_num_cores;i++)
		cpu[i]->resume();
};

void CortexA15::start_of_simulation()
{
	// Task initialization
	if (_system_cfg == "linuxRestore")
	{
	    uint32_t frq_mhz = uint32_t(ceil(1000000/clk1.value()));
	    std::cout << name() << " running at " << frq_mhz << " MHz" << std::endl;
		// Restoring Checkpoint
		Checkpoint* cp = new Checkpoint(_cpt_dir_cfg);
		Serializable::unserializeGlobals(cp); // get the curTick value, will be reset after events unserialization
		gem5_event_queue.cp_timestamp=curTick();
		curTick(0);
		linuxCortexA15->loadState(cp);
		for (uint32_t i=0;i<objects.size();i++)
			objects[i]->loadState(cp);
		//loading memory
		std::cout << "Loading linux DRAM... " << std::endl;
		toACEbridge->unserialize(cp,"system.realview.nvmem",0x80000000);
		std::cout << "Done loading linux DRAM " << std::endl;
		std::cout << "Loading linux Kernel... " << std::endl;
		toACEbridge->unserialize(cp,"system.physmem",0x0);
		std::cout << "Done loading linux Kernel " << std::endl;

		linuxCortexA15->init();
		for (uint32_t i=0;i<objects.size();i++)
			objects[i]->init();
		for (uint32_t i=0;i<_num_cores;i++)
		{
			cpu[i]->init();
			dummy_cpu[i]->init();
		}

		for (uint32_t i=0;i<_num_cores;i++)
			dummy_cpu[i]->loadState(cp);

		linuxCortexA15->resetStats();
		for (uint32_t i=0;i<objects.size();i++)
			objects[i]->resetStats();
		for (uint32_t i=0;i<_num_cores;i++)
			dummy_cpu[i]->resetStats();
		for (uint32_t i=0;i<_num_cores;i++)
			cpu[i]->resetStats();

		linuxCortexA15->startup();
		for (uint32_t i=0;i<objects.size();i++)
			objects[i]->startup();
		for (uint32_t i=0;i<_num_cores;i++)
			dummy_cpu[i]->startup();
		for (uint32_t i=0;i<_num_cores;i++)
			cpu[i]->startup();

		linuxCortexA15->resume();
		for (uint32_t i=0;i<objects.size();i++)
			objects[i]->resume();
		for (uint32_t i=0;i<_num_cores;i++)
			dummy_cpu[i]->resume();
		gem5_event_queue.cp_timestamp=0;
		switch_cpus_evt.notify(11*clk1);
	}
	else if (_system_cfg == "linux")
	{
	    uint32_t frq_mhz = uint32_t(ceil(1000000/clk1.value()));
	    std::cout << name() << " running at " << frq_mhz << " MHz" << std::endl;

	    // launching the engine

		linuxCortexA15->initState();
		for (uint32_t i=0;i<objects.size();i++)
			objects[i]->initState();
		for (uint32_t i=0;i<_num_cores;i++)
			dummy_cpu[i]->initState();
//		for (uint32_t i=0;i<_num_cores;i++)
//			cpu[i]->initState();

		linuxCortexA15->resetStats();
		for (uint32_t i=0;i<objects.size();i++)
			objects[i]->resetStats();
		for (uint32_t i=0;i<_num_cores;i++)
			dummy_cpu[i]->resetStats();
//		for (uint32_t i=0;i<_num_cores;i++)
//			cpu[i]->resetStats();

                linuxCortexA15->startup();
		for (uint32_t i=0;i<objects.size();i++)
			objects[i]->startup();
		for (uint32_t i=0;i<_num_cores;i++)
			dummy_cpu[i]->startup();
//		for (uint32_t i=0;i<_num_cores;i++)
//			cpu[i]->startup();
	}
};
void CortexA15::running_the_kernel()
{
   wait(SC_ZERO_TIME);
	//while (true){
      if (!((_system_cfg == "linux") || (_system_cfg == "linuxRestore"))){
          empty_requests();
      }
   //}
}

