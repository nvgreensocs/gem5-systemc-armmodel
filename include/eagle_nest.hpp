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

File	:	eagle_nest.hpp

Purpose	:	Class definition for the cortex A15 model

Description:

Eagle Nest bridge

******************************************************************CE*/


#ifndef __EAGLE_NEST_HPP__
#define __EAGLE_NEST_HPP__

#include <queue>

#include "greencontrol/config.h"
#include "amba.h"

#include "vbusm_ptl_ports.h"
#include "greencontrol/config.h"
#include "functional_transactor_if.hpp"
#include "fifo_synchronizer.h"
#include "generic_pipeline.hpp"

// FIXME
struct Cmd
{
	vbusm_cmd_pkt _cmd;
	uint64_t      _ts;
	bool          _return_valid;
	std::queue<vbusm_rd_pkt>  _rdata;
};

struct WrData
{
	vbusm_wr_pkt _pkt;
	uint32_t walign_2x;
};

template<unsigned int BUSWIDTH>
class EagleNest:public sc_core::sc_module
{
public:
	amba::amba_slave_socket<> slave_sock;
    vbusm_ptl_master_port       bus_if;
	
	SC_HAS_PROCESS(EagleNest);
	EagleNest(sc_core::sc_module_name nm, std::string slave_port_name,std::string master_port_name);
	~EagleNest();
private:
	
	synchronizer_fifo<Cmd>*                rd_cmd_buf;      // read command buffer
	synchronizer_fifo<Cmd>*                wr_cmd_buf;      // write command buffer
	synchronizer_fifo<vbusm_rd_pkt>*       rdata_fifo;      // read data fifo
	synchronizer_fifo<vbusm_wrstatus_pkt>* wrstat_fifo;     // wrstat pkt fifo
	synchronizer_fifo<WrData>*       wdata_buf;       // write data buffer
	
	std::deque<Cmd>                        rdata_reordering_queue;
	std::deque<Cmd>                        wstatus_reordering_queue;
	std::deque<Cmd>                        rdcmd_in_flight;
	std::deque<Cmd>                        wrcmd_in_flight;

	sc_core::sc_fifo<Cmd>                                rd_elastic_buffer;
	sc_core::sc_fifo<Cmd>                                wr_elastic_buffer;
	tlm::tlm_generic_payload*                            rd_transactions_pool[8][CBA::MAX_COMMAND_IDS];
	tlm::tlm_generic_payload*                            wr_transactions_pool[8][CBA::MAX_COMMAND_IDS];
	sc_core::sc_fifo<uint32_t>                           wr_data_size;

	sc_time               eagle_clk_period;
	sc_time               vbusm_clk_period;
	CBA::clock_edge_calc  _vbusm_clk;
	CBA::clock_edge_calc  _eagle_clk;
	
	gs::cnf::cnf_api *m_Api;
	
	gs::gs_param<uint32_t>               _cluster_id_cfg;
	uint32_t                          _cluster_id;
	uint32_t                          _wdata_counter;
	uint32_t                          _wdata_available;

	gs::gs_param<uint32_t>            _vbusm_priority_cfg;
	gs::gs_param<bool>                _linux_remap_cfg;
	bool                              _linux_remap;

	uint64_t                          _rd_return_address[CBA::MAX_COMMAND_IDS];
	
	sc_core::sc_event                 _wstatus_if_ready;
	sc_core::sc_event                 _read_if_ready;

	bool                 _wstatus_if_blocked;
	bool                 _read_if_blocked;
	uint64_t             _timestamp;

	tlm::tlm_generic_payload * pending_rd_request;
	tlm::tlm_generic_payload * pending_wr_request;
	tlm::tlm_generic_payload * pending_wr_data_request;
	amba::amba_burst_size *burstsize, default_burstsize;
	unsigned int counter;
	tlm::tlm_phase current_phase;
	void readResponseThread();
	void writeResponseThread();
	void masterTimingListener(gs::socket::timing_info);
	tlm::tlm_sync_enum nb_fw_transport(tlm::tlm_generic_payload & trans, tlm::tlm_phase & ph, sc_core::sc_time &delay);
	
	vbusm_cmd_pkt compute_vbusm_read(tlm::tlm_generic_payload * trans);
	vbusm_cmd_pkt compute_vbusm_write(tlm::tlm_generic_payload * trans);
	void before_end_of_elaboration();
	void master_ws_thread();
	void master_wr_thread();
	void master_cmd_thread();
	void master_rd_thread();
	void read_command_splitting_thread();
	void write_command_splitting_thread();
	bool checkNoOlderRead(Cmd cmd_pkt);
	bool checkNoOlderWrite(Cmd cmd_pkt);
	void Empty_stage_func(std::string _name, uint32_t id,bus_channel_in<boost::shared_ptr<boost::any> >* in,bus_channel_out<boost::shared_ptr<boost::any> >* out, sc_core::sc_event* evt);
	void rdcmd_push_sync(std::string _name, uint32_t id,bus_channel_in<boost::shared_ptr<boost::any> >* in,bus_channel_out<boost::shared_ptr<boost::any> >* out, sc_core::sc_event* evt);
	void wrcmd_push_sync(std::string _name, uint32_t id,bus_channel_in<boost::shared_ptr<boost::any> >* in,bus_channel_out<boost::shared_ptr<boost::any> >* out, sc_core::sc_event* evt);
	void wdata_push_sync(std::string _name, uint32_t id,bus_channel_in<boost::shared_ptr<boost::any> >* in,bus_channel_out<boost::shared_ptr<boost::any> >* out, sc_core::sc_event* evt);
	void rdata_push_sync(std::string _name, uint32_t id,bus_channel_in<boost::shared_ptr<boost::any> >* in,bus_channel_out<boost::shared_ptr<boost::any> >* out, sc_core::sc_event* evt);
	void ws_push_sync(std::string _name, uint32_t id,bus_channel_in<boost::shared_ptr<boost::any> >* in,bus_channel_out<boost::shared_ptr<boost::any> >* out, sc_core::sc_event* evt);
	void rdcmd_pop_sync(std::string _name, uint32_t id,bus_channel_in<boost::shared_ptr<boost::any> >* in,bus_channel_out<boost::shared_ptr<boost::any> >* out, sc_core::sc_event* evt);
	void wrcmd_pop_sync(std::string _name, uint32_t id,bus_channel_in<boost::shared_ptr<boost::any> >* in,bus_channel_out<boost::shared_ptr<boost::any> >* out, sc_core::sc_event* evt);
	void wdata_pop_sync(std::string _name, uint32_t id,bus_channel_in<boost::shared_ptr<boost::any> >* in,bus_channel_out<boost::shared_ptr<boost::any> >* out, sc_core::sc_event* evt);
	void rdata_pop_sync(std::string _name, uint32_t id,bus_channel_in<boost::shared_ptr<boost::any> >* in,bus_channel_out<boost::shared_ptr<boost::any> >* out, sc_core::sc_event* evt);
	void ws_pop_sync(std::string _name, uint32_t id,bus_channel_in<boost::shared_ptr<boost::any> >* in,bus_channel_out<boost::shared_ptr<boost::any> >* out, sc_core::sc_event* evt);
	void wdata_e3_stage(std::string _name, uint32_t id,bus_channel_in<boost::shared_ptr<boost::any> >* in,bus_channel_out<boost::shared_ptr<boost::any> >* out, sc_core::sc_event* evt);
	
	// pipeline

	bus_channel<boost::shared_ptr<boost::any> >  rdcmd_pipe_in;
	bus_channel<boost::shared_ptr<boost::any> >  wrcmd_pipe_in;
	bus_channel<boost::shared_ptr<boost::any> >  wdata_pipe_in;
	bus_channel<boost::shared_ptr<boost::any> >  rdata_pipe_in;
	bus_channel<boost::shared_ptr<boost::any> >  ws_pipe_in;
	bus_channel<boost::shared_ptr<boost::any> >  rdcmd_pipe_out;
	bus_channel<boost::shared_ptr<boost::any> >  wrcmd_pipe_out;
	bus_channel<boost::shared_ptr<boost::any> >  wdata_pipe_out;
	bus_channel<boost::shared_ptr<boost::any> >  rdata_pipe_out;
	bus_channel<boost::shared_ptr<boost::any> >  ws_pipe_out;

    std::vector<sc_core::sc_event*> main_arm_clk_pipeline;
    std::vector<sc_core::sc_event*> main_msmc_clk_pipeline;

    void main_arm_clk_pipeline_drive()
    {
    	while(true)
    	{
    		wait(eagle_clk_period);
			for ( std::vector<sc_core::sc_event*>::reverse_iterator rit=main_arm_clk_pipeline.rbegin() ; rit < main_arm_clk_pipeline.rend(); ++rit )
			{
				(*rit)->notify();
				wait(SC_ZERO_TIME);
			}
    	}
    }
    void main_msmc_clk_pipeline_drive()
    {
    	while(true)
    	{
    		wait(vbusm_clk_period);
			for ( std::vector<sc_core::sc_event*>::reverse_iterator rit=main_msmc_clk_pipeline.rbegin() ; rit < main_msmc_clk_pipeline.rend(); ++rit )
			{
				(*rit)->notify();
				wait(SC_ZERO_TIME);
			}
    	}
    }

};

ostream & operator<<(ostream &, const Cmd &);


#endif //__EAGLE_NEST_HPP__
