/*********************************************************************
 *  Copyright 2012 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 ********************************************************************/

/*CS******************************************************************

Project :	CIV TLM

Author	:   Alexandre Romaña

Date	:	Q4 2011

File	:	bridge_classic_to_ambatlm2.cpp

Purpose	:	Implementation unit 

Description:

Bridge that converts the classic sendTimingResp/recvTimingReq handshake to AMBA TLM2 CT

******************************************************************CE*/



/*************** standard files inclusion ***************/

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/user.h>
#include <fcntl.h>
#include <unistd.h>
#include <zlib.h>

#include <cerrno>
#include <cstdio>
#include <iostream>
#include <string>

/*************** pre-processor options definition ***********/

/*************** application files inclusion ***************/

#include "base/trace.hh"
#include "bridge_classic_to_ambatlm2.hpp"
#include "base/stats/info.hh"
#include "base/stats/output.hh"
#include "base/stats/text.hh"
#include "base/statistics.hh"
#include "base/callback.hh"

/*************** macros definition ***************/

/*************** types definition ***************/

/*************** constants definition ***************/

/*************** pre-processor autocheck ***************/

/*************** global variables and arrays definition ***************/

namespace Debug {

extern SimpleFlag BusBridge;

} // namespace Debug

//#define DEBUG

/*************** local variables and arrays definition ***************/

/*************** local routines definition ***************/

/*************** funtions definition  ***************/

//UGLY FIX ME ASAP, SimpleMemory uses mmap on the range size, 0 triggers an error
SimpleMemoryParams* BridgeClassicToAMBATLM2_init(SimpleMemoryParams* pm)
{
	pm->in_addr_map=true;
	pm->zero=true;
	pm->conf_table_reported=true;
    pm->latency=0;
    pm->latency_var=0;
	pm->null=true;
	pm->port_port_connection_count=1;
	pm->range = Range<long long unsigned int>("0x0:+0x1000000000000000");
	return pm;
}

template<unsigned int BUSWIDTH>
BridgeClassicToAMBATLM2<BUSWIDTH>::BridgeClassicToAMBATLM2(sc_core::sc_module_name nm, std::string ace_master_name, std::string debug_master_name)
    : sc_core::sc_module(nm),
      SimpleMemory(BridgeClassicToAMBATLM2_init(new SimpleMemoryParams())),
      writeDataQueue(1),
      slavePort(NULL),funcPort(NULL),
      master_sock(ace_master_name.c_str(),amba::amba_AXI, amba::amba_CT,false, gs::socket::GS_TXN_WITH_DATA),
      debug_port(debug_master_name.c_str())
{
	m_Api = gs::cnf::GCnf_Api::getApiInstance(this);
	
	master_sock.template register_nb_transport_bw< BridgeClassicToAMBATLM2<BUSWIDTH> >(this, & BridgeClassicToAMBATLM2<BUSWIDTH>::nb_bw_transport);
	debug_port.bind( *this ); // Initiator socket bound to the initiator itself
    counter=0;
    needRetry=false;
    needWrRetry=false;
    needWrIdRetry=false;
    needRdIdRetry=false;
    bus_stalling=false;
    for (uint32_t i=0;i<128;i++)
    {
		rd_packets[i]=NULL;
		wr_packets[i]=NULL;
	}
	clk_period=m_Api->getValue<sc_core::sc_time>("clk1");
	need_wenable_event=false;
	
	SC_THREAD(sendWriteData);
	SC_METHOD(send_dummy_response);
	sensitive << send_dummy_response_event;
	dont_initialize();

	SimObjectParams* tmp_params= const_cast<SimObjectParams*>(_params);
	tmp_params->name=std::string(sc_core::sc_object::name());

}

template<unsigned int BUSWIDTH>
void BridgeClassicToAMBATLM2<BUSWIDTH>::send_dummy_response()
{
	curTick(sc_core::sc_time_stamp().value());
	dummy_pkt.front()->makeTimingResponse();
	if (bus_stalling)
	{
		retryQueue.push(dummy_pkt.front());
	}
	else if (!slavePort->sendTimingResp(dummy_pkt.front()))
	{
		bus_stalling=true;
		retryQueue.push(dummy_pkt.front());
	}		
	dummy_pkt.pop();
}

template<unsigned int BUSWIDTH>
void BridgeClassicToAMBATLM2<BUSWIDTH>::init()
{
    // Make sure that both sides are connected to.
    if (!slavePort->isConnected())// || !master_sock->get_interface())
        fatal("Both ports of GEM5toAMBA bridge must be connected.\n");
	slavePort->sendRangeChange();
}


/** Function called by the port when the bus is receiving a Functional transaction.*/
template<unsigned int BUSWIDTH>
void BridgeClassicToAMBATLM2<BUSWIDTH>::recvFunctional(PacketPtr pkt)
{
//	std::cout << "Called recvFunctional" << std::endl;
    tlm::tlm_generic_payload trans;
	if (pkt->isRead())
		trans.set_read();
	else if (pkt->isWrite())
		trans.set_write();
	trans.set_address(pkt->getAddr());
	trans.set_data_length(pkt->getSize());
//	trans.set_streaming_width(TBD);
	trans.set_data_ptr(pkt->getPtr<unsigned char>());
	debug_port->transport_dbg(static_cast<tlm::tlm_generic_payload &>(trans));

}

/** Timing function called by port when it is once again able to process * requests. */
template<unsigned int BUSWIDTH>
void BridgeClassicToAMBATLM2<BUSWIDTH>::recvRetry()
{
#ifdef DEBUG
	assert(!retryQueue.empty());
	if (retryQueue.front()->isRead())
	{
		std::cout << "In " << sc_core::sc_object::name() << " at time " << sc_time_stamp() << " sending a READ response to GEM5 with address=0x" << hex << retryQueue.front()->getAddr() << dec << " size=" << retryQueue.front()->getSize();
		std::cout << hex << " and data= [";
		for(unsigned j=0;j<retryQueue.front()->getSize(); j++)
			std::cout << "0x" << uint32_t(retryQueue.front()->getPtr<uint8_t>()[j]) << ",";
		std::cout << "]" << dec << std::endl;
	}
	else
		std::cout << "In " << sc_core::sc_object::name() << " at time " << sc_time_stamp() << " sending a WS response to GEM5 with address=0x" << hex << retryQueue.front()->getAddr() << dec << " size=" << retryQueue.front()->getSize() << std::endl;
#endif
	curTick(sc_core::sc_time_stamp().value());
	bool check = slavePort->sendTimingResp(retryQueue.front());
	assert(check);
	retryQueue.pop();
	if (retryQueue.empty())
	{
		bus_stalling=false;
	}
	else
	{
		check = !slavePort->sendTimingResp(retryQueue.front());
		assert(check);
	}
}

template<unsigned int BUSWIDTH>
void BridgeClassicToAMBATLM2<BUSWIDTH>::end_of_elaboration()
{
	
	master_sock.set_timing_listener_callback(this, & BridgeClassicToAMBATLM2::slaveTimingListener);
	
	gs::socket::timing_info info;
	info.set_start_time(tlm::BEGIN_REQ,sc_core::sc_time(1,sc_core::SC_PS));
	info.set_start_time(amba::BEGIN_DATA,sc_core::sc_time(2,sc_core::SC_PS));
	info.set_start_time(tlm::END_RESP,sc_core::sc_time(3,sc_core::SC_PS));
	master_sock.set_initiator_timing(info);	
//	master_sock.activate_synchronization_protection();
}

template<unsigned int BUSWIDTH>
void BridgeClassicToAMBATLM2<BUSWIDTH>::sendWriteData()
{
	PacketPtr pkt;
	sc_core::sc_time delay= sc_core::SC_ZERO_TIME;
    uint32_t width=(BUSWIDTH/8);
    uint32_t burstLen=0;
    uint32_t count=0;    
    uint32_t id = 0;
    uint32_t dp_size = 0;
    uint64_t address;
	tlm::tlm_generic_payload* _trans;
	tlm::tlm_phase ph;
	amba::amba_id * m_id;
	bool last=false;
	while(true)
	{
		id = writeDataQueue.read();
		pkt=wr_packets[id];
		_trans=master_sock.get_transaction();
		master_sock.reserve_data_size(*_trans, pkt->getSize()); //reserve the requested amount of data array bytes
		_trans->set_command(tlm::TLM_WRITE_COMMAND);
		address=pkt->getAddr();
		burstLen = uint32_t(ceil(double_t(pkt->getSize()+address%width)/double_t(width)));
		master_sock.template get_extension<amba::amba_id>(m_id,*_trans);
		m_id->value=id;
		master_sock.template validate_extension<amba::amba_id>(*_trans);
		count=0;	
		memcpy(_trans->get_data_ptr(),pkt->getPtr<uint8_t>(),pkt->getSize());
		dp_size=std::min(width-uint32_t(address%width),pkt->getSize());
		uint32_t bytes_left=pkt->getSize();
		// hanle the case where first dataphase is not aligned
		if((address%width) != 0)
		{
			_trans->set_address(address);
			_trans->set_data_length(dp_size);
			if (burstLen == 1)
			{
				ph= amba::BEGIN_LAST_DATA;
				last=true;
			}
			else
			{
				ph = amba::BEGIN_DATA;
				last=false;
			}
			switch(master_sock->nb_transport_fw(*_trans,ph,delay))
			{
				case tlm::TLM_ACCEPTED:
					need_wenable_event=true;
					wait(wenable_event);
					count++;
					address += width;
					if (!last)
						bytes_left -= width;
					//as per packet.cc:74 writeback never needs a response
					if (last && (pkt->cmd != MemCmd::Writeback))
						pkt->makeTimingResponse();
                                        break;
				case tlm::TLM_UPDATED:
					assert(ph == amba::END_DATA);
					bytes_left -= dp_size;
					count++;
					address += dp_size;
					//as per packet.cc:74 writeback never needs a response
					if (last && (pkt->cmd != MemCmd::Writeback))
						pkt->makeTimingResponse();
					break;
				case tlm::TLM_COMPLETED:
					abort();
			}
		}
		while(count<burstLen)
		{
			_trans->set_address(address);
			if(count+1 >= burstLen)
			{
				_trans->set_data_length(bytes_left);
			   ph= amba::BEGIN_LAST_DATA;
			   last=true;
		    }
			else
			{
				_trans->set_data_length(width);
				last=false;
			   ph =amba::BEGIN_DATA;
		    }
//			std::cout << sc_core::sc_object::name() <<" Sending the write transaction data, at time " << sc_core::sc_time_stamp() <<" BURST-COUNT="<< (count+1)<<std::endl;
			switch(master_sock->nb_transport_fw(*_trans,ph,delay))
			{
				case tlm::TLM_ACCEPTED: 
					need_wenable_event=true;
					wait(wenable_event);
					count++;
					address += width;
					if (!last)
						bytes_left -= width;
					//as per packet.cc:74 writeback never needs a response
					if (last && (pkt->cmd != MemCmd::Writeback))
						pkt->makeTimingResponse();
					break;
				case tlm::TLM_UPDATED:
					assert(ph == amba::END_DATA);
					count++;
					address += width;
					if (!last)
						bytes_left -= width;
					//as per packet.cc:74 writeback never needs a response
					if (last && (pkt->cmd != MemCmd::Writeback))
						pkt->makeTimingResponse();
					break;
				case tlm::TLM_COMPLETED: 
					abort();
			}
			wait(clk_period);
		}
		if (needWrRetry)
		{
			curTick(sc_core::sc_time_stamp().value());
			needWrRetry=false;
			slavePort->sendRetry();
		}
	}
}


template<unsigned int BUSWIDTH>
void BridgeClassicToAMBATLM2<BUSWIDTH>::slaveTimingListener(gs::socket::timing_info info)
{
//	std::cout << sc_core::sc_object::name() << " Reached the slave timing listener" << std::endl;
}
	
template<unsigned int BUSWIDTH>
tlm::tlm_sync_enum BridgeClassicToAMBATLM2<BUSWIDTH>::nb_bw_transport(tlm::tlm_generic_payload & trans, tlm::tlm_phase & ph, sc_core::sc_time &delay)
{
//  		std::cout << sc_core::sc_object::name()<< ph << std::endl;
	  	amba::amba_id * m_id;
	  	amba::amba_exclusive * m_exclusive;
	    tlm::tlm_sync_enum returnVal=tlm::TLM_ACCEPTED;
	    std::ostringstream msg;
	    msg.str("");
	    if(ph == amba::BEGIN_LAST_RESP || ph == tlm::BEGIN_RESP)
		{
			//assert(trans.get_command()==tlm::TLM_READ_COMMAND && "Write Command doesn't support the response.");
			master_sock.template get_extension<amba::amba_id>(m_id,trans);
			if (trans.is_write())
			{
			  assert(ph == tlm::BEGIN_RESP);
			  assert(wr_packets[m_id->value] != NULL);
			  //as per packet.cc:74 writeback never needs a response
			  if (wr_packets[m_id->value]->cmd != MemCmd::Writeback)
			  {
					if (master_sock.template get_extension<amba::amba_exclusive>(m_exclusive,trans))
					{
						if (m_exclusive->value)
						{
							wr_packets[m_id->value]->req->setExtraData(0);
						}
						else
						{
							wr_packets[m_id->value]->req->setExtraData(1);
						}
					}
				  curTick(sc_core::sc_time_stamp().value());
				  if (bus_stalling)
					  retryQueue.push(wr_packets[m_id->value]);
				  else if (!slavePort->sendTimingResp(wr_packets[m_id->value]))
				  {
						bus_stalling=true;
						retryQueue.push(wr_packets[m_id->value]);
				  }	
#ifdef DEBUG
				  else
					std::cout << "In " << sc_core::sc_object::name() << " at time " << sc_time_stamp() << " sending a WS response to GEM5 with address=0x" << hex << trans.get_address() << dec << " size=" << trans.get_data_length() << std::endl;
#endif
			  }
			  master_sock.release_transaction(&trans); //now we release
			  wr_packets[m_id->value] = NULL;
				if (needWrIdRetry)
				{
					curTick(sc_core::sc_time_stamp().value());
					needWrIdRetry=false;
					slavePort->sendRetry();
				}
			}
           else if (ph == amba::BEGIN_LAST_RESP)
           {
				assert(rd_packets[m_id->value] != NULL);
				uint8_t * data=trans.get_data_ptr();
				PacketPtr pkt = rd_packets[m_id->value];
				pkt->makeTimingResponse();
				uint32_t shadow_size=pkt->getSize();
				uint64_t shadow_addr=pkt->getAddr();
//				std::cout << sc_core::sc_object::name() << " bridge received read data for address = "  << trans.get_address()-pkt->getAddr() << " at time " << sc_time_stamp() << std::endl;
				memcpy(pkt->getPtr<uint8_t>(),data,pkt->getSize());
//				if (master_sock.template get_extension<amba::amba_exclusive>(m_exclusive,trans))
//				{
//					if (m_exclusive->value)
//						pkt->req->setExtraData(0);
//					else
//						pkt->req->setExtraData(1);
//				}
				curTick(sc_core::sc_time_stamp().value());
				if (bus_stalling)
					retryQueue.push(pkt);
				else if (!slavePort->sendTimingResp(pkt))
				{
					bus_stalling=true;
					retryQueue.push(pkt);
				}	
#ifdef DEBUG				  						  
				else
				{
					std::cout << "In " << sc_core::sc_object::name() << " at time " << sc_time_stamp() << " sending a READ response to GEM5 with address=0x" << hex << shadow_addr << dec << " size=" << shadow_size;
					std::cout << hex << " and data= [";
					for(unsigned j=0;j<shadow_size; j++)
						std::cout << "0x" << uint32_t(data[j]) << ",";
					std::cout << "]" << dec << std::endl;
				}
#endif
				rd_packets[m_id->value]=NULL;
				master_sock.release_transaction(&trans);
				if (needRdIdRetry)
				{
					curTick(sc_core::sc_time_stamp().value());
					needRdIdRetry=false;
					slavePort->sendRetry();
				}
			}
			//always accept directly (cf EagleNest spec)
            ph=tlm::END_RESP;
			return tlm::TLM_UPDATED;
		}
	    else if(ph ==tlm::END_REQ)
	    { 
			if (needRetry)
			{
				curTick(sc_core::sc_time_stamp().value());
				trans.release();
				needRetry=false;
				slavePort->sendRetry();
			}
	    }
		else if(ph== amba::END_DATA )
		{
			if (need_wenable_event)
			{
				need_wenable_event=false;
            	wenable_event.notify();
			}
		}
		else
				assert("Unexpected phase returned from AXI slave.");	
		return returnVal;
}


	/** Function called by the port when the bus is receiving a Timing
	 * transaction.*/
	template<unsigned int BUSWIDTH>
	bool BridgeClassicToAMBATLM2<BUSWIDTH>::recvTimingReq(PacketPtr pkt)
	{
		amba::amba_id * m_id;
		amba::amba_burst_size * m_burst_size;
		amba::amba_exclusive * m_exclusive;
		sc_core::sc_time delay= sc_core::SC_ZERO_TIME;
		std::ostringstream msg;
		msg.str("");
		tlm::tlm_generic_payload* current_trans;
		//DPRINTF(BusBridge, "recvTiming: src %d dest %d addr 0x%x\n",pkt->getSrc(), pkt->getDest(), pkt->getAddr());	
#ifdef DEBUG	
		if (pkt->cmd == MemCmd::ReadReq)
			std::cout << "In " << sc_core::sc_object::name() << " at time " << sc_time_stamp() << " received a READ request from GEM5 with address=0x" << hex << pkt->getAddr() << dec << " size=" << pkt->getSize() << std::endl;
		else if (pkt->cmd == MemCmd::ReadExReq) 
			std::cout << "In " << sc_core::sc_object::name() << " at time " << sc_time_stamp() << " received a READ EXCLUSIVE request from GEM5 with address=0x" << hex << pkt->getAddr() << dec << " size=" << pkt->getSize() << std::endl;
		else if (pkt->cmd == MemCmd::WriteReq)
		{
			std::cout << "In " << sc_core::sc_object::name() << " at time " << sc_time_stamp() << " received a WRITE request from GEM5 with address=0x" << hex << pkt->getAddr() << dec << " size=" << pkt->getSize();
			std::cout << hex << " and data= [";
			for(unsigned j=0;j<pkt->getSize(); j++)
				std::cout << "0x" << uint32_t(pkt->getPtr<uint8_t>()[j]) << ",";
			std::cout << "]" << dec << std::endl;
		}
		else if (pkt->cmd == MemCmd::Writeback) 
		{
			std::cout << "In " << sc_core::sc_object::name() << " at time " << sc_time_stamp() << " received a WRITEBACK request from GEM5  with address=0x" << hex << pkt->getAddr() << dec << " size=" << pkt->getSize();			
			std::cout << hex << " and data= [";
			for(unsigned j=0;j<pkt->getSize(); j++)
				std::cout << "0x" << uint32_t(pkt->getPtr<uint8_t>()[j]) << ",";
			std::cout << "]" << dec << std::endl;
		}
		else if ((pkt->cmd == MemCmd::UpgradeReq) || (pkt->cmd == MemCmd::SCUpgradeReq))
		{
			std::cout << "In " << sc_core::sc_object::name() << " at time " << sc_time_stamp() << " received an UPGRADE request from GEM5  with address=0x" << hex << pkt->getAddr() << dec << " size=" << pkt->getSize() << std::endl;			
		}
		else if (pkt->cmd == MemCmd::SwapReq)
		{
			std::cout << "In " << sc_core::sc_object::name() << " at time " << sc_time_stamp() << " received a SWAP request from GEM5  with address=0x" << hex << pkt->getAddr() << dec << " size=" << pkt->getSize() << std::endl;			
		}
		else if (pkt->cmd == MemCmd::LoadLockedReq)
		{
			std::cout << "In " << sc_core::sc_object::name() << " at time " << sc_time_stamp() << " received an exclusive LOAD request from GEM5  with address=0x" << hex << pkt->getAddr() << dec << " size=" << pkt->getSize() << std::endl;
		}
		else if (pkt->cmd == MemCmd::StoreCondReq)
		{
			std::cout << "In " << sc_core::sc_object::name() << " at time " << sc_time_stamp() << " received a STORE CONDITIONAL request from GEM5  with address=0x" << hex << pkt->getAddr() << dec << " size=" << pkt->getSize() << std::endl;
		}
		else if (pkt->cmd == MemCmd::SwapReq)
		{
			std::cout << "In " << sc_core::sc_object::name() << " at time " << sc_time_stamp() << " received a SWAP request from GEM5  with address=0x" << hex << pkt->getAddr() << dec << " size=" << pkt->getSize() << std::endl;
		}
		else
		{
			std::cout << "In " << sc_core::sc_object::name() << " at time " << sc_time_stamp() << " received an UNKNOWN request from GEM5  with address=0x" << hex << pkt->getAddr() << dec << " size=" << pkt->getSize() << std::endl;
		}
	    if (pkt->memInhibitAsserted())
	    {
	    	std::cout << "In " << sc_core::sc_object::name() << " at time " << sc_time_stamp() << " request from GEM5 was inhibited" << std::endl;
	    }
#endif
	    if (pkt->memInhibitAsserted())
	    {
	    	return true;
	    }
		if (pkt->cmd == MemCmd::ReadReq || pkt->cmd == MemCmd::ReadExReq || pkt->cmd == MemCmd::WriteReq || pkt->cmd == MemCmd::Writeback || pkt->cmd == MemCmd::LoadLockedReq || pkt->cmd == MemCmd::StoreCondReq)
		{
			if (pkt->isWrite() && (writeDataQueue.num_free() == 0))
			{
				needWrRetry=true;
				return false;
			}
			// find a free id
			uint32_t id=10000;
			uint32_t mstid;
			if (pkt->req->hasContextId())
			{
				mstid=pkt->req->contextId();
			}
			else
			{
				mstid=0; //  Eagle L2 Buffer and ACP requests alias to CPU0 since there are only 4 mstids allocated to Eagle
			}
			if (pkt->isRead())
			{
				for (uint32_t i=(mstid*16);i<(mstid*16)+16;i++)
				{
					if (rd_packets[i] == NULL)
					{
						id=i;
						break;
					}
				}
				if (id == 10000)
				{
					needRdIdRetry=true;
					return false;
				}
			}
			else
			{
				for (uint32_t i=(mstid*16);i<(mstid*16)+16;i++)
				{
					if (wr_packets[i] == NULL)
					{
						id=i;
						break;
					}
				}
				if (id == 10000)
				{
					needWrIdRetry=true;
					return false;					
				}
			}

			current_trans =master_sock.get_transaction(); //get a memory managed transaction from the pool.
			current_trans->set_address(pkt->getAddr());
			current_trans->set_data_length(pkt->getSize());
			master_sock.reserve_data_size(*current_trans, pkt->getSize()); //reserve the requested amount of data array bytes
			current_trans->set_streaming_width(BUSWIDTH-(pkt->getAddr()%BUSWIDTH));
			current_trans->set_command(pkt->isRead()?tlm::TLM_READ_COMMAND:tlm::TLM_WRITE_COMMAND);
			if (pkt->cmd == MemCmd::LoadLockedReq || pkt->cmd == MemCmd::StoreCondReq)
			{
				master_sock.template get_extension<amba::amba_exclusive>(m_exclusive,*current_trans);
				m_exclusive->value=false;
				master_sock.template validate_extension<amba::amba_exclusive>(*current_trans);
			}
			if (pkt->req->isUncacheable())
				master_sock.template invalidate_extension<amba::amba_cacheable>(*current_trans);
			else
				master_sock.template validate_extension<amba::amba_cacheable>(*current_trans);

			if (pkt->req->isInstFetch()) master_sock.template validate_extension<amba::amba_instruction>(*current_trans);

			master_sock.template get_extension<amba::amba_burst_size>(m_burst_size,*current_trans);
			m_burst_size->value=(BUSWIDTH/8); //buswidth is in bits
			master_sock.template validate_extension<amba::amba_burst_size>(*current_trans);
			master_sock.template get_extension<amba::amba_id>(m_id,*current_trans);

			tlm::tlm_phase ph= tlm::BEGIN_REQ;
			m_id->value=id;
			master_sock.template validate_extension<amba::amba_id>(*current_trans);
			

			tlm::tlm_sync_enum retval= master_sock->nb_transport_fw(*current_trans,ph,delay);
			if((retval==tlm::TLM_UPDATED) && (ph == tlm::END_REQ)) // request accepted by Eagle Nest
			{
				if (pkt->isRead())
				{
					rd_packets[id]=pkt;
				}
				else
				{
					wr_packets[id]=pkt;
					writeDataQueue.write(id);
				}
				return true;
			}
			else if(retval== tlm::TLM_ACCEPTED) // request blocked till END_REQ received
			{
				needRetry=true;
				return false;
			}
		}
		else if ((pkt->cmd == MemCmd::UpgradeReq) || (pkt->cmd == MemCmd::SCUpgradeReq))
		{
			// It's a transition from S->E.
            // The cache has a block in a shared state and requires an exclusive copy so it can write to it.
			// ignore as no L3$ so far
			dummy_pkt.push(pkt);
			send_dummy_response_event.notify(clk_period);
		}
		else if (pkt->cmd == MemCmd::SwapReq)
		{
			std::cout << "In " << sc_core::sc_object::name() << " at time " << sc_time_stamp() << " received a SWAP request from GEM5  with address=0x" << hex << pkt->getAddr() << dec << " size=" << pkt->getSize() << "-----> what should I do boss?" << std::endl;			
		}
		else
		{
			std::cout << "ERROR: In " << sc_core::sc_object::name() << " at time " << sc_time_stamp() << " received an UNKNOWN request from GEM5  with address=0x" << hex << pkt->getAddr() << dec << " size=" << pkt->getSize() << std::endl;
			if (pkt->cmd == MemCmd::InvalidCmd)
			{
				std::cout << "InvalidCmd" << std::endl;
			}
			else if (pkt->cmd == MemCmd::SoftPFReq)
			{
				std::cout << "SoftPFReq" << std::endl;
			}
			else if (pkt->cmd == MemCmd::HardPFReq)
			{
				std::cout << "HardPFReq" << std::endl;
			}
			else if (pkt->cmd == MemCmd::WriteInvalidateReq)
			{
				std::cout << "WriteInvalidateReq" << std::endl;
			}
			else if (pkt->cmd == MemCmd::SCUpgradeFailReq)
			{
				std::cout << "SCUpgradeFailReq" << std::endl;
			}
			else if (pkt->cmd == MemCmd::StoreCondFailReq)
			{
				std::cout << "StoreCondFailReq" << std::endl;
			}
			else if (pkt->cmd == MemCmd::MessageReq)
			{
				std::cout << "MessageReq" << std::endl;
			}
			else if (pkt->cmd == MemCmd::NetworkNackError)
			{
				std::cout << "NetworkNackError" << std::endl;
			}
			else if (pkt->cmd == MemCmd::InvalidDestError)
			{
				std::cout << "InvalidDestError" << std::endl;
			}
			else if (pkt->cmd == MemCmd::BadAddressError)
			{
				std::cout << "BadAddressError" << std::endl;
			}
			else if (pkt->cmd == MemCmd::FunctionalReadError)
			{
				std::cout << "FunctionalReadError" << std::endl;
			}
			else if (pkt->cmd == MemCmd::FunctionalWriteError)
			{
				std::cout << "FunctionalWriteError" << std::endl;
			}
			else if (pkt->cmd == MemCmd::PrintReq)
			{
				std::cout << "PrintReq" << std::endl;
			}
			else if (pkt->cmd == MemCmd::FlushReq)
			{
				std::cout << "FlushReq" << std::endl;
			}
			//sc_core::sc_stop();
		}
		return true;
	}


template<unsigned int BUSWIDTH>
void BridgeClassicToAMBATLM2<BUSWIDTH>::BridgeSlavePort::recvRetry()
{
	bridge->recvRetry();	
}

/** Function called by the port when the bus is receiving a Atomic
 * transaction.*/
template<unsigned int BUSWIDTH>
Tick BridgeClassicToAMBATLM2<BUSWIDTH>::BridgeSlavePort::recvAtomic(PacketPtr pkt)
{
	return bridge->recvAtomic(pkt);
}

template<unsigned int BUSWIDTH>
Tick BridgeClassicToAMBATLM2<BUSWIDTH>::recvAtomic(PacketPtr pkt)
{
    tlm::tlm_generic_payload trans;

//    std::cout << "In " << sc_core::sc_object::name() << " at time " << sc_time_stamp() << " request from GEM5 with address=0x" << hex << pkt->getAddr() << dec << " size=" << pkt->getSize() << std::endl;
    if (pkt->memInhibitAsserted()) {
    	std::cout << "recvAtomic memInhibitAsserted " << hex << pkt->getAddr() << dec << std::endl;
        return 0;
    }
    if (pkt->cmd == MemCmd::SwapReq)
    {
    	panic("SwapReq not supported\n");
    }
    else if (pkt->isRead())
    {
        assert(!pkt->isWrite());
        if (pkt->isLLSC()) {
            trackLoadLocked(pkt);
        }

//        if (pkt->isLLSC())
//        {
//        	panic("isLLSC not yet supported for atomic\n");
//        }
   		trans.set_read();
    	trans.set_address(pkt->getAddr());
    	trans.set_data_length(pkt->getSize());
    	trans.set_data_ptr(pkt->getPtr<unsigned char>());
    	debug_port->transport_dbg(static_cast<tlm::tlm_generic_payload &>(trans));
    	unsigned char * data = pkt->getPtr<unsigned char>();
//		std::cout << "In " << sc_core::sc_object::name() << " at time " << sc_time_stamp() << " sending a READ response to GEM5 with address=0x" << hex << pkt->getAddr() << dec << " size=" << pkt->getSize();
//		std::cout << hex << " and data= [";
//		for(unsigned j=0;j<pkt->getSize(); j++)
//			std::cout << "0x" << uint32_t(data[j]) << ",";
//		std::cout << "]" << dec << std::endl;

    }
    else if (pkt->isWrite())
    {
//    	std::cout << "isWrite " << std::endl;
    	if (writeOK(pkt))
    	{
			trans.set_write();
			/*if (writeOK(pkt)) {
				if (pmemAddr)
					memcpy(hostAddr, pkt->getPtr<uint8_t>(), pkt->getSize());
				assert(!pkt->req->isInstFetch());
			}*/
			trans.set_address(pkt->getAddr());
			trans.set_data_length(pkt->getSize());
			trans.set_data_ptr(pkt->getPtr<unsigned char>());
			debug_port->transport_dbg(static_cast<tlm::tlm_generic_payload &>(trans));
    	}
    	else
        	std::cout << "recvAtomic !writeOK " << hex << pkt->getAddr() << dec << std::endl;
    }
    else if (pkt->isInvalidate())
    {
        //upgrade or invalidate
        if (pkt->needsResponse())
        {
            pkt->makeAtomicResponse();
        }
    }
    else
    {
        panic("unimplemented");
    }

    if (pkt->needsResponse())
    {
        pkt->makeAtomicResponse();
    }
    return 1000;
}

template<unsigned int BUSWIDTH>
AddrRangeList BridgeClassicToAMBATLM2<BUSWIDTH>::BridgeSlavePort::getAddrRanges()
{
	//FIXME: Ugly hack: whole address range minus small portion
	AddrRangeList resp;
	resp.push_back(Range<long long unsigned int>("0x0:+0x10000000"));
	resp.push_back(Range<long long unsigned int>("0x20000000:+0xdfffffff"));
	return resp;
}

template<unsigned int BUSWIDTH>
void BridgeClassicToAMBATLM2<BUSWIDTH>::serialize(ostream &os)
{

	std::cout << "SERIALIZING " << sc_core::sc_object::name() << std::endl;

	uint32_t bytesRead;
    gzFile compressedMem;
    std::string filename = std::string(sc_core::sc_object::name()) + ".physmem";

    SERIALIZE_SCALAR(filename);
    //SERIALIZE_SCALAR(_size); //Hard coded 512 MB FIXME
//    _size = 512*1024*1024;

    // write memory file
    std::string thefile = Checkpoint::dir() + "/" + filename.c_str();
    int fd = creat(thefile.c_str(), 0664);
    if (fd < 0) {
        perror("creat");
        fatal("Can't open physical memory checkpoint file '%s'\n", filename);
    }

    compressedMem = gzdopen(fd, "wb");
    if (compressedMem == NULL)
        fatal("Insufficient memory to allocate compression state for %s\n",
                filename);

//    if (gzwrite(compressedMem, pmemAddr, size()) != (int)size()) {
//        fatal("Write failed on physical memory checkpoint file '%s'\n",
//              filename);
//    }

    const uint32_t chunkSize = 16384; //Could be higher, stay below 64kB for nerios mem store
    uint32_t curSize = 0;
    long *tempPage = (long*)malloc(chunkSize);
    if (tempPage == NULL)
        fatal("Unable to malloc memory to write file %s\n", filename);

    /* Only copy bytes that are non-zero, so we don't give the VM system hell */
	 //FIXME REPLACE USING DMI/!!!!
	tlm::tlm_generic_payload trans;
	trans.set_read();
	trans.set_data_length(chunkSize);
	trans.set_data_ptr((unsigned char*)(tempPage));
    while (curSize < size())
    {
		trans.set_address(curSize);
		debug_port->transport_dbg(static_cast<tlm::tlm_generic_payload &>(trans));
        curSize += bytesRead;
        gzwrite(compressedMem, tempPage, chunkSize);
    }

    free(tempPage);


    if (gzclose(compressedMem))
        fatal("Close failed on physical memory checkpoint file '%s'\n",
              filename);

    typename std::list<LockedAddr>::iterator i = lockedAddrList.begin();

    std::vector<Addr> lal_addr;
    std::vector<int> lal_cid;
    while (i != lockedAddrList.end()) {
        lal_addr.push_back(i->addr);
        lal_cid.push_back(i->contextId);
        i++;
    }
    arrayParamOut(os, "lal_addr", lal_addr);
    arrayParamOut(os, "lal_cid", lal_cid);
}


template<unsigned int BUSWIDTH>
void BridgeClassicToAMBATLM2<BUSWIDTH>::unserialize(Checkpoint *cp, const std::string &section, uint64_t base_address)
{

    gzFile compressedMem;
    long *tempPage;
    long *pmem_current;
    uint64_t curSize;
    uint32_t bytesRead;
    const uint32_t chunkSize = 16384; //Could be higher, stay below 64kB for nerios mem store

    std::string filename;

    UNSERIALIZE_SCALAR(filename);

    filename = cp->cptDir + "/" + filename;

    // mmap memoryfile
    int fd = open(filename.c_str(), O_RDONLY);
    if (fd < 0) {
        perror("open");
        fatal("Can't open physical memory checkpoint file '%s'", filename);
    }

    compressedMem = gzdopen(fd, "rb");
    if (compressedMem == NULL)
        fatal("Insufficient memory to allocate compression state for %s\n",
                filename);

    // unmap file that was mmapped in the constructor
    // This is done here to make sure that gzip and open don't muck with our
    // nice large space of memory before we reallocate it
    //munmap((char*)pmemAddr, size());

//    UNSERIALIZE_SCALAR(_size);
//    if (size() > params()->range.size())
//        fatal("Memory size has changed! size %lld, param size %lld\n",
//              size(), params()->range.size());

//    pmemAddr = (uint8_t *)mmap(NULL, size(),
//        PROT_READ | PROT_WRITE, MAP_ANON | MAP_PRIVATE, -1, 0);

//    if (pmemAddr == (void *)MAP_FAILED) {
//        perror("mmap");
//        fatal("Could not mmap physical memory!\n");
//    }

    curSize = 0;
    tempPage = (long*)malloc(chunkSize);
    if (tempPage == NULL)
        fatal("Unable to malloc memory to read file %s\n", filename);

    /* Only copy bytes that are non-zero, so we don't give the VM system hell */
    while (curSize < size())
    {
        bytesRead = gzread(compressedMem, tempPage, chunkSize);
        if (bytesRead == 0)
            break;

        assert(bytesRead % sizeof(long) == 0);

		tlm::tlm_generic_payload trans;
		 //FIXME REPLACE USING DMI/!!!!
		trans.set_write();
		trans.set_address(base_address + curSize);
		trans.set_data_length(chunkSize);
		trans.set_data_ptr((unsigned char*)(tempPage));
		debug_port->transport_dbg(static_cast<tlm::tlm_generic_payload &>(trans));
        curSize += bytesRead;
    }

    free(tempPage);

    if (gzclose(compressedMem))
        fatal("Close failed on physical memory checkpoint file '%s'\n",
              filename);

    std::vector<Addr> lal_addr;
    std::vector<int> lal_cid;
    arrayParamIn(cp, section, "lal_addr", lal_addr);
    arrayParamIn(cp, section, "lal_cid", lal_cid);
    for(int i = 0; i < lal_addr.size(); i++)
        lockedAddrList.push_front(LockedAddr(lal_addr[i], lal_cid[i]));
}

namespace Stats
{
	void dump()
	{
		std::list<Stats::Info *> listinfo = Stats::statsList();
		Stats::Output * output = Stats::initText("gem5_stats.txt",true);
		for (std::list<Stats::Info *>::iterator iter3=listinfo.begin();iter3!=listinfo.end();++iter3)
		{
			(*iter3)->prepare();
		}
		if (output->valid())
		{
			for (std::list<Stats::Info *>::iterator iter3=listinfo.begin();iter3!=listinfo.end();++iter3)
			{
				(*iter3)->visit(*output);
			}
		}
	}

	void reset()
	{
		std::list<Stats::Info *> listinfo = Stats::statsList();
		for (std::list<Stats::Info *>::iterator iter3=listinfo.begin();iter3!=listinfo.end();++iter3)
		{
			(*iter3)->reset();
		}
	    extern CallbackQueue resetQueue;
	    resetQueue.process();
	}
}

void TemporaryFunctionBridgeClassicToAMBATLM2 ()
{
	BridgeClassicToAMBATLM2<128>* tmp = new BridgeClassicToAMBATLM2<128>("128b_bridgeClassic","ACEmaster","debugMaster");
}
