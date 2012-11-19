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

File	:	bridge_classic_to_ambatlm2.hpp

Purpose	:	Class definition for the cortex A15 model

Description:

inspired from first attempt at TLM2 by ARM in Gem5

******************************************************************CE*/


#ifndef __MEM_BRIDGE_CLASSIC_TO_AMBATLM2_HPP__
#define __MEM_BRIDGE_CLASSIC_TO_AMBATLM2_HPP__

#include "greencontrol/config.h"

#include "mem/simple_mem.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "sim/eventq.hh"
#include "params/SimpleMemory.hh"
#include "amba.h"
#include <list>

namespace Debug
{
	extern SimpleFlag LLSC;
}

template<unsigned int BUSWIDTH>
class BridgeClassicToAMBATLM2 : public SimpleMemory,
                                public sc_core::sc_module,
                                public tlm::tlm_bw_transport_if<>
{
    protected:

    class BridgeSlavePort : public SlavePort
    {
        /** A pointer to the bridge to which this port belongs. */
        BridgeClassicToAMBATLM2 *bridge;


      public:
        BridgeSlavePort(const std::string &_name, BridgeClassicToAMBATLM2 *_bridge)
         : SlavePort(_name, _bridge), bridge (_bridge)
         {
		 };

      protected:

         /** When receiving a timing request from the peer port,
             pass it to the bridge. */
         virtual bool recvTimingReq(PacketPtr pkt) { return bridge->recvTimingReq(pkt); };

        /** When receiving a retry request from the peer port,
            pass it to the bridge. */
        virtual void recvRetry();

        /** When receiving a Atomic requestfrom the peer port,
            pass it to the bridge. */
        virtual Tick recvAtomic(PacketPtr pkt);

        /** When receiving a Functional request from the peer port,
            pass it to the bridge. */
        virtual void recvFunctional(PacketPtr pkt) { bridge->recvFunctional(pkt); };

        /** When receiving a address range request the peer port,
            pass it to the bridge. */
        virtual AddrRangeList getAddrRanges();

    };


    // From GEM5 to cope with LLSC in atomic mode

    class LockedAddr
    {
        public:
          // on alpha, minimum LL/SC granularity is 16 bytes, so lower
          // bits need to masked off.
          static const Addr Addr_Mask = 0xf;

          static Addr mask(Addr paddr) { return (paddr & ~Addr_Mask); }

          Addr addr;      // locked address
          int contextId;     // locking hw context

          // check for matching execution context
          bool matchesContext(Request *req)
          {
              return (contextId == req->contextId());
          }

          LockedAddr(Request *req)
              : addr(mask(req->getPaddr())),
                contextId(req->contextId())
          {
          }
          // constructor for unserialization use
          LockedAddr(Addr _addr, int _cid)
              : addr(_addr), contextId(_cid)
          {
          }
      };

      std::list<LockedAddr> lockedAddrList;


	
  public:

    amba::amba_master_socket<> master_sock;
    
   	// Debug port.
    tlm::tlm_initiator_socket<> debug_port;


	/** Constructor **/
	 
	SC_HAS_PROCESS(BridgeClassicToAMBATLM2);
	BridgeClassicToAMBATLM2(sc_core::sc_module_name nm, std::string ace_master_name, std::string debug_master_name);

    /** Function called by the port when the bus is recieving a Timing
      transaction.*/
    bool recvTimingReq(PacketPtr pkt);

    /** Function called by the port when the bus is recieving a Atomic
      transaction.*/
    Tick recvAtomic(PacketPtr pkt);

    /** Function called by the port when the bus is recieving a Functional
        transaction.*/
    void recvFunctional(PacketPtr pkt);

    /** Timing function called by port when it is once again able to process
     * requests. */
    void recvRetry();

    virtual void init();
    
    virtual SlavePort& getSlavePort(const std::string& if_name, int idx = -1)
    {
       if (if_name == "slave") {
            if (slavePort != NULL)
                fatal("%s: slavePort port already connected",
                      sc_core::sc_object::name());
            slavePort = new BridgeSlavePort("classic2ambaBridge",this);
            return *slavePort;
        } else if (if_name == "func") {
            if (funcPort != NULL)
                fatal("%s: slavePort port already connected",
                      sc_core::sc_object::name());
            funcPort = new BridgeSlavePort("funcBridge",this);
            return *funcPort;
        } else
            return *(new BridgeSlavePort("DUMMY",this));
     }

    virtual tlm::tlm_sync_enum nb_transport_bw(tlm::tlm_generic_payload& trans,tlm::tlm_phase& phase,sc_core::sc_time& t)
	{
		return tlm::TLM_COMPLETED; // Dummy implementation
	}
    virtual void invalidate_direct_mem_ptr(sc_dt::uint64 start_range, sc_dt::uint64 end_range)
	{ } // Dummy implementation

    virtual void unserialize(Checkpoint *cp, const std::string &section, uint64_t base_address);
    void serialize(ostream &os);

  protected:

    /**
     * The slave port
     */
    BridgeSlavePort* slavePort;
    BridgeSlavePort* funcPort;

  private:

	gs::cnf::cnf_api *m_Api;

    /** TLM2 master **/
    
    sc_core::sc_time clk_period;
    unsigned int counter;
    void sendWriteData();
	void slaveTimingListener(gs::socket::timing_info);
	tlm::tlm_sync_enum nb_bw_transport(tlm::tlm_generic_payload & trans, tlm::tlm_phase & ph, sc_core::sc_time &delay);
	void end_of_elaboration();
	void send_dummy_response();
    sc_core::sc_event_queue send_dummy_response_event;
    /** Used to remember if recvTimingReq needs to be retried */
    bool needRetry;
    bool needWrRetry;
    bool needWrIdRetry;
    bool needRdIdRetry;
    /** Used to remember if Bus is stalling */
    bool bus_stalling;
    std::queue<PacketPtr> retryQueue;
    sc_core::sc_fifo<uint32_t> writeDataQueue;
    sc_core::sc_event wenable_event;
    sc_core::sc_event end_req;
    bool need_wenable_event;
    
    PacketPtr rd_packets[128];
    PacketPtr wr_packets[128];

	std::queue<PacketPtr> dummy_pkt;

    // Add load-locked to tracking list.  Should only be called if the
    // operation is a load and the LLSC flag is set.
    void trackLoadLocked(PacketPtr pkt)
    {
        Request *req = pkt->req;
        Addr paddr = LockedAddr::mask(req->getPaddr());

        // first we check if we already have a locked addr for this
        // xc.  Since each xc only gets one, we just update the
        // existing record with the new address.
        typename std::list<LockedAddr>::iterator i;

        for (i = lockedAddrList.begin(); i != lockedAddrList.end(); ++i) {
            if (i->matchesContext(req)) {
                DPRINTF(LLSC, "Modifying lock record: context %d addr %#x\n",
                        req->contextId(), paddr);
                i->addr = paddr;
                return;
            }
        }

        // no record for this xc: need to allocate a new one
        DPRINTF(LLSC, "Adding lock record: context %d addr %#x\n",
                req->contextId(), paddr);
        lockedAddrList.push_front(LockedAddr(req));
    }


    // Called on *writes* only... both regular stores and
    // store-conditional operations.  Check for conventional stores which
    // conflict with locked addresses, and for success/failure of store
    // conditionals.

    bool checkLockedAddrList(PacketPtr pkt)
    {
        Request *req = pkt->req;
        Addr paddr = LockedAddr::mask(req->getPaddr());
        bool isLLSC = pkt->isLLSC();

        // Initialize return value.  Non-conditional stores always
        // succeed.  Assume conditional stores will fail until proven
        // otherwise.
        bool success = !isLLSC;

        // Iterate over list.  Note that there could be multiple matching
        // records, as more than one context could have done a load locked
        // to this location.
        typename std::list<LockedAddr>::iterator i = lockedAddrList.begin();

        while (i != lockedAddrList.end()) {

            if (i->addr == paddr) {
                // we have a matching address

                if (isLLSC && i->matchesContext(req)) {
                    // it's a store conditional, and as far as the memory
                    // system can tell, the requesting context's lock is
                    // still valid.
                    DPRINTF(LLSC, "StCond success: context %d addr %#x\n",
                            req->contextId(), paddr);
                    success = true;
                }

                // Get rid of our record of this lock and advance to next
                DPRINTF(LLSC, "Erasing lock record: context %d addr %#x\n",
                        i->contextId, paddr);
                i = lockedAddrList.erase(i);
            }
            else {
                // no match: advance to next record
                ++i;
            }
        }

        if (isLLSC) {
            req->setExtraData(success ? 1 : 0);
        }

        return success;
    }

    // Compare a store address with any locked addresses so we can
    // clear the lock flag appropriately.  Return value set to 'false'
    // if store operation should be suppressed (because it was a
    // conditional store and the address was no longer locked by the
    // requesting execution context), 'true' otherwise.  Note that
    // this method must be called on *all* stores since even
    // non-conditional stores must clear any matching lock addresses.
    bool writeOK(PacketPtr pkt) {
        Request *req = pkt->req;
        if (lockedAddrList.empty()) {
            // no locked addrs: nothing to check, store_conditional fails
            bool isLLSC = pkt->isLLSC();
            if (isLLSC) {
                req->setExtraData(0);
            }
            return !isLLSC; // only do write if not an sc
        } else {
            // iterate over list...
            return checkLockedAddrList(pkt);
        }
    }

};


#endif //__MEM_BRIDGE_CLASSIC_TO_AMBATLM2_HPP__
