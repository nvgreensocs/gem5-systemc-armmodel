/*
 * Copyright (c) 2010 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Ali Saidi
 */

#ifndef __NERIOS_ARM_SYSTEM_HPP__
#define __NERIOS_ARM_SYSTEM_HPP__

#include <string>
#include <vector>
#include <cstddef>
#include "base/types.hh"
#include <cstddef>
#include "params/SimpleMemory.hh"
#include <cstddef>
#include "base/types.hh"
#include <cstddef>
#include <string>
#include <cstddef>
#include "base/types.hh"

#include "params/System.hh"
#include "kern/linux/events.hh"
#include "sim/sim_object.hh"
#include "sim/system.hh"


class NeriosArmSystem;

struct NeriosArmSystemParams
    : public SystemParams
{
    Addr gic_cpu_addr;
    PhysicalMemory * boot_loader_mem;
    uint32_t midr_regval;
    std::string boot_loader;
    Addr flags_addr;
    NeriosArmSystem * create();
};

class NeriosArmSystem : public System
{
  protected:
    /**
     * PC based event to skip the dprink() call and emulate its
     * functionality
     */
    Linux::DebugPrintkEvent *debugPrintkEvent;

    /**
     * Pointer to the bootloader object
     */
    ObjectFile *bootldr;

  public:
    typedef NeriosArmSystemParams Params;
    Params * params() const
    {
        return dynamic_cast<Params *>(_params);
    }

    NeriosArmSystem(Params *p);
    ~NeriosArmSystem();

    void initState();

    /** Check if an address should be uncacheable until all caches are enabled.
     * This exits because coherence on some addresses at boot is maintained via
     * sw coherence until the caches are enbaled. Since we don't support sw
     * coherence operations in gem5, this is a method that allows a system
     * type to designate certain addresses that should remain uncachebale
     * for a while.
     */
    virtual bool adderBootUncacheable(Addr a) { return false; }

    virtual Addr fixFuncEventAddr(Addr addr)
    {
        // Remove the low bit that thumb symbols have set
        // but that aren't actually odd aligned
        if (addr & 0x1)
            return addr & ~1;
        return addr;
    }
};

#endif // __NERIOS_ARM_SYSTEM_HPP__

