


#include <iostream>

#include "nerios_arm_system.hpp"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "cpu/thread_context.hh"
#include "mem/physical.hh"

namespace Debug {

extern SimpleFlag Loader;

} // namespace Debug


using namespace std;
using namespace Linux;

NeriosArmSystem::NeriosArmSystem(Params *p)
    : System(p),bootldr(NULL)
{
    debugPrintkEvent = addKernelFuncEvent<DebugPrintkEvent>("dprintk");

    /*if ((p->boot_loader == "") != (p->boot_loader_mem == NULL))
        fatal("If boot_loader is specifed, memory to load it must be also.\n");

    if (p->boot_loader != "") {
        bootldr = createObjectFile(p->boot_loader);

        if (!bootldr)
            fatal("Could not read bootloader: %s\n", p->boot_loader);

        Port *mem_port;
        FunctionalPort fp(name() + "-fport");
        mem_port = p->boot_loader_mem->getPort("functional");
        fp.setPeer(mem_port);
        mem_port->setPeer(&fp);

        bootldr->loadSections(&fp);
        bootldr->loadGlobalSymbols(debugSymbolTable);

        uint8_t jump_to_bl[] =
        {
            0x07, 0xf0, 0xa0, 0xe1  // branch to r7
        };
        functionalPort->writeBlob(0x0, jump_to_bl, sizeof(jump_to_bl));

        inform("Using bootloader at address %#x\n", bootldr->entryPoint());
    }*/
}

NeriosArmSystem::~NeriosArmSystem()
{
    if (debugPrintkEvent)
        delete debugPrintkEvent;
}


NeriosArmSystem *
NeriosArmSystemParams::create()
{
    return new NeriosArmSystem(this);
}

void NeriosArmSystem::initState()
{
    System::initState();

   /**
     * Load the kernel code into memory
     */
        
	// Load kernel code
	kernel = createObjectFile(params()->kernel);
	inform("kernel located at: %s", params()->kernel);

	if (kernel == NULL)
		fatal("Could not load kernel file %s", params()->kernel);

	// Load program sections into memory
	kernel->loadSections(physProxy, loadAddrMask);

	// setup entry points
	kernelStart = kernel->textBase();
	kernelEnd = kernel->bssBase() + kernel->bssSize();
	kernelEntry = kernel->entryPoint();

	// load symbols
	if (!kernel->loadGlobalSymbols(kernelSymtab))
		fatal("could not load kernel symbols\n");

	if (!kernel->loadLocalSymbols(kernelSymtab))
		fatal("could not load kernel local symbols\n");

	if (!kernel->loadGlobalSymbols(debugSymbolTable))
		fatal("could not load kernel symbols\n");

	if (!kernel->loadLocalSymbols(debugSymbolTable))
		fatal("could not load kernel local symbols\n");

	DPRINTF(Loader, "Kernel start = %#x\n", kernelStart);
	DPRINTF(Loader, "Kernel end   = %#x\n", kernelEnd);
	DPRINTF(Loader, "Kernel entry = %#x\n", kernelEntry);
	DPRINTF(Loader, "Kernel loaded...\n");
	

   for (int i = 0; i < threadContexts.size(); i++) {
    // Set the initial PC to be at start of the kernel code
    threadContexts[i]->pcState(kernelEntry & loadAddrMask);
        if (params()->midr_regval) {
            threadContexts[i]->setMiscReg(ArmISA::MISCREG_MIDR,
                    params()->midr_regval);
        }
    }
};
