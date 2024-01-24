import m5
from m5.objects import *
from new_cache import *

# Part I. Simulated System
# Simulate stsyem with SE mode
root = Root(full_system = False, system = System())

# Set up clock domain
root.system.clk_domain = SrcClockDomain()
root.system.clk_domain.clock = '2GHz'
root.system.clk_domain.voltage_domain = VoltageDomain()

# Create memory and memory controller
root.system.mem_mode = 'timing'         # most situations
root.system.mem_ranges = [AddrRange ('2GB')]
root.system.mem_ctrl = MemCtrl()
root.system.mem_ctrl.dram = DDR4_2400_16x4()
root.system.mem_ctrl.dram.range = root.system.mem_ranges[0]

# Create CPU
root.system.cpu = TimingSimpleCPU()

# Create system bus
root.system.membus = SystemXBar()

# Project 3: Add cache
root.system.cpu.icache = L1ICache()
root.system.cpu.dcache = L1DCache()
root.system.cpu.l2icache = L2ICache()
root.system.cpu.l2dcache = L2DCache()

# Describe the interconnect logic
#root.system.cpu.icache_port = root.system.membus.cpu_side_ports     # ICache port
#root.system.cpu.dcache_port = root.system.membus.cpu_side_ports     # DCache port
root.system.cpu.icache.cpu_side = root.system.cpu.icache_port
root.system.cpu.dcache.cpu_side = root.system.cpu.dcache_port

root.system.cpu.l2ibus = L2XBar()
root.system.cpu.icache.mem_side = root.system.cpu.l2ibus.cpu_side_ports
root.system.cpu.l2icache.cpu_side = root.system.cpu.l2ibus.mem_side_ports
root.system.cpu.l2icache.mem_side = root.system.membus.cpu_side_ports

root.system.cpu.l2dbus = L2XBar()
root.system.cpu.dcache.mem_side = root.system.cpu.l2dbus.cpu_side_ports
root.system.cpu.l2dcache.cpu_side = root.system.cpu.l2dbus.mem_side_ports
root.system.cpu.l2dcache.mem_side = root.system.membus.cpu_side_ports


root.system.mem_ctrl.port = root.system.membus.mem_side_ports       # memory controller port
root.system.cpu.createInterruptController()                         # interrupt controller
root.system.system_port = root.system.membus.cpu_side_ports


# Part II. Execution
exe_path = 'tests/test-progs/hello/bin/arm/linux/hello'
root.system.workload = SEWorkload.init_compatible(exe_path)         # set system workload
process = Process()                                                 # create process
process.cmd = [exe_path]
root.system.cpu.workload = process                                  # set cpu to execute
root.system.cpu.createThreads()                                     # create contexts


# Part III. Run
m5.instantiate()
exit_event = m5.simulate()
print('Exiting @ tick {} because {}'.format(m5.curTick(), exit_event.getCause()))