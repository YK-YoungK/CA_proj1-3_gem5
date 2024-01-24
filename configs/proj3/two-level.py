import m5
from m5.objects import *
from new_cache import *

from optparse import OptionParser
parser = OptionParser()
parser.add_option('--workload', type="string", default="hello")
parser.add_option('--cacheRP', type="string", default="lru")
parser.add_option('--ppf', action="store_true")
parser.add_option('--train_threshold', type="int", default=20)
parser.add_option('--max_prefetch_depth', type="int", default=20)
parser.add_option('--predict_threshold', type="int", default=-20)
parser.add_option('--delta', type="int", default=0)
(options, args) = parser.parse_args()

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
root.system.cpu = DerivO3CPU()
root.system.cpu.max_insts_any_thread = 1e+9             #set maximum instructions as 1 billion


# Create system bus
root.system.membus = SystemXBar()

# Project 3: Add cache
root.system.cpu.icache = L1ICache()
# root.system.cpu.icache.replacement_policy = MyMRURP()
root.system.cpu.dcache = L1DCache()
# root.system.cpu.dcache.replacement_policy = MyMRURP()
# root.system.l2cache = L2Cache()
if options.ppf:
    root.system.l2cache = L2Cache_SPPV2_PPF()
    root.system.l2cache.prefetcher.train_threshold = options.train_threshold
    root.system.l2cache.prefetcher.max_prefetch_depth = options.max_prefetch_depth
    root.system.l2cache.prefetcher.predict_threshold = options.predict_threshold
    root.system.l2cache.prefetcher.delta = options.delta
else:
    root.system.l2cache = L2Cache_SPPV2()
# root.system.l2cache.replacement_policy = MyMRURP()

if options.cacheRP == "random":
    root.system.cpu.icache.replacement_policy = RandomRP()
    root.system.cpu.dcache.replacement_policy = RandomRP()
    root.system.l2cache.replacement_policy = RandomRP()
elif options.cacheRP == "lru":
    root.system.cpu.icache.replacement_policy = LRURP()
    root.system.cpu.dcache.replacement_policy = LRURP()
    root.system.l2cache.replacement_policy = LRURP()
elif options.cacheRP == "mru":
    root.system.cpu.icache.replacement_policy = MRURP()
    root.system.cpu.dcache.replacement_policy = MRURP()
    root.system.l2cache.replacement_policy = MRURP()
elif options.cacheRP == "clock":
    root.system.cpu.icache.replacement_policy = MyCLOCKRP()
    root.system.cpu.dcache.replacement_policy = MyCLOCKRP()
    root.system.l2cache.replacement_policy = MyCLOCKRP()
else:
    print("Not supported cache replacement policy.")
    exit(1)

# Describe the interconnect logic
#root.system.cpu.icache_port = root.system.membus.cpu_side_ports     # ICache port
#root.system.cpu.dcache_port = root.system.membus.cpu_side_ports     # DCache port
root.system.cpu.icache.cpu_side = root.system.cpu.icache_port
root.system.cpu.dcache.cpu_side = root.system.cpu.dcache_port

root.system.l2bus = L2XBar()
root.system.cpu.icache.mem_side = root.system.l2bus.cpu_side_ports
root.system.cpu.dcache.mem_side = root.system.l2bus.cpu_side_ports
root.system.l2cache.cpu_side = root.system.l2bus.mem_side_ports
root.system.l2cache.mem_side = root.system.membus.cpu_side_ports

root.system.mem_ctrl.port = root.system.membus.mem_side_ports       # memory controller port
root.system.cpu.createInterruptController()                         # interrupt controller
root.system.system_port = root.system.membus.cpu_side_ports


# Part II. Execution
cmd = {
    'hello': ['tests/test-progs/hello/bin/arm/linux/hello'],
    '2mm_base': ['test_bench/2MM/2mm_base'],
    'bfs': ['test_bench/BFS/bfs','-f','test_bench/BFS/USA-road-d.NY.gr'],
    'bzip2': ['test_bench/bzip2/bzip2_base.amd64-m64-gcc42-nn','test_bench/bzip2/input.source','280'],
    'mcf': ['test_bench/mcf/mcf_base.amd64-m64-gcc42-nn','test_bench/mcf/inp.in']
}
exe = {
    'hello': 'tests/test-progs/hello/bin/arm/linux/hello',
    '2mm_base': 'test_bench/2MM/2mm_base',
    'bfs': 'test_bench/BFS/bfs',
    'bzip2': 'test_bench/bzip2/bzip2_base.amd64-m64-gcc42-nn',
    'mcf': 'test_bench/mcf/mcf_base.amd64-m64-gcc42-nn'
}
# exe_path = 'tests/test-progs/hello/bin/arm/linux/hello'
exe_path = exe[options.workload]
root.system.workload = SEWorkload.init_compatible(exe_path)         # set system workload
process = Process()                                                 # create process
# process.cmd = [exe_path]
process.cmd = cmd[options.workload]
root.system.cpu.workload = process                                  # set cpu to execute
root.system.cpu.createThreads()                                     # create contexts


# Part III. Run
m5.instantiate()
exit_event = m5.simulate()
print('Exiting @ tick {} because {}'.format(m5.curTick(), exit_event.getCause()))