from m5.objects import *

class L1DCache(Cache):
    size = '32kB'
    assoc = 2
    tag_latency = 2
    data_latency = 2
    response_latency = 2
    mshrs = 4
    tgts_per_mshr = 20
    def __init__(self):
        super(L1DCache, self).__init__()

class L1ICache(Cache):
    size = '32kB'
    assoc = 2
    tag_latency = 2
    data_latency = 2
    response_latency = 2
    mshrs = 4
    tgts_per_mshr = 20
    def __init__(self):
        super(L1ICache, self).__init__()

class L1Cache(Cache):
    size = '64kB'
    assoc = 2
    tag_latency = 2
    data_latency = 2
    response_latency = 2
    mshrs = 4
    tgts_per_mshr = 20
    def __init__(self):
        super(L1Cache, self).__init__()

class L2ICache(Cache):
    size = '128kB'
    assoc = 8
    tag_latency = 20
    data_latency = 20
    response_latency = 20
    mshrs = 20
    tgts_per_mshr = 12
    def __init__(self):
        super(L2ICache, self).__init__()

class L2DCache(Cache):
    size = '128kB'
    assoc = 8
    tag_latency = 20
    data_latency = 20
    response_latency = 20
    mshrs = 20
    tgts_per_mshr = 12
    def __init__(self):
        super(L2DCache, self).__init__()

class L2Cache_SPPV2(Cache):
    size = '256kB'
    assoc = 8
    tag_latency = 20
    data_latency = 20
    response_latency = 20
    mshrs = 20
    tgts_per_mshr = 12
    prefetcher = SignaturePathPrefetcherV2()
    def __init__(self):
        super(L2Cache_SPPV2, self).__init__()
class L2Cache_SPPV2_PPF(Cache):
    size = '256kB'
    assoc = 8
    tag_latency = 20
    data_latency = 20
    response_latency = 20
    mshrs = 20
    tgts_per_mshr = 12
    prefetcher = SPPV2_PPF()
    def __init__(self):
        super(L2Cache_SPPV2_PPF, self).__init__()


class L3Cache(Cache):
    size = '8MB'
    assoc = 8
    tag_latency = 50
    data_latency = 50
    response_latency = 50
    mshrs = 100
    tgts_per_mshr = 8
    def __init__(self):
        super(L3Cache, self).__init__()