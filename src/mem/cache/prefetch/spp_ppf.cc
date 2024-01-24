/**
 * Copyright (c) 2018 Metempsy Technology Consulting
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
 */

#include "mem/cache/prefetch/spp_ppf.hh"

#include <cassert>
#include <climits>

#include "debug/HWPrefetch.hh"
#include "mem/cache/prefetch/associative_set_impl.hh"
#include "params/SPP_PPF.hh"

namespace gem5
{

namespace prefetch
{

SPP_PPF::SPP_PPF(const SPP_PPFParams &p)
    : Queued(p),
      stridesPerPatternEntry(p.strides_per_pattern_entry),
      signatureShift(p.signature_shift),
      signatureBits(p.signature_bits),
      prefetchConfidenceThreshold(p.prefetch_confidence_threshold),
      lookaheadConfidenceThreshold(p.lookahead_confidence_threshold),
      signatureTable(p.signature_table_assoc, p.signature_table_entries,
                     p.signature_table_indexing_policy,
                     p.signature_table_replacement_policy),
      patternTable(p.pattern_table_assoc, p.pattern_table_entries,
                   p.pattern_table_indexing_policy,
                   p.pattern_table_replacement_policy,
                   PatternEntry(stridesPerPatternEntry, p.num_counter_bits)),
      delta(p.delta), train_threshold(p.train_threshold), 
      max_prefetch_depth(p.max_prefetch_depth),
      predict_threshold(p.predict_threshold)
{
    fatal_if(prefetchConfidenceThreshold < 0,
        "The prefetch confidence threshold must be greater than 0\n");
    fatal_if(prefetchConfidenceThreshold > 1,
        "The prefetch confidence threshold must be less than 1\n");
    fatal_if(lookaheadConfidenceThreshold < 0,
        "The lookahead confidence threshold must be greater than 0\n");
    fatal_if(lookaheadConfidenceThreshold > 1,
        "The lookahead confidence threshold must be less than 1\n");
    
    
    
    pc1 = 0; pc2 = 0; pc3 = 0;
    for (int _ = 0; _ < 1024; ++_)
        prefetch_table[_] = prefetch_entry();
    for (int _ = 0; _ < 1024; ++_)
        reject_table[_] = prefetch_entry();
}

SPP_PPF::PatternStrideEntry &
SPP_PPF::PatternEntry::getStrideEntry(stride_t stride)
{
    PatternStrideEntry *pstride_entry = findStride(stride);
    if (pstride_entry == nullptr) {
        // Specific replacement algorithm for this table,
        // pick the entry with the lowest counter value,
        // then decrease the counter of all entries

        // If all counters have the max value, this will be the pick
        PatternStrideEntry *victim_pstride_entry = &(strideEntries[0]);

        unsigned long current_counter = ULONG_MAX;
        for (auto &entry : strideEntries) {
            if (entry.counter < current_counter) {
                victim_pstride_entry = &entry;
                current_counter = entry.counter;
            }
            entry.counter--;
        }
        pstride_entry = victim_pstride_entry;
        pstride_entry->counter.reset();
        pstride_entry->stride = stride;
    }
    return *pstride_entry;
}

void
SPP_PPF::addPrefetch(Addr ppn, stride_t last_block,
    stride_t delta, double path_confidence, signature_t signature,
    bool is_secure, std::vector<AddrPriority> &addresses)
{
    stride_t block = last_block + delta;

    Addr pf_ppn;
    stride_t pf_block;
    if (block < 0) {
        stride_t num_cross_pages = 1 + (-block) / (pageBytes/blkSize);
        if (num_cross_pages > ppn) {
            // target address smaller than page 0, ignore this request;
            return;
        }
        pf_ppn = ppn - num_cross_pages;
        pf_block = block + (pageBytes/blkSize) * num_cross_pages;
        handlePageCrossingLookahead(signature, last_block, delta,
                                    path_confidence);
    } else if (block >= (pageBytes/blkSize)) {
        stride_t num_cross_pages = block / (pageBytes/blkSize);
        if (MaxAddr/pageBytes < (ppn + num_cross_pages)) {
            // target address goes beyond MaxAddr, ignore this request;
            return;
        }
        pf_ppn = ppn + num_cross_pages;
        pf_block = block - (pageBytes/blkSize) * num_cross_pages;
        handlePageCrossingLookahead(signature, last_block, delta,
                                    path_confidence);
    } else {
        pf_ppn = ppn;
        pf_block = block;
    }

    Addr new_addr = pf_ppn * pageBytes;
    new_addr += pf_block * (Addr)blkSize;

    DPRINTF(HWPrefetch, "Queuing prefetch to %#x.\n", new_addr);
    addresses.push_back(AddrPriority(new_addr, 0));
}

void
SPP_PPF::handleSignatureTableMiss(stride_t current_block,
    signature_t &new_signature, double &new_conf, stride_t &new_stride)
{
    new_signature = current_block;
    new_conf = 1.0;
    new_stride = current_block;
}

void
SPP_PPF::increasePatternEntryCounter(
        PatternEntry &pattern_entry, PatternStrideEntry &pstride_entry)
{
    pstride_entry.counter++;
}

void
SPP_PPF::updatePatternTable(Addr signature, stride_t stride)
{
    assert(stride != 0);
    // The pattern table is indexed by signatures
    PatternEntry &p_entry = getPatternEntry(signature);
    PatternStrideEntry &ps_entry = p_entry.getStrideEntry(stride);
    increasePatternEntryCounter(p_entry, ps_entry);
}

SPP_PPF::SignatureEntry &
SPP_PPF::getSignatureEntry(Addr ppn, bool is_secure,
        stride_t block, bool &miss, stride_t &stride,
        double &initial_confidence)
{
    SignatureEntry* signature_entry = signatureTable.findEntry(ppn, is_secure);
    if (signature_entry != nullptr) {
        signatureTable.accessEntry(signature_entry);
        miss = false;
        stride = block - signature_entry->lastBlock;
    } else {
        signature_entry = signatureTable.findVictim(ppn);
        assert(signature_entry != nullptr);

        // Sets signature_entry->signature, initial_confidence, and stride
        handleSignatureTableMiss(block, signature_entry->signature,
            initial_confidence, stride);

        signatureTable.insertEntry(ppn, is_secure, signature_entry);
        miss = true;
    }
    signature_entry->lastBlock = block;
    return *signature_entry;
}

SPP_PPF::PatternEntry &
SPP_PPF::getPatternEntry(Addr signature)
{
    PatternEntry* pattern_entry = patternTable.findEntry(signature, false);
    if (pattern_entry != nullptr) {
        // Signature found
        patternTable.accessEntry(pattern_entry);
    } else {
        // Signature not found
        pattern_entry = patternTable.findVictim(signature);
        assert(pattern_entry != nullptr);

        patternTable.insertEntry(signature, false, pattern_entry);
    }
    return *pattern_entry;
}

double
SPP_PPF::calculatePrefetchConfidence(PatternEntry const &sig,
        PatternStrideEntry const &entry) const
{
    return entry.counter.calcSaturation();
}

double
SPP_PPF::calculateLookaheadConfidence(PatternEntry const &sig,
        PatternStrideEntry const &lookahead) const
{
    double lookahead_confidence = lookahead.counter.calcSaturation();
    if (lookahead_confidence > 0.95) {
        /**
         * maximum confidence is 0.95, guaranteeing that
         * current confidence will eventually fall beyond
         * the threshold
         */
        lookahead_confidence = 0.95;
    }
    return lookahead_confidence;
}

void
SPP_PPF::calculatePrefetch(const PrefetchInfo &pfi,
                                 std::vector<AddrPriority> &addresses)
{
    Addr request_addr = pfi.getAddr();
    Addr ppn = request_addr / pageBytes;
    stride_t current_block = (request_addr % pageBytes) / blkSize;
    stride_t stride;
    bool is_secure = pfi.isSecure();
    double initial_confidence = 1.0;

    // Get the SignatureEntry of this page to:
    // - compute the current stride
    // - obtain the current signature of accesses
    bool miss;
    SignatureEntry &signature_entry = getSignatureEntry(ppn, is_secure,
            current_block, miss, stride, initial_confidence);

    if (miss) {
        // No history for this page, can't continue
        return;
    }

    if (stride == 0) {
        // Can't continue with a stride 0
        return;
    }

    // Update the confidence of the current signature
    updatePatternTable(signature_entry.signature, stride);

    // Update the current SignatureEntry signature
    signature_entry.signature =
        updateSignature(signature_entry.signature, stride);

    signature_t current_signature = signature_entry.signature;
    double current_confidence = initial_confidence;
    stride_t current_stride = signature_entry.lastBlock;

    // Look for prefetch candidates while the current path confidence is
    // high enough
    for (int depth = 0; depth < max_prefetch_depth; ++depth) {
        // With the updated signature, attempt to generate prefetches
        // - search the PatternTable and select all entries with enough
        //   confidence, these are prefetch candidates
        // - select the entry with the highest counter as the "lookahead"
        PatternEntry *current_pattern_entry =
            patternTable.findEntry(current_signature, false);
        PatternStrideEntry const *lookahead = nullptr;
        if (current_pattern_entry != nullptr) {
            unsigned long max_counter = 0;
            for (auto const &entry : current_pattern_entry->strideEntries) {
                //select the entry with the maximum counter value as lookahead
                if (max_counter < entry.counter) {
                    max_counter = entry.counter;
                    lookahead = &entry;
                }
                // double prefetch_confidence =
                //     calculatePrefetchConfidence(*current_pattern_entry, entry);
                Addr prefetch_addr = calculate_prefetch_addr(ppn, current_stride, entry.stride, current_confidence, current_signature, is_secure, addresses);
                if (prefetch_addr == 0)
                    continue;
                bool prefetch_result = predict_perceptron(pfi, depth, current_confidence, current_signature, prefetch_addr);

                if (prefetch_result && entry.stride != 0) {
                    // assert(entry.stride != 0);
                    //prefetch candidate
                    addPrefetch(ppn, current_stride, entry.stride,
                                current_confidence, current_signature,
                                is_secure, addresses);
                }
            }
        }

        if (lookahead != nullptr) {
            current_confidence *= calculateLookaheadConfidence(
                    *current_pattern_entry, *lookahead);
            current_signature =
                updateSignature(current_signature, lookahead->stride);
            current_stride += lookahead->stride;
        } else {
            current_confidence = 0.0;
        }
        if (current_confidence <= lookaheadConfidenceThreshold)
            break;
    }

    auxiliaryPrefetcher(ppn, current_block, is_secure, addresses);
}

void
SPP_PPF::auxiliaryPrefetcher(Addr ppn, stride_t current_block,
        bool is_secure, std::vector<AddrPriority> &addresses)
{
    if (addresses.empty()) {
        // Enable the next line prefetcher if no prefetch candidates are found
        addPrefetch(ppn, current_block, 1, 0.0 /* unused*/, 0 /* unused */,
                    is_secure, addresses);
    }
}

void SPP_PPF::notifyEviction(const PacketPtr &pkt)
{
    DPRINTF(HWPrefetch, "notifyEviction\n");
    Addr evicted_addr = pkt->getAddr();
    // Train
    update_perceptron(evicted_addr, true);
}

void SPP_PPF::notifyAccess(const PacketPtr &pkt)
{
    DPRINTF(HWPrefetch, "notifyAccess\n");
    Addr evicted_addr = pkt->getAddr();
    // Train
    update_perceptron(evicted_addr, false);
}

bool SPP_PPF::predict_perceptron(const PrefetchInfo &pfi, int depth, double confidence, signature_t sig, Addr prefetch_addr)
{
    Addr request_addr = pfi.getAddr();
    Addr ppn = request_addr / pageBytes;
    int16_t _phy_address_entry = request_addr & 4095;
    int16_t _page_address_entry = ppn & 4095;

    Addr pc = 0;
    if (pfi.hasPC())
        pc = pfi.getPC();
    int16_t _pc_depth_entry = (pc ^ depth) & 4095;

    int16_t _pc_three_entry = ((pc1 ^ (pc2 >> 1)) ^ (pc3 >> 2)) & 4095;
    pc3 = pc2; pc2 = pc1; pc1 = pc;
    
    int16_t _pc_delta_entry = (pc ^ delta) & 4095;

    int confi = int(confidence * 10);
    int16_t _page_confi_entry = (ppn ^ confi) & 4095;
    int16_t _sig_delta_entry = (sig ^ delta) & 4095;

    int result = phy_address_table[_phy_address_entry]       // Physical Address
    + page_address_table[_page_address_entry]      // Page Address
    + pc_depth_table[_pc_depth_entry]          // PC XOR Depth
    + pc_three_table[_pc_three_entry]          // PC1 XOR (PC2>>1) XOR (PC3>>2)
    + pc_delta_table[_pc_delta_entry]          // PC XOR Delta
    + page_confi_table[_page_confi_entry]        // Page Address XOR Confidence
    + sig_delta_table[_sig_delta_entry];
    bool predict = (result >= predict_threshold);

    /* store in the table */
    if (predict == true)
    {
        prefetch_table[(prefetch_addr & 1023)] = prefetch_entry(prefetch_addr, 
        _phy_address_entry,
        _page_address_entry,
        _pc_depth_entry,
        _pc_three_entry,
        _pc_delta_entry,
        _page_confi_entry,
        _sig_delta_entry, 
        result,
        true);
    }
    else
    {
        reject_table[(prefetch_addr & 1023)] = prefetch_entry(prefetch_addr, 
        _phy_address_entry,
        _page_address_entry,
        _pc_depth_entry,
        _pc_three_entry,
        _pc_delta_entry,
        _page_confi_entry,
        _sig_delta_entry,
        result,
        true);
    }
    return predict;
}

void SPP_PPF::update_perceptron(Addr evict_addr, bool isevict = false)
{
    int addr_index = evict_addr & 1023;
    if (isevict == true)
    /* Eviction, check whether the address appears in the prefetch table; if so, indicating a misprediction */
    {
        if (prefetch_table[addr_index].address == evict_addr && prefetch_table[addr_index].valid == true)
        {
            phy_address_table[prefetch_table[addr_index].phy_address_entry] -= 1;       // Physical Address
            page_address_table[prefetch_table[addr_index].page_address_entry] -= 1;      // Page Address
            pc_depth_table[prefetch_table[addr_index].pc_depth_entry] -= 1;          // PC XOR Depth
            pc_three_table[prefetch_table[addr_index].pc_three_entry] -= 1;          // PC1 XOR (PC2>>1) XOR (PC3>>2)
            pc_delta_table[prefetch_table[addr_index].pc_delta_entry] -= 1;          // PC XOR Delta
            page_confi_table[prefetch_table[addr_index].page_confi_entry] -= 1;        // Page Address XOR Confidence
            sig_delta_table[prefetch_table[addr_index].sig_delta_entry] -= 1;
            prefetch_table[addr_index].valid = false;
        }
    }
    else
    /* Demand access, the address is expected to be in prefetch_table with enough confidence */
    {
        if (prefetch_table[addr_index].address == evict_addr && prefetch_table[addr_index].prediction_result <= predict_threshold + train_threshold && prefetch_table[addr_index].valid == true)
        {
            phy_address_table[prefetch_table[addr_index].phy_address_entry] += 1;       // Physical Address
            page_address_table[prefetch_table[addr_index].page_address_entry] += 1;      // Page Address
            pc_depth_table[prefetch_table[addr_index].pc_depth_entry] += 1;          // PC XOR Depth
            pc_three_table[prefetch_table[addr_index].pc_three_entry] += 1;          // PC1 XOR (PC2>>1) XOR (PC3>>2)
            pc_delta_table[prefetch_table[addr_index].pc_delta_entry] += 1;          // PC XOR Delta
            page_confi_table[prefetch_table[addr_index].page_confi_entry] += 1;        // Page Address XOR Confidence
            sig_delta_table[prefetch_table[addr_index].sig_delta_entry] += 1;
            prefetch_table[addr_index].valid = false;
        }
        if (reject_table[addr_index].address == evict_addr && reject_table[addr_index].valid == true)
        {
            phy_address_table[prefetch_table[addr_index].phy_address_entry] += 1;       // Physical Address
            page_address_table[prefetch_table[addr_index].page_address_entry] += 1;      // Page Address
            pc_depth_table[prefetch_table[addr_index].pc_depth_entry] += 1;          // PC XOR Depth
            pc_three_table[prefetch_table[addr_index].pc_three_entry] += 1;          // PC1 XOR (PC2>>1) XOR (PC3>>2)
            pc_delta_table[prefetch_table[addr_index].pc_delta_entry] += 1;          // PC XOR Delta
            page_confi_table[prefetch_table[addr_index].page_confi_entry] += 1;        // Page Address XOR Confidence
            sig_delta_table[prefetch_table[addr_index].sig_delta_entry] += 1;
            reject_table[addr_index].valid = false;
        }
    }
}

Addr SPP_PPF::calculate_prefetch_addr(Addr ppn, stride_t last_block,
    stride_t delta, double path_confidence, signature_t signature,
    bool is_secure, std::vector<AddrPriority> &addresses)
{
    if (delta == 0)
        return 0;
    
    stride_t block = last_block + delta;

    Addr pf_ppn;
    stride_t pf_block;
    if (block < 0) {
        stride_t num_cross_pages = 1 + (-block) / (pageBytes/blkSize);
        if (num_cross_pages > ppn) {
            // target address smaller than page 0, ignore this request;
            return 0;
        }
        pf_ppn = ppn - num_cross_pages;
        pf_block = block + (pageBytes/blkSize) * num_cross_pages;
        handlePageCrossingLookahead(signature, last_block, delta,
                                    path_confidence);
    } else if (block >= (pageBytes/blkSize)) {
        stride_t num_cross_pages = block / (pageBytes/blkSize);
        if (MaxAddr/pageBytes < (ppn + num_cross_pages)) {
            // target address goes beyond MaxAddr, ignore this request;
            return 0;
        }
        pf_ppn = ppn + num_cross_pages;
        pf_block = block - (pageBytes/blkSize) * num_cross_pages;
        handlePageCrossingLookahead(signature, last_block, delta,
                                    path_confidence);
    } else {
        pf_ppn = ppn;
        pf_block = block;
    }

    Addr new_addr = pf_ppn * pageBytes;
    new_addr += pf_block * (Addr)blkSize;

    DPRINTF(HWPrefetch, "Prefetch address: %#x.\n", new_addr);
    return new_addr;
}

} // namespace prefetch
} // namespace gem5
