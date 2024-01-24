#include "mem/cache/replacement_policies/myclock_rp.hh"

#include <cassert>
#include <memory>

#include "params/MyCLOCKRP.hh"
#include "sim/cur_tick.hh"

namespace gem5
{

namespace replacement_policy
{

MyCLOCK::MyCLOCK(const Params &p)
  : Base(p)
{
}

void
MyCLOCK::invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
{
    // Set reference to false.
    std::static_pointer_cast<MyCLOCKReplData>(
        replacement_data)->reference = false;
}

void
MyCLOCK::touch(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    // Set reference to true.
    std::static_pointer_cast<MyCLOCKReplData>(
        replacement_data)->reference = true;
}

void
MyCLOCK::reset(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    // Set reference to true.
    std::static_pointer_cast<MyCLOCKReplData>(
        replacement_data)->reference = true;
}

ReplaceableEntry*
MyCLOCK::getVictim(const ReplacementCandidates& candidates) const
{
    // There must be at least one replacement candidate
    assert(candidates.size() > 0);

    // Find out the position of the ptr
    int ptr_pos = 0;
    for (const auto& candidate : candidates) {
        std::shared_ptr<MyCLOCKReplData> candidate_data =
            std::static_pointer_cast<MyCLOCKReplData>(candidate->replacementData);
        
        // Stop searching if the ptr_here is true.
        if (candidate_data->ptr_here == true)
        {
            candidate_data->ptr_here = false;
            break;
        }
        ptr_pos += 1;
    }
    // If fail to find the position of ptr (which means at the beginning of the simulation),
    // set ptr_pos = 0
    if (ptr_pos >= candidates.size())
        ptr_pos = 0;

    // Visit all candidates from ptr_pos
    ReplaceableEntry* victim = candidates[0];
    while (1) {
        std::shared_ptr<MyCLOCKReplData> candidate_replacement_data =
            std::static_pointer_cast<MyCLOCKReplData>(candidates[ptr_pos]->replacementData);

        // Stop searching entry if a cache line with reference=false is found.
        // For other entries met, set reference from true to false.
        if (candidate_replacement_data->reference == false) {
            victim = candidates[ptr_pos];
            ptr_pos = (ptr_pos + 1) % candidates.size();
            candidate_replacement_data =
            std::static_pointer_cast<MyCLOCKReplData>(candidates[ptr_pos]->replacementData);
            candidate_replacement_data->ptr_here = true;
            break;
        }
        else {
            candidate_replacement_data->reference = false;
            ptr_pos = (ptr_pos + 1) % candidates.size();
        }
    }

    return victim;
}

std::shared_ptr<ReplacementData>
MyCLOCK::instantiateEntry()
{
    return std::shared_ptr<ReplacementData>(new MyCLOCKReplData());
}

} // namespace replacement_policy
} // namespace gem5