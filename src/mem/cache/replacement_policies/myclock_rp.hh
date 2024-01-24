#ifndef __MEM_CACHE_REPLACEMENT_POLICIES_MYCLOCK_RP_HH__
#define __MEM_CACHE_REPLACEMENT_POLICIES_MYCLOCK_RP_HH__

#include "base/types.hh"
#include "mem/cache/replacement_policies/base.hh"

namespace gem5
{

struct MyCLOCKRPParams;

namespace replacement_policy
{

class MyCLOCK : public Base
{
  protected:
    /** MyCLOCK-specific implementation of replacement data. */
    struct MyCLOCKReplData : ReplacementData
    {
        bool reference;               /* Reference bit. */
        bool ptr_here;                /* Indicating whether current pointer is here. */

        /**
         * Default constructor. Invalidate data.
         */
        MyCLOCKReplData() : reference(false), ptr_here(false) {}
    };

  public:
    typedef MyCLOCKRPParams Params;
    MyCLOCK(const Params &p);
    ~MyCLOCK() = default;

    /**
     * Invalidate replacement data to set it as the next probable victim.
     * Set reference to false.
     *
     * @param replacement_data Replacement data to be invalidated.
     */
    void invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
                                                                    override;

    /**
     * Touch an entry to update its replacement data.
     * Set reference to true.
     *
     * @param replacement_data Replacement data to be touched.
     */
    void touch(const std::shared_ptr<ReplacementData>& replacement_data) const
                                                                     override;

    /**
     * Reset replacement data. Used when an entry is inserted.
     * Set reference to true.
     *
     * @param replacement_data Replacement data to be reset.
     */
    void reset(const std::shared_ptr<ReplacementData>& replacement_data) const
                                                                     override;

    /**
     * Find replacement victim using access timestamps.
     *
     * @param cands Replacement candidates, selected by indexing policy.
     * @return Replacement entry to be replaced.
     */
    ReplaceableEntry* getVictim(const ReplacementCandidates& candidates) const
                                                                     override;

    /**
     * Instantiate a replacement data entry.
     *
     * @return A shared pointer to the new replacement data.
     */
    std::shared_ptr<ReplacementData> instantiateEntry() override;
};

} // namespace replacement_policy
} // namespace gem5

#endif // __MEM_CACHE_REPLACEMENT_POLICIES_MYMRU_RP_HH__