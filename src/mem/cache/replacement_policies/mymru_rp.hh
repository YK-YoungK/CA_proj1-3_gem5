#ifndef __MEM_CACHE_REPLACEMENT_POLICIES_MYMRU_RP_HH__
#define __MEM_CACHE_REPLACEMENT_POLICIES_MYMRU_RP_HH__

#include "base/types.hh"
#include "mem/cache/replacement_policies/base.hh"

namespace gem5
{

struct MyMRURPParams;

namespace replacement_policy
{

class MyMRU : public Base
{
  protected:
    /** MyMRU-specific implementation of replacement data. */
    struct MyMRUReplData : ReplacementData
    {
        /** Tick on which the entry was last touched. */
        Tick lastTouchTick;

        /**
         * Default constructor. Invalidate data.
         */
        MyMRUReplData() : lastTouchTick(0) {}
    };

  public:
    typedef MyMRURPParams Params;
    MyMRU(const Params &p);
    ~MyMRU() = default;

    /**
     * Invalidate replacement data to set it as the next probable victim.
     * Sets its last touch tick as the starting tick.
     *
     * @param replacement_data Replacement data to be invalidated.
     */
    void invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
                                                                    override;

    /**
     * Touch an entry to update its replacement data.
     * Sets its last touch tick as the current tick.
     *
     * @param replacement_data Replacement data to be touched.
     */
    void touch(const std::shared_ptr<ReplacementData>& replacement_data) const
                                                                     override;

    /**
     * Reset replacement data. Used when an entry is inserted.
     * Sets its last touch tick as the current tick.
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