#ifndef __CPU_PRED_MYPERCEPTRON_PRED_HH__
#define __CPU_PRED_MYPERCEPTRON_PRED_HH__

#include "base/types.hh"
#include "cpu/pred/bpred_unit.hh"
#include "params/MyPerceptron.hh"

namespace gem5 {
namespace branch_prediction {

class MyPerceptron: public BPredUnit {
public:
    MyPerceptron(const MyPerceptronParams &params);
    void uncondBranch(ThreadID tid, Addr pc, void *&bp_history);
    bool lookup(ThreadID tid, Addr branch_addr, void *&bp_history);
    void btbUpdate(ThreadID tid, Addr branch_addr, void *&bp_history);
    void update(ThreadID tid, Addr branch_addr, bool taken, void *bp_history, bool squashed, const StaticInstPtr &inst, Addr corrTarget);
    void squash(ThreadID tid, void *bp_history);

private:
    void updateGlobalHistReg(ThreadID tid, bool taken);

    struct BPHistory
    {
        unsigned globalHistoryReg;
        
        // perceptron value
        int predValue;

        // the final taken/not-taken prediction
        // true: predict taken
        // false: predict not-taken
        bool finalPred;
    };

    std::vector<unsigned> globalHistoryReg;

    // perceptron size
    unsigned numPerceptrons;
    unsigned perceptronBits;

    // For compute entry
    unsigned globalHistoryBits;
    unsigned historyRegisterMask;

    // For select perceptron
    unsigned PCBits;
    unsigned PCMask;

    // preceptron predictors
    std::vector<std::vector<int> > preceptrons;

    unsigned preceptronThreshold;
};

}   // namespace branch_prediction
}   // namespace gem5

#endif      // __CPU_PRED_MYPERCEPTRON_PRED_HH__