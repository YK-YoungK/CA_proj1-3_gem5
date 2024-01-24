#include "cpu/pred/myperceptron.hh"

namespace gem5 {
namespace branch_prediction {

MyPerceptron::MyPerceptron(const MyPerceptronParams &params): BPredUnit(params),
    globalHistoryReg(params.numThreads, 0),
    numPerceptrons(params.numPerceptrons),
    perceptronBits(params.perceptronBits),
    globalHistoryBits(params.perceptronBits - 1),
    PCBits(ceilLog2(params.numPerceptrons)),
    preceptrons(params.numPerceptrons, std::vector<int>(params.perceptronBits, 0)),
    preceptronThreshold(params.perceptronThreshold)
{
    if (!isPowerOf2(numPerceptrons))
        fatal("Invalid preceptron size.\n");
    historyRegisterMask = mask(globalHistoryBits);
    PCMask = mask(PCBits);
}

bool MyPerceptron::lookup(ThreadID tid, Addr branch_addr, void *&bp_history) {
    /*unsigned perceptronIdx = (((branch_addr >> instShiftAmt)
                                ^ globalHistoryReg[tid])
                                & PCMask);*/
    unsigned perceptronIdx = ((branch_addr >> instShiftAmt) & PCMask);
    assert(perceptronIdx < numPerceptrons);

    std::vector<int> finalperceptron = preceptrons[perceptronIdx];
    int prediction = finalperceptron[0];
    for (int i = 1; i < perceptronBits; ++i)
    {
        if ((globalHistoryReg[tid] >> i) & 1)
            prediction += finalperceptron[i];
        else
            prediction -= finalperceptron[i];
    }
    bool finalPrediction = (prediction >= 0);

    BPHistory *history = new BPHistory;
    history->globalHistoryReg = globalHistoryReg[tid];
    history->predValue = prediction;
    history->finalPred = finalPrediction;
    bp_history = static_cast<void*>(history);
    updateGlobalHistReg(tid, finalPrediction);
    
    return finalPrediction;
}


// For an unconditional branch we set its history such that everything is set to taken.
void MyPerceptron::uncondBranch(ThreadID tid, Addr pc, void *&bp_history) {
    BPHistory *history = new BPHistory;
    history->globalHistoryReg = globalHistoryReg[tid];
    history->finalPred = true;
    bp_history = static_cast<void*>(history);
    updateGlobalHistReg(tid, true);
}

void MyPerceptron::btbUpdate(ThreadID tid, Addr branch_addr, void *&bp_history) {
    globalHistoryReg[tid] &= (historyRegisterMask & ~1ULL);
}

void MyPerceptron::update(ThreadID tid, Addr branch_addr, bool taken, void *bp_history, bool squashed, const StaticInstPtr &inst, Addr corrTarget) {
    assert(bp_history);
    BPHistory *history = static_cast<BPHistory*>(bp_history);

    // We do not update the counters speculatively on a squash.
    // We just restore the global history register.
    if (squashed) {
        globalHistoryReg[tid] = (history->globalHistoryReg << 1) | taken;
        return;
    }

    /*unsigned perceptronIdx = (((branch_addr >> instShiftAmt)
                                ^ globalHistoryReg[tid])
                                & PCMask);*/
    unsigned perceptronIdx = ((branch_addr >> instShiftAmt) & PCMask);
    assert(perceptronIdx < numPerceptrons);

    int abspred = history->predValue >= 0 ? history->predValue : -history->predValue;

    if (history->finalPred != taken || abspred <= preceptronThreshold)
    {
        if (taken)
        {
            preceptrons[perceptronIdx][0] += 1;
            for (int i = 1; i < perceptronBits; ++i)
            {
                if (((history->globalHistoryReg) >> i) & 1)
                    preceptrons[perceptronIdx][i] += 1;
                else
                    preceptrons[perceptronIdx][i] -= 1;
            }
        }
        else
        {
            preceptrons[perceptronIdx][0] -= 1;
            for (int i = 1; i < perceptronBits; ++i)
            {
                if (((history->globalHistoryReg) >> i) & 1)
                    preceptrons[perceptronIdx][i] -= 1;
                else
                    preceptrons[perceptronIdx][i] += 1;
            }
        }
    }

    delete history;
}

void MyPerceptron::squash(ThreadID tid, void *bp_history) {
    BPHistory *history = static_cast<BPHistory*>(bp_history);
    globalHistoryReg[tid] = history->globalHistoryReg;

    delete history;
}

void MyPerceptron::updateGlobalHistReg(ThreadID tid, bool taken) {
    globalHistoryReg[tid] = taken ? (globalHistoryReg[tid] << 1) | 1 :
                               (globalHistoryReg[tid] << 1);
    globalHistoryReg[tid] &= historyRegisterMask;
}


}   // namespace branch_prediction
}   // namespace gem5