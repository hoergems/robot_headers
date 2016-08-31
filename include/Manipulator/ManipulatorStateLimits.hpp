#ifndef __MANIPULATOR_STATE_LIMITS__
#define __MANIPULATOR_STATE_LIMITS__
#include <robot_headers/StateSpace.hpp>

namespace shared
{
class ManipulatorStateLimits: public frapu::VectorStateLimits
{
public:
    ManipulatorStateLimits(std::vector< double >& lowerLimits,
                           std::vector< double >& upperLimits):
        frapu::VectorStateLimits(lowerLimits, upperLimits) {

    }

    bool enforceLimits(frapu::RobotStateSharedPtr& state) const override {
        std::vector<double> stateVec = static_cast<frapu::VectorState*>(state.get())->asVector();
        bool return_val = true;
        for (size_t i = 0; i < stateVec.size() / 2; i++) {
            if (stateVec[i] < lowerLimits_[i]) {
                stateVec[i] = lowerLimits_[i];
                stateVec[i + stateVec.size() / 2] = 0.0;
                return_val = false;
            } else if (stateVec[i] > upperLimits_[i]) {
                stateVec[i] = upperLimits_[i];
                stateVec[i + stateVec.size() / 2] = 0.0;
                return_val = false;
            }

            if (stateVec[i + stateVec.size() / 2] < lowerLimits_[i + stateVec.size() / 2]) {
                stateVec[i + stateVec.size() / 2] = lowerLimits_[i + stateVec.size() / 2];
                return_val = false;
            }

            else if (stateVec[i + stateVec.size() / 2] > upperLimits_[i + stateVec.size() / 2]) {
                stateVec[i + stateVec.size() / 2] = upperLimits_[i + stateVec.size() / 2];
                return_val = false;
            }
        }

        return return_val;
    }


};

}

#endif
