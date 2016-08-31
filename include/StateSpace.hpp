#ifndef __FRAPU_STATE_SPACE_HPP__
#define __FRAPU_STATE_SPACE_HPP__
#include <frapu_core/core.hpp>
#include "RobotState.hpp"

namespace frapu
{

class StateLimits
{
public:
    StateLimits() {

    }

    virtual bool enforceLimits(frapu::RobotStateSharedPtr& state) const = 0;

};

class VectorStateLimits: public StateLimits
{
public:
    VectorStateLimits(std::vector<double>& lowerLimits,
                      std::vector<double>& upperLimits):
        lowerLimits_(lowerLimits),
        upperLimits_(upperLimits) {

    }

    virtual bool enforceLimits(frapu::RobotStateSharedPtr& state) const override {
        std::vector<double> stateVec = static_cast<frapu::VectorState*>(state.get())->asVector();
        bool enforced = false;
        for (size_t i = 0; i < stateVec.size(); i++) {
            if (stateVec[i] < lowerLimits_[i]) {
                enforced = true;
                stateVec[i] = lowerLimits_[i];
            } else if (stateVec[i] > upperLimits_[i]) {
                enforced = true;
                stateVec[i] = upperLimits_[i];
            }
        }

        state = std::make_shared<frapu::VectorState>(stateVec);
        return enforced;
    }

    virtual void getVectorLimits(std::vector<double>& lowerLimits,
                                 std::vector<double>& upperLimits) const {
        lowerLimits = lowerLimits_;
        upperLimits = upperLimits_;
    }

protected:
    std::vector<double> lowerLimits_;
    std::vector<double> upperLimits_;

};


class StateSpace
{
public:
    StateSpace():
        stateLimits_(nullptr) {

    }

    virtual std::string getType() const = 0;

    void setStateLimits(std::shared_ptr<StateLimits>& stateLimits) {
        stateLimits_ = stateLimits;
    }

    bool enforceStateLimits(frapu::RobotStateSharedPtr& state) const {
        if (!stateLimits_) {
            cout << "Warning: StateSpace: Trying to enforce state limits, but no limits set." << endl;
            return false;

        } else {
            return stateLimits_->enforceLimits(state);
        }
    }

    std::shared_ptr<StateLimits> getStateLimits() const {
        return stateLimits_;
    }

protected:
    std::shared_ptr<StateLimits> stateLimits_;

};

class VectorStateSpace: public StateSpace
{
public:
    VectorStateSpace(unsigned int dimensions):
        StateSpace(),
        dimensions_(dimensions) {

    }

    virtual std::string getType() const override {
        std::string type = "VectorStateSpace";
        return type;
    }

    unsigned int getDimensions() const {
        return dimensions_;
    }

protected:
    unsigned int dimensions_;


};

}

#endif
