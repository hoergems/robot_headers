#ifndef __FRAPU_ACTION_HPP__
#define __FRAPU_ACTION_HPP__
#include <frapu_core/core.hpp>

namespace frapu
{
class Action
{
public:
    Action() {}

    virtual std::unique_ptr<Action> copy() const = 0;

    virtual void print(std::ostream& os) const = 0;

    virtual bool equals(const Action& otherAction) const = 0;

    virtual std::size_t hash() const = 0;

    virtual double distanceTo(const Action& otherAction) const = 0;

};

class VectorAction: public Action
{
public:
    VectorAction():
        Action(),
        actionVec_() {	   

    }

    VectorAction(std::vector<double>& actionValues):
        Action(),
        actionVec_(actionValues) {

    }

    VectorAction(const std::vector<double>& actionValues):
        Action(),
        actionVec_(actionValues) {

    }

    virtual std::unique_ptr<Action> copy() const override {
        std::unique_ptr<Action> copiedAction(new VectorAction(actionVec_));
        return copiedAction;
    }

    virtual void print(std::ostream& os) const override {
        for (auto & k : actionVec_) {
            os << k << " ";
        }
    }

    virtual bool equals(const Action& otherAction) const override {
        std::vector<double> otherActionVec = static_cast<const VectorAction&>(otherAction).asVector();
        for (size_t i = 0; i < otherActionVec.size(); i++) {
            if (actionVec_[i] != otherActionVec[i]) {
                return false;
            }
        }

        return true;
    }

    virtual std::size_t hash() const override {
        size_t hashValue = 0;
        for (auto & k : actionVec_) {
            frapu::hash_combine(hashValue, k);
        }

        return hashValue;
    }

    virtual double distanceTo(const Action& otherAction) const override {
        std::vector<double> otherActionVec = static_cast<const VectorAction&>(otherAction).asVector();
        double distance = 0.0;
        for (size_t i = 0; i < otherActionVec.size(); i++) {
            distance += std::pow(actionVec_[i] - otherActionVec[i], 2);
        }

        return std::sqrt(distance);
    }

    virtual std::vector<double> asVector() const {
        return actionVec_;
    }

protected:
    std::vector<double> actionVec_;

};

class BoundedVectorAction: public VectorAction
{
public:
    BoundedVectorAction(std::vector<double>& actionVec,
                        std::vector<double>& lowerLimits,
                        std::vector<double>& upperLimits):
        VectorAction(actionVec) {

    }
    
    virtual void getLimits(std::vector<double> &lowerLimits, std::vector<double> &upperLimits) const {
	lowerLimits = lowerLimits_;
	upperLimits = upperLimits_;
    }
    
protected:
    std::vector<double> lowerLimits_;
    std::vector<double> upperLimits_;

};

}

#endif
