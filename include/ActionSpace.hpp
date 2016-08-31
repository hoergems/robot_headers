#ifndef __ACT_SPACE_HPP__
#define __ACT_SPACE_HPP__
#include <string>
#include <vector>
#include <iostream>
#include <memory>
#include <frapu_core/core.hpp>
#include "Action.hpp"

using std::cout;
using std::endl;

namespace frapu
{

struct ActionSpaceInfo {
    ActionSpaceInfo() {}
    
    // 'discrete' or 'continuous'
    std::string type = "discrete";
    
    bool normalized = false;
};

class ActionLimits
{
public:
    ActionLimits() {

    }

    virtual bool enforceLimits(frapu::ActionSharedPtr& action) const = 0;

};

class VectorActionLimits: public ActionLimits
{
public:
    VectorActionLimits(std::vector<double>& lowerLimits,
                       std::vector<double>& upperLimits):
        ActionLimits(),
        lowerLimits_(lowerLimits),
        upperLimits_(upperLimits) {

    }

    virtual bool enforceLimits(frapu::ActionSharedPtr& action) const override {
        std::vector<double> actionVec = static_cast<frapu::VectorAction*>(action.get())->asVector();
        bool enforced = false;
        for (size_t i = 0; i < actionVec.size(); i++) {
            if (actionVec[i] < lowerLimits_[i]) {
                enforced = true;
                actionVec[i] = lowerLimits_[i];
            } else if (actionVec[i] > upperLimits_[i]) {
                enforced = true;
                actionVec[i] = upperLimits_[i];
            }
        }

        action = std::make_shared<frapu::VectorAction>(actionVec);
        return enforced;
    }

    virtual void getRawLimits(std::vector<double>& lowerLimits,
                              std::vector<double>& upperLimits) const {
        lowerLimits = lowerLimits_;
        upperLimits = upperLimits_;

    }

protected:
    std::vector<double>& lowerLimits_;
    std::vector<double>& upperLimits_;

};

// Normalization functor
struct normalize {
public:
    normalize():
        actionLimits_(nullptr) {

    }

    virtual void denormalizeAction(frapu::ActionSharedPtr& action) = 0;

    virtual void operator()(frapu::ActionSharedPtr& action) = 0;

    void setActionLimits(frapu::ActionLimitsSharedPtr& actionLimits) {
        actionLimits_ = actionLimits;
    }

protected:
    frapu::ActionLimitsSharedPtr actionLimits_;
};

struct standardNormalize: public normalize {
public:
    standardNormalize():
        normalize() {

    }

    virtual void denormalizeAction(frapu::ActionSharedPtr& action) override {
        std::vector<double> lowerActionLimits;
        std::vector<double> upperActionLimits;
        frapu::VectorActionLimits* vecA = static_cast<frapu::VectorActionLimits*>(actionLimits_.get());
        static_cast<frapu::VectorActionLimits*>(actionLimits_.get())->getRawLimits(lowerActionLimits, upperActionLimits);
        std::vector<double> actionVec = static_cast<frapu::VectorAction*>(action.get())->asVector();
        for (size_t i = 0; i < actionVec.size(); i++) {
            actionVec[i] = actionVec[i] * (upperActionLimits[i] - lowerActionLimits[i]) + lowerActionLimits[i];
        }

        action = std::make_shared<frapu::VectorAction>(actionVec);
    }

    virtual void operator()(frapu::ActionSharedPtr& action) override {
        std::vector<double> lowerActionLimits;
        std::vector<double> upperActionLimits;
        static_cast<frapu::VectorActionLimits*>(actionLimits_.get())->getRawLimits(lowerActionLimits, upperActionLimits);
        std::vector<double> actionVec = static_cast<frapu::VectorAction*>(action.get())->asVector();
        for (size_t i = 0; i < actionVec.size(); i++) {
            actionVec[i] = (actionVec[i] - lowerActionLimits[i]) / (upperActionLimits[i] - lowerActionLimits[i]);
        }

        action = std::make_shared<frapu::VectorAction>(actionVec);
    }
};

struct nullNormalize: public normalize {
public:
    nullNormalize():
        normalize() {

    }

    virtual void denormalizeAction(frapu::ActionSharedPtr& action) override {

    }

    virtual void operator()(frapu::ActionSharedPtr& action) override {

    }

};

class ActionSpace
{
public:
    ActionSpace(bool normalizedActionSpace);

    /**
     * Returns the type of the action space (discrete, continuous, hybrid)
     */
    virtual std::string getType() const = 0;

    virtual ActionSharedPtr sampleUniform(std::default_random_engine* randGen) const = 0;

    void setNumDimensions(unsigned int& numDimensions);

    void setActionLimits(frapu::ActionLimitsSharedPtr& actionLimits);

    frapu::ActionLimitsSharedPtr getActionLimits() const;

    unsigned int getNumDimensions() const;

    void normalizeAction(frapu::ActionSharedPtr& action);

    void denormalizeAction(frapu::ActionSharedPtr& action);

    bool isNormalized() const;

protected:
    unsigned int numDimensions_;

    frapu::ActionLimitsSharedPtr actionLimits_;

    bool normalizedActionSpace_;

    std::unique_ptr<normalize> actionNormalizer_;

};
}

#endif
