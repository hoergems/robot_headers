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

    virtual void denormalizeAction(const ActionSharedPtr& action, ActionSharedPtr &denormalizedAction) = 0;

    virtual void operator()(const ActionSharedPtr& action, ActionSharedPtr &normalizedAction) = 0;

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

    virtual void denormalizeAction(const ActionSharedPtr& action, ActionSharedPtr &denormalizedAction) override {
        std::vector<double> lowerActionLimits;
        std::vector<double> upperActionLimits;
        frapu::VectorActionLimits* vecA = static_cast<frapu::VectorActionLimits*>(actionLimits_.get());
        static_cast<frapu::VectorActionLimits*>(actionLimits_.get())->getRawLimits(lowerActionLimits, upperActionLimits);
        std::vector<double> actionVec = static_cast<frapu::VectorAction*>(action.get())->asVector();
        for (size_t i = 0; i < actionVec.size(); i++) {
            actionVec[i] = actionVec[i] * (upperActionLimits[i] - lowerActionLimits[i]) + lowerActionLimits[i];
        }

        denormalizedAction = std::make_shared<frapu::VectorAction>(actionVec);
    }

    virtual void operator()(const ActionSharedPtr& action, ActionSharedPtr &normalizedAction) override {
        std::vector<double> lowerActionLimits;
        std::vector<double> upperActionLimits;
        static_cast<frapu::VectorActionLimits*>(actionLimits_.get())->getRawLimits(lowerActionLimits, upperActionLimits);
        std::vector<double> actionVec = static_cast<frapu::VectorAction*>(action.get())->asVector();
        for (size_t i = 0; i < actionVec.size(); i++) {
            actionVec[i] = (actionVec[i] - lowerActionLimits[i]) / (upperActionLimits[i] - lowerActionLimits[i]);
        }

        normalizedAction = std::make_shared<frapu::VectorAction>(actionVec);
    }
};

struct nullNormalize: public normalize {
public:
    nullNormalize():
        normalize() {

    }

    virtual void denormalizeAction(const ActionSharedPtr& action, ActionSharedPtr &denormalizedAction) override {
	denormalizedAction = action;
    }

    virtual void operator()(const ActionSharedPtr& action, ActionSharedPtr &normalizedAction) override {
	normalizedAction = action;
    }

};

class ActionSpace
{
public:
    ActionSpace(const ActionSpaceInfo &actionSpaceInfo);

    /**
     * Returns the type of the action space (discrete, continuous, hybrid)
     */
    virtual std::string getType() const = 0;

    virtual ActionSharedPtr sampleUniform(std::default_random_engine* randGen) const = 0;

    void setNumDimensions(unsigned int& numDimensions);

    void setActionLimits(frapu::ActionLimitsSharedPtr& actionLimits);

    frapu::ActionLimitsSharedPtr getActionLimits() const;

    unsigned int getNumDimensions() const;
    
    const ActionSpaceInfo getInfo() const;

    void normalizeAction(const ActionSharedPtr& action, ActionSharedPtr &normalizedAction);

    void denormalizeAction(const ActionSharedPtr& action, ActionSharedPtr &denormalizedAction);    

protected:
    unsigned int numDimensions_;

    frapu::ActionLimitsSharedPtr actionLimits_;

    ActionSpaceInfo actionSpaceInfo_;

    std::unique_ptr<normalize> actionNormalizer_;

};

class DiscreteActionSpace: public ActionSpace
{
public:
    DiscreteActionSpace(const ActionSpaceInfo &actionSpaceInfo);
    
    virtual std::string getType() const override;
    
    virtual std::vector<frapu::ActionSharedPtr> getAllActionsInOrder(unsigned int &numStepsPerDimension) const = 0;

};

class DiscreteVectorActionSpace: public DiscreteActionSpace
{
public:
    DiscreteVectorActionSpace(const ActionSpaceInfo &actionSpaceInfo);
    
    virtual ActionSharedPtr sampleUniform(std::default_random_engine* randGen) const override;
    
    virtual std::vector<frapu::ActionSharedPtr> getAllActionsInOrder(unsigned int &numStepsPerDimension) const override;
};

class ContinuousActionSpace: public ActionSpace
{
public:
    ContinuousActionSpace(const ActionSpaceInfo &actionSpaceInfo);
    
    virtual std::string getType() const override;

};

class ContinuousVectorActionSpace: public ContinuousActionSpace
{
public:
    ContinuousVectorActionSpace(const ActionSpaceInfo &actionSpaceInfo);
    
    virtual ActionSharedPtr sampleUniform(std::default_random_engine* randGen) const override;
};

}

#endif
