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

namespace shared
{

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

        state = std::make_shared<frapu::VectorAction>(stateVec);
        return enforced;
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

    virtual void denormalizeAction(std::vector<double>& a1,
                                   std::vector<double>& a2) = 0;

    virtual void operator()(std::vector<double>& a1,
                            std::vector<double>& a2) = 0;

    void setActionLimits(std::shared_ptr<ActionLimits> &actionLimits) {
        actionLimits_ = actionLimits;
    }

protected:
    std::shared_ptr<ActionLimits> actionLimits_;    
};

struct standardNormalize: public normalize {
public:
    standardNormalize():
        normalize() {

    }

    virtual void denormalizeAction(std::vector<double>& a1,
                                   std::vector<double>& a2) override {
        a2.clear();
        a2.resize(a1.size());
        for (size_t i = 0; i < lowerActionLimits_.size(); i++) {
            a2[i] = a1[i] * (upperActionLimits_[i] - lowerActionLimits_[i]) + lowerActionLimits_[i];
        }
    }

    virtual void operator()(std::vector<double>& a1,
                            std::vector<double>& a2) override {
        a2.clear();
        a2.resize(a1.size());
        for (size_t i = 0; i < lowerActionLimits_.size(); i++) {
            a2[i] = (a1[i] - lowerActionLimits_[i]) / (upperActionLimits_[i] - lowerActionLimits_[i]);
        }
    }
};

struct nullNormalize: public normalize {
public:
    nullNormalize():
        normalize() {

    }

    virtual void denormalizeAction(std::vector<double>& a1,
                                   std::vector<double>& a2) override {
        a2 = a1;
    }

    virtual void operator()(std::vector<double>& a1,
                            std::vector<double>& a2) override {
        a2 = a1;
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

    void setNumDimensions(unsigned int& numDimensions);

    void setActionLimits(std::shared_ptr<shared::ActionLimits> &actionLimits);

    std::shared_ptr<ActionLimits> getActionLimits() const;

    unsigned int getNumDimensions() const;

    void normalizeAction(std::vector<double>& action,
                         std::vector<double>& normalizedAction);

    void denormalizeAction(std::vector<double>& action,
                           std::vector<double>& normalizedAction);

    bool isNormalized() const;

protected:
    unsigned int numDimensions_;

    std::shared_ptr<ActionLimits> actionLimits_;

    bool normalizedActionSpace_;

    std::unique_ptr<shared::normalize> actionNormalizer_;

};
}

#endif
