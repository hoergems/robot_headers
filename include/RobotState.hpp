#ifndef ___ROBOT_STATE_HPP___
#define ___ROBOT_STATE_HPP___
#include <iosfwd>                       // for istream, ostream
#include <fstream>
#include <vector>
#include <memory>
#include <frapu_core/core.hpp>

namespace frapu
{
class RobotState
{
public:
    RobotState() {}

    virtual void serialize(std::ostream& os) const = 0;

    virtual void deSerialize(std::istream& is) = 0;

    virtual bool equals(frapu::RobotStateSharedPtr& otherState) const = 0;
    
    virtual std::size_t hash() const = 0;

};

class WeightedRobotState: public RobotState
{
public:
    WeightedRobotState():
        RobotState(),
        weight_(0.0) {

    }

    double getWeight() {
        return weight_;
    }

    void setWeight(const double& weight) {
        weight_ = weight;
    }

protected:
    mutable double weight_;
};


class VectorState: public WeightedRobotState
{
public:
    VectorState():
        WeightedRobotState(),
        state_() {

    }

    VectorState(std::vector<double>& stateVector):
        WeightedRobotState(),
        state_(stateVector) {

    }

    VectorState(const std::vector<double>& stateVector):
        WeightedRobotState(),
        state_(stateVector) {

    }

    virtual void serialize(std::ostream& os) const override {
        for (size_t i = 0; i < state_.size(); i++) {
            os << state_[i] << " ";
        }
        os << "END";
    }

    virtual void deSerialize(std::istream& is) override {
        std::vector<std::string> strings;
        std::string s;
        is >> s;
        /**if (s == "NULL") {
            return nullptr;
        }*/
        while (s != "END") {
            strings.push_back(s);
            is >> s;
        }

        std::vector<double> state_values;
        for (size_t i = 0; i < strings.size(); i++) {
            double val;
            std::istringstream(strings[i]) >> val;
            state_values.push_back(val);
        }

        state_ = state_values;
    }

    virtual bool equals(frapu::RobotStateSharedPtr& otherState) const override {
	VectorState *otherStatePtr = static_cast<VectorState *>(otherState.get());
        std::vector<double> otherStateVec = otherStatePtr->asVector();
        for (size_t i = 0; i < state_.size(); i++) {
            if (round_(state_[i]) != round_(otherStateVec[i])) {
		return false;
                /**if (otherState->getWeight() != weight_) {
                    return false;
                }*/
            }
        }
        return true;
    }
    
    virtual std::size_t hash() const override {
	
    }

    std::vector<double> asVector() const {
        return state_;
    }

protected:
    std::vector<double> state_;

    int roundingPrecision_ = 4;

    double round_(const double& value) const {
        double ceil_value = ceil(value * pow(10, roundingPrecision_)) / pow(10, roundingPrecision_);
        double floor_value = floor(value * pow(10, roundingPrecision_)) / pow(10, roundingPrecision_);
        if (fabs(value - floor_value) < fabs(ceil_value - value)) {
            return floor_value;
        }
        return ceil_value;


    }

};
}

#endif
