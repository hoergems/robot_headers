#ifndef ___ROBOT_STATE_HPP___
#define ___ROBOT_STATE_HPP___
#include <iosfwd>                       // for istream, ostream
#include <fstream>
#include <vector>

namespace robot
{
class RobotState
{
public:
    RobotState() {}

    virtual void serialize(std::ostream& os) const = 0;

    virtual void deSerialize(std::istream& is) const = 0;

};

class VectorState: public RobotState
{
public:
    VectorState():
        RobotState(),
        state_() {

    }

    VectorState(std::vector<double>& stateVector):
        RobotState(),
        state_(stateVector) {

    }

    virtual void serialize(std::ostream& os) const override {
        for (size_t i = 0; i < state_.size(); i++) {
            os << state_[i] << " ";
        }
        os << "END";
    }

    virtual void deSerialize(std::istream& is) const override {
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

    std::vector<double> asVector() {
        return state_;
    }

protected:
    std::vector<double> state_;

};
}
