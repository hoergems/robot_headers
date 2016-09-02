#ifndef __FRAPU__SERIALIZER__HPP__
#define __FRAPU__SERIALIZER__HPP__
#include <frapu_core/core.hpp>

namespace frapu
{

class Serializer
{
public:
    virtual frapu::RobotStateSharedPtr loadState(std::istream& is) const = 0;

    virtual frapu::ActionSharedPtr loadAction(std::istream& is) const = 0;

    virtual frapu::ObservationSharedPtr loadObservation(std::istream& is) const = 0;

    virtual void saveState(frapu::RobotStateSharedPtr& state, std::ostream& os) const {
        state->serialize(os);
    }

    virtual void saveAction(frapu::ActionSharedPtr& action, std::ostream& os) const {
        action->serialize(os);
    }

    virtual void saveObservation(frapu::ObservationSharedPtr& observation, std::ostream& os) const {
        observation->serialize(os);
    }
};

class VectorSerializer: public Serializer
{
public:
    VectorSerializer():
        Serializer() {

    }

    virtual frapu::RobotStateSharedPtr loadState(std::istream& is) const override {
        std::vector<std::string> strings;
        std::string s;
        is >> s;
        if (s == "NULL") {
            return nullptr;
        }
        while (s != "END") {
            strings.push_back(s);
            is >> s;
        }

        std::vector<double> stateValues;
        for (size_t i = 0; i < strings.size(); i++) {
            double val;
            std::istringstream(strings[i]) >> val;
            stateValues.push_back(val);
        }

        return std::make_shared<frapu::VectorState>(stateValues);
    }

    virtual frapu::ActionSharedPtr loadAction(std::istream& is) const override {
        std::vector<std::string> strings;
        std::string s;
        is >> s;
        if (s == "NULL") {
            return nullptr;
        }
        while (s != "END") {
            strings.push_back(s);
            is >> s;
        }

        std::vector<double> actionValues(strings.size());
        for (size_t i = 0; i < strings.size(); i++) {
            double val;
            std::istringstream(strings[i]) >> val;
            actionValues[i] = val;
        }
        
        return std::make_shared<frapu::VectorAction>(actionValues);               
    }
    
    virtual frapu::ObservationSharedPtr loadObservation(std::istream& is) const override {
	std::vector<std::string> strings;
        std::string s;
        is >> s;
        if (s == "NULL") {
            return nullptr;
        }
        while (s != "END") {
            strings.push_back(s);
            is >> s;
        }

        std::vector<double> observationValues(strings.size());
        for (size_t i = 0; i < strings.size(); i++) {
            double val;
            std::istringstream(strings[i]) >> val;
            observationValues[i] = val;
        }
        
        return std::make_shared<frapu::VectorObservation>(observationValues);
    }

};

}

#endif
