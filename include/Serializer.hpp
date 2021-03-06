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

    virtual std::vector<frapu::RobotStateSharedPtr> loadGoalStatesFromFile(std::string &filename) const = 0;

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

    virtual std::vector<frapu::RobotStateSharedPtr> loadGoalStatesFromFile(std::string &filename) const override {
        std::vector<frapu::RobotStateSharedPtr> goalStates;
        std::ifstream file;
        try {
            file.open(filename);
        } catch (std::ios_base::failure& e) {
            std::cerr << e.what() << '\n';
            sleep(5);
        }

        double dub_val;
        std::string line;

        while (std::getline(file, line)) {
            std::istringstream sin(line);
            std::vector<double> stateVec;
            while (sin >> dub_val) {
                stateVec.push_back(dub_val);
            }
            
            frapu::RobotStateSharedPtr state = std::make_shared<frapu::VectorState>(stateVec);
            goalStates.push_back(state);
        }
        
        file.clear();
        file.seekg(0, file.beg);
        file.close();
        return goalStates;
    }

};

}

#endif
