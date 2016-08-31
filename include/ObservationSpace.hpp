#ifndef __OBSERVATION_SPACE_HPP_
#define __OBSERVATION_SPACE_HPP_
#include <vector>
#include <string>
#include "utils.hpp"
#include <frapu_core/core.hpp>
#include "Observation.hpp"

namespace frapu
{

struct ObservationSpaceInfo {
public:
    ObservationSpaceInfo() {}

    // The observation type ('discrete' or 'continuous')
    std::string observationType;

    // Contains additional information (e.g. 'linear' or 'nonlinear')
    std::string observationModelInfo;
};

class ObservationLimits
{
public:
    ObservationLimits() {

    }

    virtual bool enforceLimits(frapu::ObservationSharedPtr& observation) const = 0;

};

class VectorObservationLimits: public ObservationLimits
{
public:
    VectorObservationLimits(std::vector<double>& lowerLimits,
                            std::vector<double>& upperLimits):
        ObservationLimits(),
        lowerLimits_(lowerLimits),
        upperLimits_(upperLimits) {

    }

    virtual bool enforceLimits(frapu::ObservationSharedPtr& observation) const override {
        std::vector<double> observationVec =
            static_cast<frapu::VectorObservation*>(observation.get())->asVector();
        bool enforced = false;
        for (size_t i = 0; i < observationVec.size(); i++) {
            if (observationVec[i] < lowerLimits_[i]) {
                enforced = true;
                observationVec[i] = lowerLimits_[i];
            } else if (observationVec[i] > upperLimits_[i]) {
                enforced = true;
                observationVec[i] = upperLimits_[i];
            }
        }

        observation = std::make_shared<frapu::VectorObservation>(observationVec);
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

class ObservationSpace
{
public:
    ObservationSpace(const ObservationSpaceInfo& observationSpaceInfo);

    void setDimension(unsigned int dimension);

    unsigned int getDimension() const;

    const ObservationSpaceInfo getObservationSpaceInfo() const;

private:
    unsigned int dimension_;

    ObservationSpaceInfo observationSpaceInfo_;
};

}

#endif
