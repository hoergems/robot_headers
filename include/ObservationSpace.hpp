#ifndef __OBSERVATION_SPACE_HPP_
#define __OBSERVATION_SPACE_HPP_
#include <vector>
#include <string>
#include "utils.hpp"
#include <frapu_core/core.hpp>

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
