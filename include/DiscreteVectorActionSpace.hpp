#ifndef __DISCRETE_VECTOR_ACTION_SPACE__
#define __DISCRETE_VECTOR_ACTION_SPACE__
#include "DiscreteActionSpace.hpp"

namespace frapu
{
class DiscreteVectorActionSpace: public DiscreteActionSpace
{
public:
    DiscreteVectorActionSpace(const ActionSpaceInfo &actionSpaceInfo);
    
    virtual ActionSharedPtr sampleUniform(std::default_random_engine* randGen) const override;
};
}

#endif
