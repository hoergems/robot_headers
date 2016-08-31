#ifndef __CONTINUOUS_VECTOR_ACTION_SPACE__
#define __CONTINUOUS_VECTOR_ACTION_SPACE__
#include "ContinuousActionSpace.hpp"

namespace frapu
{
class ContinuousVectorActionSpace: public ContinuousActionSpace
{
public:
    ContinuousVectorActionSpace(const ActionSpaceInfo &actionSpaceInfo);
    
    virtual ActionSharedPtr sampleUniform(std::default_random_engine* randGen) const override;
};
}

#endif
