#ifndef __CONTINUOUS_ACTION_SPACE_HPP__
#define __CONTINUOUS_ACTION_SPACE_HPP__
#include "ActionSpace.hpp"

namespace frapu
{
class ContinuousActionSpace: public ActionSpace
{
public:
    ContinuousActionSpace(const ActionSpaceInfo &actionSpaceInfo);
    
    virtual std::string getType() const override;

};
}

#endif