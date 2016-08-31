#ifndef __DISCRETE_ACTION_SPACE_HPP__
#define __DISCRETE_ACTION_SPACE_HPP__
#include "ActionSpace.hpp"

namespace frapu
{
class DiscreteActionSpace: public ActionSpace
{
public:
    DiscreteActionSpace(bool normalizedActionSpace);
    
    virtual std::string getType() const override;

};
}

#endif
