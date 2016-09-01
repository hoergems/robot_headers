#ifndef __FRAPU__SERIALIZER__HPP__
#define __FRAPU__SERIALIZER__HPP__
#include <frapu_core/core.hpp>

namespace frapu
{
    
class Serializer
{
public: 
    virtual ~Serializer() = 0;
    
    virtual frapu::RobotStateSharedPtr loadState(std::istream &is) const = 0;
    
    virtual void saveState(frapu::RobotStateSharedPtr &state, std::ostream &os) const {
	state->serialize(os);
    }

};

}

#endif
