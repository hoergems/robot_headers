#ifndef __DUBIN_SERIALIZER_HPP__
#define __DUBIN_SERIALIZER_HPP__
#include <robot_headers/Serializer.hpp>

namespace frapu
{
class DubinSerializer: public Serializer
{
public:
    DubinSerializer():
        Serializer() {

    }
    
    virtual frapu::RobotStateSharedPtr loadState(std::istream &is) const override {
	
    }
    
    frapu::RobotStateSharedPtr loadInitalState(std::ifstream &is) const {
	cout << "IN LOAD INITIAL STATE" << endl;
	frapu::RobotStateSharedPtr robotState;
	return robotState;
    }
};
}

#endif
