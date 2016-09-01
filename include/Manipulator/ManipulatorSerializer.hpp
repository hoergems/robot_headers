#ifndef __MANIPULATOR_SERIALIZER_HPP__
#define __MANIPULATOR_SERIALIZER_HPP__
#include <robot_headers/Serializer.hpp>

using std::cout;
using std::endl;

namespace frapu
{
class ManipulatorSerializer: public Serializer
{
public:
    ManipulatorSerializer():
        Serializer() {

    }
    
    virtual frapu::RobotStateSharedPtr loadState(std::istream &is) const override {
	
    }
    
    virtual frapu::RobotStateSharedPtr loadInitalState(std::ifstream &is) const {
	cout << "IN LOAD INITIAL STATE" << endl;
	frapu::RobotStateSharedPtr robotState;
	return robotState;
    }

};
}

#endif
