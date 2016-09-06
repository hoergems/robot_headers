#ifndef __MANIPULATOR_SERIALIZER_HPP__
#define __MANIPULATOR_SERIALIZER_HPP__
#include <robot_headers/Serializer.hpp>

using std::cout;
using std::endl;

namespace frapu
{
class ManipulatorSerializer: public VectorSerializer
{
public:
    ManipulatorSerializer():
        VectorSerializer() {

    }
    
    virtual frapu::RobotStateSharedPtr loadState(std::istream &is) const override {
	
    }
    
    virtual frapu::RobotStateSharedPtr loadInitalState(std::ifstream &is) const {
	cout << "IN LOAD INITIAL STATE" << endl;
	std::vector<double> stateVec({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
	frapu::RobotStateSharedPtr robotState = std::make_shared<frapu::VectorState>(stateVec);	
	return robotState;
    }

};
}

#endif
