#ifndef __DUBIN_SERIALIZER_HPP__
#define __DUBIN_SERIALIZER_HPP__
#include <robot_headers/Serializer.hpp>

namespace frapu
{
class DubinSerializer: public VectorSerializer
{
public:
    DubinSerializer():
        VectorSerializer() {

    }
    
    frapu::RobotStateSharedPtr loadInitalState(std::ifstream &is) const {
	cout << "IN LOAD INITIAL STATE" << endl;
	std::vector<double> stateVec({-0.7, -0.7, 1.57, 0.0});
	frapu::RobotStateSharedPtr robotState = std::make_shared<frapu::VectorState>(stateVec);	
	return robotState;
    }
};
}

#endif
