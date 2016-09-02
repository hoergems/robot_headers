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
	frapu::RobotStateSharedPtr robotState;
	return robotState;
    }
};
}

#endif
