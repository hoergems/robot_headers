#ifndef __AUV_SERIALIZER_HPP__
#define __AUV_SERIALIZER_HPP__
#include <robot_headers/Serializer.hpp>

namespace frapu
{
class AUVSerializer: public VectorSerializer
{
public :
    AUVSerializer():
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
