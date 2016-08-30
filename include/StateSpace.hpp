#ifndef __FRAPU_STATE_SPACE_HPP__
#define __FRAPU_STATE_SPACE_HPP__
#include <frapu_core/core.hpp>

namespace frapu
{
class StateSpace
{
public:
    StateSpace() {}

    virtual std::string getType() const = 0;

};

class VectorStateSpace: public StateSpace
{
public:
    VectorStateSpace(unsigned int& dimensions):
        dimensions_(dimensions) {

    }
    
    virtual std::string getType() const override {
	std::string type = "VectorStateSpace";
	return type;
    }

    unsigned int getDimensions() const {
        return dimensions_;
    }

protected:
    unsigned int dimensions_;


};

class BoundedVectorStateSpace: public VectorStateSpace
{
public:
    BoundedVectorStateSpace(std::vector<double>& lowerLimits,
                            std::vector<double>& upperLimits):
        VectorStateSpace(lowerLimits.size()),
        lowerLimits_(lowerLimits),
        upperLimits_(upperLimits) {

    }
    
    virtual std::string getType() const override {
	std::string type = "BoundedVectorStateSpace";
	return type;
    }

    void getLimits(std::vector<double>& lowerLimits,
                   std::vector<double>& upperLimits) const {
        lowerLimits = lowerLimits_;
        upperLimits = upperLimits_;
    }

protected:
    std::vector<double> lowerLimits_;

    std::vector<double> upperLimits_;

};

}

#endif
