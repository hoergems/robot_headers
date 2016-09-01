#ifndef __FRAPU_BELIEF_HPP__
#define __FRAPU_BELIEF_HPP__
#include <frapu_core/core.hpp>

namespace frapu
{

class Particle
{
public:
    Particle(frapu::RobotStateSharedPtr& state, double weight):
        state_(state),
        weight_(weight){

    }
    
    double getWeight() const {
	return weight_;
    }  
    
    frapu::RobotStateSharedPtr getState() const {
	return state_;
    }    

protected:
    frapu::RobotStateSharedPtr state_;
    
    double weight_;

};



class Belief
{
public:
    Belief() {

    }

};

class ParticleBelief: public Belief
{
public:
    ParticleBelief(std::vector<frapu::ParticleSharedPtr>& particles):
        Belief(),
        particles_(particles) {

    }

    virtual void getParticles(std::vector<frapu::ParticleSharedPtr>& particles) const {
        return particles_;
    }

protected:
    std::vector<frapu::ParticleSharedPtr> particles_;

};

}

#endif
