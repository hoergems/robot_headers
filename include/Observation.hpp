#ifndef __FRAPU_OBSERVATION_HPP__
#define __FRAPU_OBSERVATION_HPP__
#include <frapu_core/core.hpp>

namespace frapu
{
class Observation
{
public:
    Observation() {}

    virtual frapu::ObservationUniquePtr copy() const = 0;

    virtual void serialize(std::ostream& os) const = 0;

    virtual void print(std::ostream& os) const = 0;
    
    virtual std::ostream& operator<< (std::ostream & out) const {
        print(out);
    }

    virtual bool equals(const Observation& otherObservation) const = 0;

    virtual std::size_t hash() const = 0;

    virtual double distanceTo(const Observation& otherObservation) const = 0;

};

class VectorObservation: public Observation
{
public:
    VectorObservation(std::vector<double>& observationVec):
        Observation(),
        observationVec_(observationVec) {

    }

    VectorObservation(const std::vector<double>& observationVec):
        Observation(),
        observationVec_(observationVec) {

    }

    virtual frapu::ObservationUniquePtr copy() const override {
        frapu::ObservationUniquePtr copiedObservation(new VectorObservation(observationVec_));
        return copiedObservation;
    }

    virtual void serialize(std::ostream& os) const override {
        for (auto & k : observationVec_) {
            os << k << " ";
        }

        os << "END";
    }


    virtual void print(std::ostream& os) const override {
        for (auto & k : observationVec_) {
            os << k << " ";
        }
    }

    virtual bool equals(const Observation& otherObservation) const override {
        std::vector<double> otherObservationVec =
            static_cast<const VectorObservation&>(otherObservation).asVector();
        for (size_t i = 0; i < otherObservationVec.size(); i++) {
            if (observationVec_[i] != otherObservationVec[i]) {
                return false;
            }
        }

        return true;
    }

    virtual std::size_t hash() const override {
        size_t hashValue = 0;
        for (auto & k : observationVec_) {
            frapu::hash_combine(hashValue, k);
        }

        return hashValue;
    }

    virtual double distanceTo(const Observation& otherObservation) const override {
        std::vector<double> otherObservationVec =
            static_cast<const VectorObservation&>(otherObservation).asVector();
        double distance = 0.0;
        for (size_t i = 0; i < otherObservationVec.size(); i++) {
            distance += std::pow(observationVec_[i] - otherObservationVec[i], 2);
        }

        return std::sqrt(distance);
    }

    virtual std::vector<double> asVector() const {
        return observationVec_;
    }

protected:
    std::vector<double> observationVec_;
};

}

#endif
