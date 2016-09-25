#ifndef __PARTIAL_DERIVATIVE_HPP__
#define __PARTIAL_DERIVATIVE_HPP__
#include <frapu_core/core.hpp>

using std::cout;
using std::endl;

namespace frapu
{

/**
 * Functor for a single partial derivative (one column of the jacobian matrix of f)
 * This functor operates directly on a single column with columnIndex
 * of the Jacobian matrix.
 * The valueIndex variable determines the variable the derivative is taken with respect to
 */
template<typename F>
struct PartialDerivative {
    PartialDerivative(F f,
                      std::shared_ptr<Eigen::MatrixXd>& jacobian,
                      size_t columnIndex,
                      size_t valueIndex):
        f_(f),
        columnIndex_(columnIndex),
        jacobian_(jacobian),
        valueIndex_(valueIndex) {}

    /*
     * Uses the symetric difference quotient for numerical differentiation of function f
     * with respect to variable with variableIndex. The result is a column vector of the
     * Jacobian matrix.
     */
    void operator()(double* x) {
        // This epsilon has to be choosed problem dependent
        static const double eps = 1e-6;

        double old_xi = x[valueIndex_];
        x[valueIndex_] = old_xi + eps;
        Eigen::VectorXd resPlus(jacobian_->rows());

        // Evalue f at x + eps
        f_(x, resPlus);

        volatile double actual_2eps = x[valueIndex_];
        x[valueIndex_] = old_xi - eps;
        actual_2eps -= x[valueIndex_];
        Eigen::VectorXd resMinus(jacobian_->rows());

        // Evaluate f at x - eps
        f_(x, resMinus);
        double eps2 = actual_2eps;
        Eigen::VectorXd result = (resPlus - resMinus) / eps2;
        jacobian_->col(columnIndex_) = result;
        x[valueIndex_] = old_xi;
    }

private:
    F f_;
    std::shared_ptr<Eigen::MatrixXd> jacobian_;
    size_t columnIndex_;
    size_t valueIndex_;
};

/**
 * Returns a PartialDerivative functor for the Jacobian matrix column with columnIndex
 */
template<typename F>
PartialDerivative<F> partial(F f,
                             std::shared_ptr<Eigen::MatrixXd>& jacobian,
                             size_t columnIndex,
                             size_t valueIndex)
{
    return PartialDerivative<F>(f, jacobian, columnIndex, valueIndex);
}

/**
 * Struct that assigns a PartialDerivative functor to each column of the Jacobian matrix
 */
template<typename F>
struct MixedDerivatives {
    MixedDerivatives(F f,
                     std::shared_ptr<Eigen::MatrixXd>& jacobian,
                     size_t firstValueIndex):
        columnFunctions_() {

        // Every column of the Jacobian matrix is the result of a PartialDerivative function
        // Here we define these function for every column
        for (size_t i = 0; i < jacobian->cols(); i++) {
            std::function<void(double*)> columnFunction =
                partial(f, jacobian, i, firstValueIndex + i);
            columnFunctions_.push_back(columnFunction);
        }
    }

    void operator()(double* vals) {
        for (auto & columnFunction : columnFunctions_) {
            columnFunction(vals);
        }
    }

private:
    std::vector<std::function<void(double*)>> columnFunctions_;
};

/*
 * Returns a MixedDerivative functor
 */
template<typename F>
MixedDerivatives<F> mixedDerivatives(F f,
                                     std::shared_ptr<Eigen::MatrixXd>& jacobian,
                                     size_t firstValueIndex)
{
    return MixedDerivatives<F>(f, jacobian, firstValueIndex);
}

}

#endif

