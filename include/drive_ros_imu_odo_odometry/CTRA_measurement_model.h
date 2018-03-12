#ifndef CTRA_MEASUREMENT_MODEL_H
#define CTRA_MEASUREMENT_MODEL_H

#include <kalman/LinearizedMeasurementModel.hpp>
#include "CTRA_system_model.h"

namespace CTRA {

/**
 * @brief Measurement vector measuring the acceleration in x- and y-direction and the turn rate
 *
 * @param T Numeric scalar type
 */
template<typename T>
class Measurement : public Kalman::Vector<T, 2>
{
public:
    KALMAN_VECTOR(Measurement, T, 2)

    //! acceleration in x-direction
    static constexpr size_t A = 0;

    //! turn rate around z-axis
    static constexpr size_t OMEGA = 1;


    T a()       const { return (*this)[ A ]; }
    T omega()    const { return (*this)[ OMEGA ]; }

    T& a()      { return (*this)[ A ]; }
    T& omega()   { return (*this)[ OMEGA ]; }
};

/**
 * @brief Measurement model
 *
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class MeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, Measurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef State<T> S;

    //! Measurement type shortcut definition
    typedef Measurement<T> M;

    /**
     * @brief Definition of (possibly non-linear) measurement function
     *
     * This function maps the system state to the measurement that is expected
     * to be received from the sensor assuming the system is currently in the
     * estimated state.
     *
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    M h(const S& x) const
    {
        M measurement;

        measurement.a()     = x.a();
        measurement.omega() = x.omega();

        return measurement;
    }

protected:
    void updateJacobians( const S& x )
    {
        this->H.setZero();

        this->H(M::A,     S::A)     = 1;
        this->H(M::OMEGA, S::OMEGA) = 1;
    }
};

} // namespace CTRA

#endif
