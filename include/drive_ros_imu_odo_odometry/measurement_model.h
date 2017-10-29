#ifndef CAR_TRACKER_FILTER_MEASUREMENT_MODEL_H
#define CAR_TRACKER_FILTER_MEASUREMENT_MODEL_H

#include <kalman/LinearizedMeasurementModel.hpp>
#include "system_model.h"

namespace CTRA {

/**
 * @brief Measurement vector measuring the acceleration in x- and y-direction and the turn rate
 *
 * @param T Numeric scalar type
 */
template<typename T>
class Measurement : public Kalman::Vector<T, 4>
{
public:
    KALMAN_VECTOR(Measurement, T, 4)

    //! acceleration in x-direction
    static constexpr size_t AX = 0;

    //! acceleration in y-direction
    static constexpr size_t AY = 1;

    //! velocity
    static constexpr size_t V = 2;

    //! turn rate around z-axis
    static constexpr size_t OMEGA = 3;


    T ax()       const { return (*this)[ AX ]; }
    T ay()       const { return (*this)[ AY ]; }
    T v()       const { return (*this)[ V ]; }
    T omega()    const { return (*this)[ OMEGA ]; }

    T& ax()      { return (*this)[ AX ]; }
    T& ay()      { return (*this)[ AY ]; }
    T& v()      { return (*this)[ V ]; }
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

        measurement.ax() = x.a();
        measurement.ay() = x.omega()*x.v(); //ay = omega^2*r = v^2/r  ==> r=v/omega
        measurement.v() = x.v();
        measurement.omega() = x.omega();

        return measurement;
    }

protected:
    void updateJacobians( const S& x )
    {
        this->H.setZero();

        this->H(M::AX, S::A) = 1;
        this->H(M::AY, S::V) = x.omega();
        this->H(M::AY, S::OMEGA) = x.v();
        this->H(M::V, S::V) = 1;
        this->H(M::OMEGA, S::OMEGA) = 1;
    }
};

} // namespace CTRA

#endif
