#ifndef CTRV_SYSTEM_MODEL_H
#define CTRV_SYSTEM_MODEL_H

#include <kalman/LinearizedSystemModel.hpp>

namespace CTRV {

/**
 * @brief System state vector-type for CTRV motion model
 *
 * @param T Numeric scalar type
 */
template<typename T>
class State : public Kalman::Vector<T, 3>
{
public:
    KALMAN_VECTOR(State, T, 3)

    //! X-position
    static constexpr size_t X = 0;
    //! Y-Position
    static constexpr size_t Y = 1;
    //! orientation
    static constexpr size_t THETA = 2;

    T x()       const { return (*this)[ X ]; }
    T y()       const { return (*this)[ Y ]; }
    T theta()   const { return (*this)[ THETA ]; }

    T& x()      { return (*this)[ X ]; }
    T& y()      { return (*this)[ Y ]; }
    T& theta()  { return (*this)[ THETA ]; }
};

/**
 * @brief System control-input vector-type for CTRV motion model
 *
 *
 * @param T Numeric scalar type
 */
template<typename T>
class Control : public Kalman::Vector<T, 3>
{
public:
    KALMAN_VECTOR(Control, T, 3)

    static constexpr size_t DT      = 0;
    static constexpr size_t V       = 1;
    static constexpr size_t OMEGA   = 2;

    T dt()      const { return (*this)[ DT ]; }
    T v()       const { return (*this)[ V ]; }
    T om()      const { return (*this)[ OMEGA ]; }

    T& dt()     { return (*this)[ DT ]; }
    T& v()      { return (*this)[ V ]; }
    T& om()     { return (*this)[ OMEGA ]; }
};

/**
 * @brief System model CTRV motion
 *
 * This is the system model defining how our car moves from one
 * time-step to the next, i.e. how the system state evolves over time.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef State<T> S;

    //! Control type shortcut definition
    typedef Control<T> C;

    /**
     * @brief Definition of (non-linear) state transition function
     *
     * This function defines how the system state is propagated through time,
     * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to
     * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
     * the system control input \f$u\f$.
     *
     * @param [in] s The system state in current time-step
     * @param [in] u The control vector input
     * @returns The (predicted) system state in the next time-step
     */
    S f(const S& s, const C& u) const
    {
        //! Predicted state vector after transition
        S s_;

        auto x = s.x();
        auto y = s.y();
        auto theta = s.theta();
        auto v = u.v();
        auto omega = u.om();
        auto t = u.dt();

        // simple model
        if (std::abs(omega) < T(0.01))
        {

          /* Matlab generated code (check the docs) for symbolic expression: f */
          s_(0) = x+t*v*cos(theta);
          s_(1) = y+t*v*sin(theta);
          s_(2) = theta;

        // standard model
        }else{

            /* Matlab generated code (check the docs) for symbolic expression: fLimit */
            auto t2 = 1.0/omega;
            auto t3 = omega*t;
            auto t4 = t3+theta;
            s_(0) = x+t2*v*(sin(t4)-sin(theta));
            s_(1) = y-t2*v*(cos(t4)-cos(theta));
            s_(2) = t4;

        }

        // Return transitioned state vector
        return s_;
    }
protected:
    void updateJacobians( const S& s, const C& u )
    {
        this->F.setIdentity();

        auto theta = s.theta();
        auto v = u.v();
        auto omega = u.om();
        auto t = u.dt();

        // simple model
        if (std::abs(omega) < T(0.01))
        {

          /*
          Matlab generated code (check the docs) for symbolic expression: FLimit
          */
          this->F(0,0) = 1.0;
          this->F(0,2) = -t*v*sin(theta);
          this->F(1,1) = 1.0;
          this->F(1,2) = t*v*cos(theta);
          this->F(2,2) = 1.0;

        // standard model
        }else{

          /*
          Matlab generated code (check the docs) for symbolic expression: F
          */
          auto t2 = 1.0/omega;
          auto t3 = omega*t;
          auto t4 = t3+theta;
          this->F(0,0) = 1.0;
          this->F(0,2) = t2*v*(cos(t4)-cos(theta));
          this->F(1,1) = 1.0;
          this->F(1,2) = t2*v*(sin(t4)-sin(theta));
          this->F(2,2) = 1.0;

        }
    }
};

} // namespace CTRV

#endif
