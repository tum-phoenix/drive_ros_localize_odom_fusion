#ifndef CTRA_SYSTEM_MODEL_H
#define CTRA_SYSTEM_MODEL_H

#include <kalman/LinearizedSystemModel.hpp>

namespace CTRA {

/**
 * @brief System state vector-type for CTRA motion model
 *
 * @param T Numeric scalar type
 */
template<typename T>
class State : public Kalman::Vector<T, 6>
{
public:
    KALMAN_VECTOR(State, T, 6)

    //! X-position
    static constexpr size_t X = 0;
    //! Y-Position
    static constexpr size_t Y = 1;
    //! orientation
    static constexpr size_t THETA = 2;
    //! velocity
    static constexpr size_t V = 3;
    //! acceleration
    static constexpr size_t A = 4;
    //! Angular velocity
    static constexpr size_t OMEGA = 5;

    T x()       const { return (*this)[ X ]; }
    T y()       const { return (*this)[ Y ]; }
    T theta()   const { return (*this)[ THETA ]; }
    T v()      const { return (*this)[ V ]; }
    T a()      const { return (*this)[ A ]; }
    T omega()   const { return (*this)[ OMEGA ]; }

    T& x()      { return (*this)[ X ]; }
    T& y()      { return (*this)[ Y ]; }
    T& theta()  { return (*this)[ THETA ]; }
    T& v()     { return (*this)[ V ]; }
    T& a()     { return (*this)[ A ]; }
    T& omega()  { return (*this)[ OMEGA ]; }
};

/**
 * @brief System control-input vector-type for CTRA motion model
 *
 *
 * @param T Numeric scalar type
 */
template<typename T>
class Control : public Kalman::Vector<T, 2>
{
public:
    KALMAN_VECTOR(Control, T, 2)

    //! time since filter was last called
    static constexpr size_t DT = 0;
    //! velocity
    static constexpr size_t V = 1;


    T dt()      const { return (*this)[ DT ]; }
    T v()       const { return (*this)[ V ]; }


    T& dt()     { return (*this)[ DT ]; }
    T& v()      { return (*this)[ V ]; }
};

/**
 * @brief System model CTRA motion
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
     * @param [in] x The system state in current time-step
     * @param [in] u The control vector input
     * @returns The (predicted) system state in the next time-step
     */
    S f(const S& x, const C& u) const
    {
        //! Predicted state vector after transition
        S x_;

        auto th = x.theta();
        auto v = x.v();
        auto a = x.a();
        auto om = x.omega();
        auto dT = u.dt();
        auto v_new = u.v();

        auto cosTh = std::cos(th);
        auto sinTh = std::sin(th);

        if (std::abs(om) < T(0.01))
        {
            x_.x() = x.x() + T(0.5)*dT*(2*v+a*dT)*cosTh;
            x_.y() = x.y() + T(0.5)*dT*(2*v+a*dT)*sinTh;
        }
        else
        {
            auto cosThOmT = std::cos(th + om*dT);
            auto sinThOmT = std::sin(th + om*dT);

            x_.x() = x.x() + 1/(om*om)*((v*om+a*om*dT)*sinThOmT + a*cosThOmT - v*om*sinTh - a*cosTh);
            x_.y() = x.y() + 1/(om*om)*((-v*om-a*om*dT)*cosThOmT + a*sinThOmT + v*om*cosTh - a*sinTh);
        }

        x_.theta() = x.theta() + om*dT;
        x_.v()     = v_new + a*dT;
        x_.a()     = a;
        x_.omega() = om;

        // Return transitioned state vector
        return x_;
    }
protected:
    void updateJacobians( const S& x, const C& u )
    {
        this->F.setIdentity();

        auto th = x.theta();
        auto v = x.v();
        auto a = x.a();
        auto om = x.omega();
        auto dT = u.dt();

        auto cosTh = std::cos(th);
        auto sinTh = std::sin(th);

        if (std::abs(om) < T(0.01))
        {
            auto pTheta = T(0.5)*dT*(2*v+a*dT);
            auto pOmega = 1/T(6) * dT*dT * (3*v+2*a*dT);
            auto pA = T(0.5) * dT*dT;

            this->F( S::X, S::THETA ) = pTheta * -sinTh;
            this->F( S::X, S::V )     = dT * cosTh;
            this->F( S::Y, S::OMEGA ) = pOmega * -sinTh;
            this->F( S::X, S::A )     = pA * cosTh;

            this->F( S::Y, S::THETA ) = pTheta * cosTh;
            this->F( S::Y, S::V )     = dT * sinTh;
            this->F( S::Y, S::OMEGA ) = pOmega * cosTh;
            this->F( S::Y, S::A )     = pA * sinTh;
        }
        else
        {
            auto cosThOmT = std::cos(th + om*dT);
            auto sinThOmT = std::sin(th + om*dT);

            auto omSqrInv = 1/(om*om);

            this->F( S::X, S::THETA ) = omSqrInv * ( a*om*dT*cosThOmT + v*om*(cosThOmT-cosTh) - a*(sinThOmT-sinTh) );
            this->F( S::X, S::V )     = 1/om * ( sinThOmT - sinTh );
            this->F( S::X, S::OMEGA ) = omSqrInv * ( (a*dT+v)*(om*dT)*cosThOmT - (T(2)*a*dT+v)*sinThOmT + v*sinTh ) - T(2)*a*omSqrInv/om * (cosThOmT - cosTh);
            this->F( S::X, S::A )     = omSqrInv * ( om*dT*sinThOmT + cosThOmT - cosTh );

            this->F( S::Y, S::THETA ) = omSqrInv * ( a*om*dT*sinThOmT + v*om*(sinThOmT-sinTh) + a*(cosThOmT-cosTh) );
            this->F( S::Y, S::V )     = 1/om * ( - (cosThOmT - cosTh) );
            this->F( S::Y, S::OMEGA ) = omSqrInv * ( (a*dT+v)*(om*dT)*sinThOmT + (T(2)*a*dT+v)*cosThOmT - v*cosTh ) - T(2)*a*omSqrInv/om * (sinThOmT - sinTh);
            this->F( S::Y, S::A )     = omSqrInv * ( -om*dT*cosThOmT + sinThOmT - sinTh );
        }

        this->F( S::X, S::THETA ) = -std::sin(x.theta())*x.v()*u.dt();
        this->F( S::X, S::V )     = std::cos(x.theta()) * u.dt();
        this->F( S::X, S::OMEGA ) = std::cos(x.theta()) * u.dt();
        this->F( S::X, S::A )     = std::cos(x.theta()) * u.dt();

        this->F( S::Y, S::THETA ) = -std::cos(x.theta())*x.v()*u.dt();
        this->F( S::Y, S::V ) = std::sin(x.theta()) * u.dt();

        //this->F( S::THETA, S::THETA ) = 0;
        this->F( S::THETA, S::OMEGA ) = u.dt();

        this->F( S::V, S::V ) = 0;
        this->F( S::V, S::A ) = u.dt();

    }
};

} // namespace CTRA

#endif
