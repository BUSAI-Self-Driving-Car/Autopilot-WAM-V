#ifndef QPCONTROLALLOCATION_H
#define QPCONTROLALLOCATION_H

#include <memory>
#include <Eigen/Core>
#include <CGAL/basic.h>
#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>

#include <CGAL/MP_Float.h>


class QPControlAllocation
{
public:
    using Vector2s = Eigen::Vector2d;
    using Vector4s = Eigen::Vector4d;
    using Vector3s = Eigen::Vector3d;
    using Vector7s = Eigen::Matrix<double, 7, 1>;
    using Matrix2s = Eigen::Matrix2d;
    using Matrix3s = Eigen::Matrix3d;
    using Matrix3x2s = Eigen::Matrix<double, 3, 2>;
    using Matrix3x7s = Eigen::Matrix<double, 3, 7>;
    using Matrix7s = Eigen::Matrix<double, 7, 7>;

    struct ActuatorConfig
    {
        double M1x; // Motor 1 forward offset
        double M1y; // Motor 1 starboard offset
        double M2x; // Motor 2 forward offset
        double M2y; // Motor 2 starboard offset
    };

    struct ActuatorContraints
    {
        double Fmin;            // Minimum Force output of a single motor
        double Fmax;            // Maximum Force output of a single motor
        double alphaMin;        // Minimum angle of a single motor
        double alphaMax;        // Maximum angle of a single motor
        double DeltaFmin;       // Minimum discrete force slew rate per time step of a single motor
        double DeltaFmax;       // Maximum discrete force slew rate per time step of a single motor
        double DeltaAlphaMin;   // Minimum discrete angle slew rate per time step of a single motor
        double DeltaAlphaMax;   // Maximum discrete angle slew rate per time step of a single motor
    };

    typedef CGAL::MP_Float ET;


    //// program and solution types
    typedef CGAL::Quadratic_program_from_iterators
    <double**,                                                // for A
     double*,                                                 // for b
     CGAL::Const_oneset_iterator<CGAL::Comparison_result>, // for r
     bool*,                                                // for fl
     double*,                                                 // for l
     bool*,                                                // for fu
     double*,                                                 // for u
     double**,                                                // for D
     double*>                                                 // for c
    Program;
    // program and solution types
    typedef CGAL::Quadratic_program_solution<ET> Solution;

    QPControlAllocation();

    void init(Vector4s x0_,
              Matrix2s P_,
              Matrix3s Q_,
              Matrix2s Omega_,
              ActuatorConfig actuatorConfig,
              ActuatorContraints actuatorConstraints,
              double qpIterations_);

    Vector4s operator() (const Vector3s& tau_desired);
    inline bool initialised() const { return isInitialised; }
    Matrix3x2s B(const Vector2s &alpha);

protected:

    Matrix3x7s Aeq(const Vector7s& x);
    Vector7s gradient(const Vector7s& x);
    Vector7s lowerBound( const Vector7s& x);
    Vector7s upperBound(const Vector7s& x);

protected:
    bool isInitialised;
    bool isQPInitialised;
    Vector4s x0;
    Matrix7s H;
    Matrix2s P;
    Matrix3s Q;
    Matrix2s Omega;
    ActuatorConfig config;
    ActuatorContraints constraints;
    double qpIterations;
};

#endif // QPCONTROLALLOCATION_H
