#ifndef UTILITY_POLICY_STATE_HPP
#define UTILITY_POLICY_STATE_HPP

#include <Eigen/Core>
#include <opengnc/common/math.hpp>

namespace utility {
namespace policy {

struct VehicleState {

    enum { state_vector_length = 16 };
    typedef double scalar_type;

    typedef Eigen::Matrix<scalar_type, state_vector_length, 1> XVector;
    typedef Eigen::Matrix<scalar_type, state_vector_length, state_vector_length> XMatrix;
    typedef Eigen::Matrix<scalar_type, 4, 3> Matrix4x3d;
    typedef Eigen::Matrix<double,3,4> Matrix3x4d;

    static Eigen::Vector3d rBNn(const XVector& x)
    {
        return  Eigen::Vector3d(x.segment<3>(0));
    }

    static Eigen::Vector4d thetanb(const XVector& x)
    {
        Eigen::Vector4d q = x.segment<4>(3);
        if (q.norm() == 0)
        {
            q << 1,0,0,0;
        }
        else
        {
            q = q / q.norm() ;
        }
        return std::move(q);
    }

    static Eigen::Vector3d vBNb(const XVector& x)
    {
        return Eigen::Vector3d(x.segment<3>(7));
    }

    static Eigen::Vector3d omegaBNb(const XVector& x)
    {
        return Eigen::Vector3d(x.segment<3>(10));
    }

    static Eigen::Vector3d gbBNi(const XVector& x)
    {
        return Eigen::Vector3d(x.segment<3>(13));
    }

    static Eigen::Matrix3d Rnb(const XVector& x)
    {
        return opengnc::common::math::rotationQuaternion(thetanb(x));
    }

    static Eigen::Matrix3d Rnm(const XVector& x)
    {
        Eigen::Matrix3d Rnm;
        Rnm = Eigen::AngleAxisd(thetanm(x), Eigen::Vector3d::UnitZ());

        return Rnm;
    }

    static int parameters_length() { return 3; }

	static void pack_state(XVector& x,
						   const Eigen::Vector3d& rBNn,
						   const Eigen::Vector4d& thetanb,
						   const Eigen::Vector3d& vBNb,
						   const Eigen::Vector3d& omegaBNb,
                           const Eigen::Vector3d& gbBNi)
    {
        x.segment<3>(0) = rBNn;
        x.segment<4>(3) = thetanb;
        x.segment<3>(7) = vBNb;
        x.segment<3>(10) = omegaBNb;
        x.segment<3>(13) = gbBNi;
    }

    static void apply_constraints(XVector& x)
    {
         x.segment(3,4) = thetanb(x);
         if (x(2) > -0.1) x(2) = -0.1;
    }
};

}
}

#endif // UTILITY_POLICY_STATE_HPP
