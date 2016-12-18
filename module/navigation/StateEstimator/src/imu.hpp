#ifndef IMU_HPP
#define IMU_HPP

#include <Eigen/Core>

template <typename state_policy>
class imu {
public:
    using Vector3s = Eigen::Matrix<typename state_policy::scalar_type, 3, 1>;
    using Matrix3s = Eigen::Matrix<typename state_policy::scalar_type, 3, 3>;

    enum { input_length = state_policy::state_vector_length, output_length = 9 };
    using output_scalar_type = double;
    using input_scalar_type = typename state_policy::scalar_type;
    using YVector =  Eigen::Matrix<input_scalar_type, output_length, 1>;
    using XVector = Eigen::Matrix<input_scalar_type, input_length, 1>;

    imu()
        : mag_scale(1)
        , mag_vector(1, 0, 0)
        , Rib(Matrix3s::Identity())
        , Rnmag(Matrix3s::Identity())
        , rIBb(0, 0, 0)
        , gn(0, 0, 9.8)
    { }

    void set_mag_scale(double value) { mag_scale = value; }
    void set_mag_vector(const Vector3s& value) { mag_vector = value; }
    void set_Rib(const Matrix3s& value) { Rib = value; }
    void set_Rnmag(const Matrix3s& value) { Rnmag = value; }
    void set_rIBb(const Vector3s& value) { rIBb = value; }
    void set_gn(const Vector3s& value) { gn = value; }

    YVector operator() (const XVector& x) const
    {
        using namespace opengnc::common;

        const auto Rnb = math::rotationQuaternion(state_policy::thetanb(x));
        const auto omegaBNb = state_policy::omegaBNb(x);
        const auto gbBNi = state_policy::gbBNi(x);

        Vector3s acc =  Rib*(math::Skew(omegaBNb)*rIBb - Rnb.transpose()*gn);
        Vector3s gyro = Rib*(omegaBNb) + gbBNi;
        Vector3s mag =  Rib*Rnb.transpose()*(mag_scale*Rnmag*mag_vector);
        mag[2]=0;

        YVector y;
        y << acc,
             gyro,
             mag;

        return y;
    }


protected:
    double  mag_scale;
    Vector3s mag_vector;
    Matrix3s Rib;
    Matrix3s Rnmag;
    Vector3s rIBb;
    Vector3s gn;

};

#endif // IMU_HPP
