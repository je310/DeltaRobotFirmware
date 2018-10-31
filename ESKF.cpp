#include "ESKF.h"
#include "unrolledFPFt.h"

#define SQ(x) (x*x)
#define I_3 (Eigen::Matrix3f::Identity())
#define I_dx (Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE>::Identity())

using namespace Eigen;
using namespace std;

ESKF::ESKF(Eigen::Vector3f a_gravity,
        const Eigen::Matrix<float, STATE_SIZE, 1>& initialState,
        const Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE>& initalP,
        float var_acc, float var_omega, float var_acc_bias, float var_omega_bias)
        : var_acc_(var_acc),
        var_omega_(var_omega),
        var_acc_bias_(var_acc_bias),
        var_omega_bias_(var_omega_bias),
        a_gravity_(a_gravity),
        nominalState_(initialState),
        P_(initalP) {
    
    // Jacobian of the state transition: page 59, eqn 269
    // Precompute constant part only
    F_x_.setZero();
    // dPos row
    F_x_.block<3, 3>(dPOS_IDX, dPOS_IDX) = I_3;
    // dVel row
    F_x_.block<3, 3>(dVEL_IDX, dVEL_IDX) = I_3;
    // dTheta row
    // dAccelBias row
    F_x_.block<3, 3>(dAB_IDX, dAB_IDX) = I_3;
    // dGyroBias row
    F_x_.block<3, 3>(dGB_IDX, dGB_IDX) = I_3;
}


Matrix<float, STATE_SIZE, 1> ESKF::makeState(
            const Vector3f& p,
            const Vector3f& v,
            const Quaternionf& q,
            const Vector3f& a_b,
            const Vector3f& omega_b) {
    Matrix<float, STATE_SIZE, 1> out;
    out << p, v, quatToHamilton(q).normalized(), a_b, omega_b;
    return out;
}

Matrix<float, dSTATE_SIZE, dSTATE_SIZE> ESKF::makeP(
        const Matrix3f& cov_pos,
        const Matrix3f& cov_vel,
        const Matrix3f& cov_dtheta,
        const Matrix3f& cov_a_b,
        const Matrix3f& cov_omega_b) {
    Matrix<float, dSTATE_SIZE, dSTATE_SIZE> P;
    P.setZero();
    P.block<3, 3>(dPOS_IDX, dPOS_IDX) = cov_pos;
    P.block<3, 3>(dVEL_IDX, dVEL_IDX) = cov_vel;
    P.block<3, 3>(dTHETA_IDX, dTHETA_IDX) = cov_dtheta;
    P.block<3, 3>(dAB_IDX, dAB_IDX) = cov_a_b;
    P.block<3, 3>(dGB_IDX, dGB_IDX) = cov_omega_b;
    return P;
}

Matrix3f ESKF::getDCM() {
    return getQuat().matrix();
}

Quaternionf ESKF::quatFromHamilton(const Vector4f& qHam) {
    return Quaternionf(
        (Vector4f() <<
            qHam.block<3, 1>(1, 0), // x, y, z
            qHam.block<1, 1>(0, 0) // w
        ).finished());
}

Vector4f ESKF::quatToHamilton(const Quaternionf& q){
    return (Vector4f() <<
            q.coeffs().block<1, 1>(3, 0), // w
            q.coeffs().block<3, 1>(0, 0) // x, y, z
        ).finished();
}

Matrix3f ESKF::getSkew(const Vector3f& in) {
    Matrix3f out;
    out << 0, -in(2), in(1),
        in(2), 0, -in(0),
        -in(1), in(0), 0;
    return out;
}

Matrix3f ESKF::rotVecToMat(const Vector3f& in) {
    float angle = in.norm();
    Vector3f axis = (angle == 0) ? Vector3f(1, 0, 0) : in.normalized();
    AngleAxisf angAx(angle, axis);
    return angAx.toRotationMatrix();
}

Quaternionf ESKF::rotVecToQuat(const Vector3f& in) {
    float angle = in.norm();
    Vector3f axis = (angle == 0) ? Vector3f(1, 0, 0) : in.normalized();
    return Quaternionf(AngleAxisf(angle, axis));
}

Vector3f ESKF::quatToRotVec(const Quaternionf& q) {
    AngleAxisf angAx(q);
    return angAx.angle() * angAx.axis();
}

void ESKF::predictIMU(const Vector3f& a_m, const Vector3f& omega_m, const float dt) {
    // DCM of current state
    Matrix3f Rot = getDCM();
    // Accelerometer measurement
    Vector3f acc_body = a_m - getAccelBias();
    Vector3f acc_global = Rot * acc_body;
    // Gyro measruement
    Vector3f omega = omega_m - getGyroBias();
    Vector3f delta_theta = omega * dt;
    Quaternionf q_delta_theta = rotVecToQuat(delta_theta);
    Matrix3f R_delta_theta = q_delta_theta.toRotationMatrix();

    // Nominal state kinematics (eqn 259, pg 58)
    Vector3f delta_pos = getVel()*dt + 0.5f*(acc_global + a_gravity_)*dt*dt;
    nominalState_.block<3, 1>(POS_IDX, 0) += delta_pos;
    nominalState_.block<3, 1>(VEL_IDX, 0) += (acc_global + a_gravity_)*dt;
    nominalState_.block<4, 1>(QUAT_IDX, 0) = quatToHamilton(getQuat()*q_delta_theta).normalized();

    // // Jacobian of the state transition (eqn 269, page 59)
    // // Update dynamic parts only
    // // dPos row
    // F_x_.block<3, 3>(dPOS_IDX, dVEL_IDX).diagonal().fill(dt); // = I_3 * _dt
    // // dVel row
    // F_x_.block<3, 3>(dVEL_IDX, dTHETA_IDX) = -Rot * getSkew(acc_body) * dt;
    // F_x_.block<3, 3>(dVEL_IDX, dAB_IDX) = -Rot * dt;
    // // dTheta row
    // F_x_.block<3, 3>(dTHETA_IDX, dTHETA_IDX) = R_delta_theta.transpose();
    // F_x_.block<3, 3>(dTHETA_IDX, dGB_IDX).diagonal().fill(-dt); // = -I_3 * dt;

    // Predict P and inject variance (with diagonal optimization)
    // P_ = F_x_*P_*F_x_.transpose();

    Matrix<float, dSTATE_SIZE, dSTATE_SIZE> Pnew;
    unrolledFPFt(P_, Pnew, dt,
        -Rot * getSkew(acc_body) * dt,
        -Rot * dt,
        R_delta_theta.transpose());
    P_ = Pnew;

    // Inject process noise
    P_.diagonal().block<3, 1>(dVEL_IDX, 0).array() += var_acc_ * SQ(dt);
    P_.diagonal().block<3, 1>(dTHETA_IDX, 0).array() += var_omega_ * SQ(dt);
    P_.diagonal().block<3, 1>(dAB_IDX, 0).array() += var_acc_bias_ * dt;
    P_.diagonal().block<3, 1>(dGB_IDX, 0).array() += var_omega_bias_ * dt;
}

// eqn 280, page 62
Matrix<float, 4, 3> ESKF::getQ_dtheta() {
    Vector4f qby2 = 0.5f*getQuatVector();
    // Assing to letters for readability. Note Hamilton order.
    float w = qby2[0];
    float x = qby2[1];
    float y = qby2[2];
    float z = qby2[3];
    Matrix<float, 4, 3>Q_dtheta;
    Q_dtheta <<
        -x, -y, -z,
        w, -z, y,
        z, w, -x,
        -y, x, w;
    return Q_dtheta;
}

void ESKF::measurePos(const Vector3f& pos_meas, const Matrix3f& pos_covariance) {
    // delta measurement is trivial
    Vector3f delta_pos = pos_meas - getPos();
    // H is a trivial observation of purely the position
    Matrix<float, 3, dSTATE_SIZE> H;
    H.setZero();
    H.block<3, 3>(0, dPOS_IDX) = I_3;

    // Apply update
    update_3D(delta_pos, pos_covariance, H);
}

void ESKF::measureQuat(const Quaternionf& q_gb_meas, const Matrix3f& theta_covariance) {
    // Transform the quaternion measurement to a measurement of delta_theta:
    // a rotation in the body frame from nominal to measured.
    // This is identical to the form of dtheta in the error_state,
    // so this becomes a trivial measurement of dtheta.
    Quaternionf q_gb_nominal = getQuat();
    Quaternionf q_bNominal_bMeas = q_gb_nominal.conjugate() * q_gb_meas;
    Vector3f delta_theta = quatToRotVec(q_bNominal_bMeas);
    // Because of the above construction, H is a trivial observation of dtheta
    Matrix<float, 3, dSTATE_SIZE> H;
    H.setZero();
    H.block<3, 3>(0, dTHETA_IDX) = I_3;

    // Apply update
    update_3D(delta_theta, theta_covariance, H);
}

void ESKF::update_3D(
        const Vector3f& delta_measurement,
        const Matrix3f& meas_covariance,
        const Matrix<float, 3, dSTATE_SIZE>& H) {
    // Kalman gain
    Matrix<float, dSTATE_SIZE, 3> PHt = P_*H.transpose();
    Matrix<float, dSTATE_SIZE, 3> K = PHt * (H*PHt + meas_covariance).inverse();
    // Correction error state
    Matrix<float, dSTATE_SIZE, 1> errorState = K * delta_measurement;
    // Update P (simple form)
    // P = (I_dx - K*H)*P;
    // Update P (Joseph form)
    Matrix<float, dSTATE_SIZE, dSTATE_SIZE> I_KH = I_dx - K*H;
    P_ = I_KH*P_*I_KH.transpose() + K*meas_covariance*K.transpose();

    injectErrorState(errorState);
}

void ESKF::injectErrorState(const Matrix<float, dSTATE_SIZE, 1>& error_state) {\
    // Inject error state into nominal state (eqn 282, pg 62)
    nominalState_.block<3, 1>(POS_IDX, 0) += error_state.block<3, 1>(dPOS_IDX, 0);
    nominalState_.block<3, 1>(VEL_IDX, 0) += error_state.block<3, 1>(dVEL_IDX, 0);
    Vector3f dtheta = error_state.block<3, 1>(dTHETA_IDX, 0);
    Quaternionf q_dtheta = rotVecToQuat(dtheta);
    nominalState_.block<4, 1>(QUAT_IDX, 0) = quatToHamilton(getQuat()*q_dtheta).normalized();
    nominalState_.block<3, 1>(AB_IDX, 0) += error_state.block<3, 1>(dAB_IDX, 0);
    nominalState_.block<3, 1>(GB_IDX, 0) += error_state.block<3, 1>(dGB_IDX, 0);

    // Reflect this tranformation in the P matrix, aka ESKF Reset
    // Note that the document suggests that this step is optional
    // eqn 287, pg 63
    Matrix3f G_theta = I_3 - getSkew(0.5f * dtheta);
    P_.block<3, 3>(dTHETA_IDX, dTHETA_IDX) = 
            G_theta * P_.block<3, 3>(dTHETA_IDX, dTHETA_IDX) * G_theta.transpose();
}