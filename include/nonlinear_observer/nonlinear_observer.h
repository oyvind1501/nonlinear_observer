#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <unsupported/Eigen/MatrixFunctions>

constexpr int STATESIZE{9};
constexpr long double PI{ 3.1415926535 };
constexpr double GRAVITY{9.80771};

namespace nlo
{

constexpr int NOMINAL_POSITION_STATE_SIZE{ 3 };
constexpr int NOMINAL_VELOCITY_STATE_SIZE{ 3 };
constexpr int NOMINAL_QUATERNION_STATE_SIZE{ 4 };

struct parametersInNLO
{
  Eigen::Matrix<double, 1, 1> R_pressureZ;
  Eigen::Vector3d Sr_to_ned_accelerometer;
  Eigen::Vector3d Sr_to_ned_gyro;
  Eigen::Vector3d Sr_accelerometer_aligment;
  Eigen::Vector3d Sr_gyro_aligment;
  Eigen::Vector3d Sr_to_ned_dvl;
  Eigen::Vector3d Sr_dvl_alignment;
  Eigen::Matrix<double, 3, 3> S_dvl;
  Eigen::Matrix3d R_nb_hat;
  Eigen::Vector3d initial_b_ars_b_hat;
  Eigen::Vector3d initial_b_acc_b_hat;
  bool use_feedback_interconnection;
  Eigen::Matrix<double, STATESIZE, STATESIZE> initial_covariance;
  bool use_ENU;
  Eigen::Vector3d p_init{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_init{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_init{Eigen::Quaterniond::Identity()};
  Eigen::Matrix3d R_init{Eigen::Matrix3d::Zero()};
  Eigen::Vector3d b_acc_init{Eigen::Vector3d::Zero()};
  Eigen::Vector3d b_ars_init{Eigen::Vector3d::Zero()};
  Eigen::Vector3d xi_init{Eigen::Vector3d::Zero()};
  double k1;
  double k2;
  double ki;
  double mb;
  Eigen::Matrix<double,6,6> Q;
};


struct IMUmessage 
{
  bool predicted_msg_;
  double timeStamp_;
  double deltaIMU_;
  Eigen::Vector3d zAccMeasurement_;
  Eigen::Vector3d zGyroMeasurement_;
  Eigen::Matrix<double,3,3> R_acc_;
  Eigen::Matrix<double,3,3> R_gyro_;
  IMUmessage()
  :timeStamp_{0},deltaIMU_{0},predicted_msg_{false}
  {
    zAccMeasurement_.setZero();
    zGyroMeasurement_.setZero();
    R_acc_.setZero();
    R_gyro_.setZero();
  }
  IMUmessage(const double& timeStamp,const double& deltaIMU, const Eigen::Vector3d& zAcc, const Eigen::Vector3d& zGyro, const Eigen::Matrix<double,3,3>& Racc, const Eigen::Matrix<double,3,3> Rgyro)
  : timeStamp_{timeStamp},deltaIMU_{deltaIMU},zAccMeasurement_{zAcc},zGyroMeasurement_{zGyro},R_acc_{Racc},R_gyro_{Rgyro},predicted_msg_{false}
  {
  }

};

struct DVLmessage
{
  double timeStamp_;
  Eigen::Vector3d zDVl_;
  Eigen::Matrix<double,3,3> R_dvl_;
  DVLmessage()
  :timeStamp_{0}
  {
    zDVl_.setZero();
    R_dvl_.setZero();
  }
  DVLmessage(const double& timeStamp, const Eigen::Vector3d zDvl, const Eigen::Matrix<double,3,3> R_dvl)
  :timeStamp_{timeStamp},zDVl_{zDvl},R_dvl_{R_dvl}
  {
  }
};

struct PressureZmessage
{
  double timeStamp_;
  double pressureZ_msg_;
  Eigen::Matrix<double,1,1> R_pressureZ_;
  PressureZmessage()
  :timeStamp_{0}, pressureZ_msg_{0}
  {
    R_pressureZ_.setZero();
  }
  PressureZmessage(const double& timeStamp, const double& pressureZ_msg, Eigen::Matrix<double,1,1> R_pressureZ)
  :timeStamp_{timeStamp},pressureZ_msg_{pressureZ_msg},R_pressureZ_{R_pressureZ}
  {
  }
};


struct  OutputStatesAndErrorCovariance
{
    Eigen::Vector3d p_nb_n_hat{Eigen::Vector3d::Zero()};
    Eigen::Vector3d v_nb_n_hat{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond q_nb_hat{Eigen::Quaterniond::Identity()};
    Eigen::Matrix3d R_nb_hat{Eigen::Matrix3d::Zero()};
    Eigen::Vector3d b_ars_b_hat{Eigen::Vector3d::Zero()};
    Eigen::Vector3d b_acc_b_hat{Eigen::Vector3d::Zero()};
    Eigen::Vector3d xi_nb_n{Eigen::Vector3d::Zero()};
    Eigen::Matrix<double,STATESIZE,STATESIZE> P_hat{Eigen::Matrix<double,STATESIZE,STATESIZE>::Zero()};

    OutputStatesAndErrorCovariance(const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Quaterniond& q, const Eigen::Matrix3d& R, const Eigen::Vector3d& b_ars, const Eigen::Vector3d& b_acc, const Eigen::Vector3d& xi, const Eigen::Matrix<double,STATESIZE,STATESIZE> P)
    :p_nb_n_hat{p},v_nb_n_hat{v},q_nb_hat{q},R_nb_hat{R},b_ars_b_hat{b_ars},b_acc_b_hat{b_acc},xi_nb_n{xi},P_hat{P}
    {
    }
};


struct  StatesAndErrorCovariance
{
    Eigen::Vector3d p_hat{Eigen::Vector3d::Zero()};
    Eigen::Vector3d v_hat{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond q_hat{Eigen::Quaterniond::Identity()};
    Eigen::Matrix3d R_hat{Eigen::Matrix3d::Zero()};
    Eigen::Vector3d b_ars_b_hat{Eigen::Vector3d::Zero()};
    Eigen::Vector3d b_acc_b_hat{Eigen::Vector3d::Zero()};
    Eigen::Vector3d xi_hat{Eigen::Vector3d::Zero()};
    Eigen::Matrix<double,STATESIZE,STATESIZE> P_hat{Eigen::Matrix<double,STATESIZE,STATESIZE>::Zero()};
    Eigen::VectorXd X_hat{Eigen::VectorXd::Zero(STATESIZE)};
};

struct  StatesAndErrorCovariance_next
{
    Eigen::Vector3d p_hat_next{Eigen::Vector3d::Zero()};
    Eigen::Vector3d v_hat_next{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond q_hat_next{Eigen::Quaterniond::Identity()};
    Eigen::Matrix3d R_hat_next{Eigen::Matrix3d::Zero()};
    Eigen::Vector3d b_ars_b_hat_next{Eigen::Vector3d::Zero()};
    Eigen::Vector3d b_acc_b_hat_next{Eigen::Vector3d::Zero()};
    Eigen::Vector3d xi_hat_next{Eigen::Vector3d::Zero()};
    Eigen::Matrix<double,STATESIZE,STATESIZE> P_hat_next{Eigen::Matrix<double,STATESIZE,STATESIZE>::Zero()};
    Eigen::VectorXd X_hat_next{Eigen::VectorXd::Zero(STATESIZE)};
};

struct OptimizationParameters
{
  Eigen::Vector3d v_b1_hat{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_b1{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_b2{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_n1{Eigen::Vector3d::Zero()};
  Eigen::Vector3d sigma_hat{Eigen::Vector3d::Zero()};
};



class NLO
{

public:

    explicit NLO(const parametersInNLO& parameters);

    //explicit NLO(const Eigen::Matrix3d& Sa, const Eigen::Matrix3d& Sg, const Eigen::Matrix3d& Sdvl,const bool& feedback_interconnection,
    //             const double& k1, const double& k2, const double& ki, const double& Mb, const Eigen::Matrix<double,6,6>& Q);

    void bufferIMUMessages(const Eigen::Vector3d& zAccMeasurements, const Eigen::Vector3d& zGyroMeasurements, const double& timeStamp, const double& deltaIMU,const Eigen::Matrix3d& Racc, const Eigen::Matrix3d& Rgyro);
    void bufferDVLMessages(const Eigen::Vector3d& zDvlMeasurements,const double timeStamp,const Eigen::Matrix3d& Rdvl);
    void bufferPressureZMessages(const double& pressureZ,const double& timeStamp, Eigen::Matrix<double,1,1> R_pressureZ);
    void firstAttitudeVectorPair(const Eigen::Vector3d& f_nb_b);
    void secondAttitudeVectorPair(const double& delta_dvl, const Eigen::Vector3d dvl_measurement);
    void attitudeCorrection();
    void ARSBiasCorrection();
    void TMOPositionMeasurements(const double& pressureZvalue, const double& pressureZleverarm, const double& R_pressure_Z);
    void attitudePredictionAhead(const Eigen::Vector3d omega_imu_b_mes);
    void posVelPredictionAhead(const Eigen::Vector3d& f_nb_b);
    void AllocateNLO();
    void AllocateTMO();
    void update();
    void updateFilter();
    void predict(const Eigen::Vector3d& gyro_msg, const Eigen::Vector3d& acc_msg, const double& Ts);
    void UpdatePressureZ(const double& zPressureZpos, const Eigen::MatrixXd& RpressureZ);
    void UpdateDVl(const Eigen::Vector3d& dvlmsg, const Eigen::Matrix3d& Rdvl);
    void TMOvelocityMeasurements(const Eigen::Vector3d& dvl_measurement, const Eigen::Matrix3d& R_dvl);

    inline Eigen::Quaterniond getQuaternion() const
  {
    if (use_ENU_)
    {
      return Eigen::Quaterniond{ states_.q_nb_hat.w(), states_.q_nb_hat.y(), states_.q_nb_hat.x(), states_.q_nb_hat.z() };
    }
    else  // use NED
    {
       return Eigen::Quaterniond{ states_.q_nb_hat.w(), states_.q_nb_hat.x(), states_.q_nb_hat.y(), states_.q_nb_hat.z() };
    }
  }

  inline Eigen::Vector3d getPosition() const
  {
    Eigen::Matrix3d R_ned_to_enu = Eigen::Matrix3d::Zero();
    Eigen::Vector3d position = Eigen::Vector3d::Zero();

    R_ned_to_enu << 0, 1, 0, 1, 0, 0, 0, 0, -1;

    position = states_.p_nb_n_hat;

    if (use_ENU_)
    {
      return R_ned_to_enu * position;
    }
    else
    {
      return position;
    }
  }

  inline Eigen::Vector3d getVelocity() const
  {
    Eigen::Matrix3d R_ned_to_enu = Eigen::Matrix3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();

    R_ned_to_enu << 0, 1, 0, 1, 0, 0, 0, 0, -1;

    velocity = states_.v_nb_n_hat;

    // return R_ned_to_enu*velocity;

    if (use_ENU_)
    {
      return R_ned_to_enu * velocity;
    }
    else
    {
      return velocity;
    }
  }

  inline Eigen::MatrixXd getErrorCovariance() const
  {
    return states_.P_hat;
  }

private:
    void rectifyIMUMessages();
    
    // Constants
    const Eigen::Matrix3d I_3x3_{Eigen::Matrix3d::Identity()};
    const Eigen::Matrix3d Zeros_3x3_{Eigen::Matrix3d::Zero()};
    const Eigen::Matrix<double,STATESIZE,STATESIZE> I_9x9_{Eigen::Matrix<double,STATESIZE,STATESIZE>::Identity()};
    const Eigen::Vector3d Zeros_3x1_{Eigen::Vector3d::Zero()};
    const Eigen::Vector3d g_b_n_{0,0,GRAVITY};
    double Ts_{1/125};

    // Correction matricies
    Eigen::Matrix3d Sa_{ Eigen::Matrix3d::Zero() };    // Accelerometer
    Eigen::Matrix3d Sg_{ Eigen::Matrix3d::Zero() };    // Gyro
    Eigen::Matrix3d Sdvl_{ Eigen::Matrix3d::Zero() };  // DVL
    Eigen::Matrix3d Sinc_{ Eigen::Matrix3d::Zero() };  // Inclinometer
    double SpressureZ_{ 0 };                           // Pressure

    // Tuning parameters

    double k1_;
    double k2_;
    double KI_;
    double Mb_;
    Eigen::Matrix<double,6,6> Q_;

    // Sensor buffer implementation
    std::vector<IMUmessage> imu_msg_buffer_;
    std::vector<DVLmessage> dvl_msg_buffer_;
    std::vector<PressureZmessage> pressureZ_msg_buffer_;
    void emptyIMUBuffer();
    void emptyDVLBuffer();
    void emptyPressureZBuffer();
    bool updated_;

    // States and error covariance
    StatesAndErrorCovariance_next next_Update_{};
    StatesAndErrorCovariance current_Update_{};
    OutputStatesAndErrorCovariance states_;
    
    OptimizationParameters optparams_{};

    // Feedback interconnection

    bool use_feedback_interconnection_;
    bool use_ENU_;
    
    

    
};

} //namespace nlo

