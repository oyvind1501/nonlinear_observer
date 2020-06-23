#include "nonlinear_observer.h"
#include <utility>
#include "common.h"

using namespace Eigen;
using namespace nlo;


NLO::NLO(const parametersInNLO& parameters)
  : Sa_{ eulerToRotationMatrix(parameters.Sr_to_ned_accelerometer + parameters.Sr_accelerometer_aligment)}
  , Sg_{ eulerToRotationMatrix(parameters.Sr_to_ned_gyro + parameters.Sr_gyro_aligment) }
  , Sdvl_{ eulerToRotationMatrix(parameters.Sr_to_ned_dvl + parameters.Sr_dvl_alignment) }
  , use_feedback_interconnection_{parameters.use_feedback_interconnection}
  , use_ENU_{ parameters.use_ENU }
  , states_{parameters.p_init,parameters.v_init,parameters.q_init,quat2RotMat_fast(parameters.q_init), parameters.b_ars_init, parameters.b_acc_init, parameters.xi_init, parameters.initial_covariance}
  , k1_{parameters.k1}
  , k2_{parameters.k2}
  , KI_{parameters.ki}
  , Mb_{parameters.mb}
  , Q_{parameters.Q}
{
  std::cout << "S_DVL: " << Sdvl_ << std::endl;
  std::cout << "Sr_to_ned_accelerometer: " << parameters.Sr_to_ned_accelerometer << std::endl;
  std::cout << "Sr_to_ned_gyro: " << parameters.Sr_to_ned_gyro << std::endl;
  std::cout << "Sr_to_ned_DVL: " <<parameters.Sr_to_ned_dvl << std::endl;
  std::cout << "Sr_accelerometer_alignment: "<<parameters.Sr_accelerometer_aligment << std::endl;
  std::cout << "Sr_gyro_alignment: "<<parameters.Sr_gyro_aligment << std::endl;
  std::cout << "Sr_dvl_alignment: " <<parameters.Sr_dvl_alignment << std::endl;
  std::cout << "S_a: "<<Sa_<<std::endl;
  std::cout << "Sr_to_ned_accelerometer + Sr_accelerometer_alignment: " << parameters.Sr_to_ned_accelerometer + parameters.Sr_accelerometer_aligment<< std::endl;
  std::cout << "Ki " << parameters.ki<<std::endl;
  std::cout << "k1 " << parameters.k1<<std::endl;
  std::cout << "k2 " << parameters.k2<<std::endl;
  std::cout << "mb: " << parameters.mb<<std::endl;
  std::cout << "Q: "  <<parameters.Q<<std::endl;
  std::cout << "p_init: "<<parameters.p_init<<std::endl;
  std::cout << "v_init: "<<parameters.v_init<<std::endl;
  std::cout << "q_init: "<<parameters.q_init.w()<<std::endl;
  std::cout << "R_init: "<<quat2RotMat_fast(parameters.q_init)<<std::endl;
  std::cout << "use_feedback_interconnection: "<<parameters.use_feedback_interconnection<<std::endl;

}

/*
NLO::NLO(const Eigen::Matrix3d& Sa, const Eigen::Matrix3d& Sg, const Eigen::Matrix3d& Sdvl,const bool& feedback_interconnection,
                 const double& k1, const double& k2, const double& ki, const double& Mb, const Eigen::Matrix<double,6,6>& Q)
: Sa_{std::move(Sa)},Sg_{std::move(Sg)},Sdvl_{std::move(Sdvl)},k1_{k1},k2_{k2},KI_{ki},Mb_{Mb}, Q_{Q},use_feedback_interconnection_{feedback_interconnection} 
{
  Vector3d euler_init{0.2618,-0.1047,0.7330};
  EulerAngles euler_init_2;
  euler_init_2.roll = 0.2618;
  euler_init_2.pitch = -0.1047;
  euler_init_2.yaw = 0.7330;
  states_.R_nb_hat = eulerToRotationMatrix(euler_init);
  states_.q_nb_hat = fromRPYToQuaternion(euler_init_2);
  states_.P_hat.setIdentity();
}
*/

void NLO::bufferIMUMessages(const Vector3d& zAccMeasurements, const Vector3d& zGyroMeasurements, const double& timeStamp, const double& deltaIMU,const Matrix3d& Racc, const Matrix3d& Rgyro)
{
  IMUmessage imu_msg{timeStamp,deltaIMU,zAccMeasurements,zGyroMeasurements,Racc,Rgyro};

  imu_msg_buffer_.push_back(imu_msg);

  if(imu_msg_buffer_.size() == 8)
  {
    emptyIMUBuffer();
  }
}

void NLO::bufferDVLMessages(const Vector3d& zDvlMeasurements,const double timeStamp,const Matrix3d& Rdvl)
{
  DVLmessage dvlmsg{timeStamp,zDvlMeasurements,Rdvl};

  dvl_msg_buffer_.push_back(dvlmsg);

  /*
  if(dvl_msg_buffer_.size() == 2)
  {
    emptyDVLBuffer();
  }
  */
}

void NLO::bufferPressureZMessages(const double& pressureZ,const double& timeStamp, Matrix<double,1,1> R_pressureZ)
{
  PressureZmessage prsmsg{timeStamp,pressureZ,R_pressureZ};

  pressureZ_msg_buffer_.push_back(prsmsg);

  /*
  if(pressureZ_msg_buffer_.size() == 2)
  {
    emptyPressureZBuffer();
  }
  */
}

void NLO::emptyIMUBuffer()
{
  imu_msg_buffer_ = std::vector<IMUmessage>{imu_msg_buffer_.back()};

  //imu_msg_buffer_.back()
}

void NLO::emptyDVLBuffer()
{
  dvl_msg_buffer_ = std::vector<DVLmessage>{};

  //dvl_msg_buffer_.back()
}

void NLO::emptyPressureZBuffer()
{
  pressureZ_msg_buffer_ = std::vector<PressureZmessage>{};

  //pressureZ_msg_buffer_.back()
}

void NLO::AllocateNLO()
{
  current_Update_.R_hat = states_.R_nb_hat;
  //std::cout<<"R_hat: "<<current_Update_.R_hat<<std::endl;
  current_Update_.q_hat = states_.q_nb_hat;
  //std::cout<<"q_hat: " << current_Update_.q_hat.w()<<std::endl;
  current_Update_.b_ars_b_hat = states_.b_ars_b_hat;
  //std::cout<<"b_ars_b_hat: "<<current_Update_.b_ars_b_hat<<std::endl;
  
}

void NLO::AllocateTMO()
{
  current_Update_.p_hat = states_.p_nb_n_hat;
  //std::cout<<"p: "<<current_Update_.p_hat<<std::endl;
  current_Update_.v_hat = states_.v_nb_n_hat;
  //std::cout<<"v: "<<current_Update_.v_hat<<std::endl;
  current_Update_.xi_hat = states_.xi_nb_n;
  //std::cout<<"xi: "<<current_Update_.xi_hat<<std::endl;
  current_Update_.P_hat = states_.P_hat;
  //std::cout<<"P: "<<current_Update_.P_hat<<std::endl;
  current_Update_.X_hat << current_Update_.p_hat, current_Update_.v_hat, current_Update_.xi_hat;
  //std::cout<<"X: "<<current_Update_.X_hat<<std::endl;
}


void NLO::update()
{
  AllocateNLO();
  AllocateTMO();  


  Eigen::Vector3d f_nb_b{-current_Update_.R_hat.transpose()*g_b_n_};

  firstAttitudeVectorPair(f_nb_b);

  Eigen::Vector3d dvl_msg{1,2,3};
  dvl_msg = dvl_msg*Ts_;
  //std::cout<<"dvl_msg: "<<dvl_msg<<std::endl;
  double dvl_delta_t{Ts_};
  //std::cout<<"dvl_delta_t: "<<dvl_delta_t<<std::endl;

  secondAttitudeVectorPair(dvl_delta_t,dvl_msg);



  attitudeCorrection();
  ARSBiasCorrection();

  double pressureZ{1};
  double lever_arm_pressure{0};
  double R_pressureZ{1};


  TMOPositionMeasurements(pressureZ,lever_arm_pressure,R_pressureZ);

  Vector3d omega_msg{1,2,3};
  omega_msg = omega_msg*0.0175;
  attitudePredictionAhead(omega_msg);

  posVelPredictionAhead(f_nb_b);


}

void NLO::predict(const Vector3d& gyro_msg, const Vector3d& acc_msg, const double& Ts)
{
    Vector3d accBias = Sa_ * states_.b_acc_b_hat;
    Vector3d gyroBias = Sg_ * states_.b_ars_b_hat;

    Vector3d accelerationRectified = (Sa_ * acc_msg) - accBias;
    Vector3d gyroRectified = (Sg_ * gyro_msg) - gyroBias;

     // Update and predict

     Ts_ = Ts;
    



    AllocateNLO();
    AllocateTMO();

    firstAttitudeVectorPair(accelerationRectified);
    attitudeCorrection();
    ARSBiasCorrection();
    attitudePredictionAhead(gyroRectified);
    posVelPredictionAhead(accelerationRectified);

}

void NLO::UpdatePressureZ(const double& zPressureZpos, const Eigen::MatrixXd& RpressureZ)
{
    const double pressureZleverarm{0};
    const double Rpressure{RpressureZ.coeff(0,0)};
    TMOPositionMeasurements(zPressureZpos,pressureZleverarm,Rpressure);
}

void NLO::UpdateDVl(const Vector3d& dvlmsg, const Matrix3d& Rdvl)
{
    const Vector3d dvl_msg{dvl_msg};
    const Matrix3d R_dvl{Rdvl};
    const double delta_t_dvl{1.0/8.0};
    secondAttitudeVectorPair(delta_t_dvl,dvl_msg);
    TMOvelocityMeasurements(dvl_msg,R_dvl);
}



void NLO::updateFilter()
{
    if(imu_msg_buffer_.size() != 0)
    {
        Vector3d gyroMessage{imu_msg_buffer_.front().zGyroMeasurement_};
        Vector3d accMessage{imu_msg_buffer_.front().zAccMeasurement_};
        
        double Ts{imu_msg_buffer_.front().deltaIMU_};

        // TODO: Maybe change this
        Ts_ = Ts;

        

        Vector3d accBias = Sa_ * states_.b_acc_b_hat;
        Vector3d gyroBias = Sg_ * states_.b_ars_b_hat;

        Vector3d accelerationRectified = (Sa_ * accMessage) - accBias;
        Vector3d gyroRectified = (Sg_ * gyroMessage) - gyroBias;


        // Update and predict

        AllocateNLO();
        AllocateTMO();

        firstAttitudeVectorPair(accelerationRectified);

        
        if (dvl_msg_buffer_.size() == 1)
        {
           
            const Vector3d dvl_msg{dvl_msg_buffer_.back().zDVl_};
            const Matrix3d R_dvl{dvl_msg_buffer_.back().R_dvl_};
            const double delta_t_dvl{1.0/8.0};
            secondAttitudeVectorPair(delta_t_dvl,dvl_msg);
            TMOvelocityMeasurements(dvl_msg,R_dvl);
        }
        

        attitudeCorrection();
        ARSBiasCorrection();

        
        if (pressureZ_msg_buffer_.size() == 1)
        {
            const double prs_msg{pressureZ_msg_buffer_.back().pressureZ_msg_};
            const Matrix<double,1,1> R_pressureZ{pressureZ_msg_buffer_.back().R_pressureZ_};
            const double pressureZleverarm{0};
            const double Rpressure{R_pressureZ.coeff(0,0)};
            TMOPositionMeasurements(prs_msg,pressureZleverarm,Rpressure);
            emptyPressureZBuffer();
            
        }

        if (dvl_msg_buffer_.size() == 1)
        {
           
            const Vector3d dvl_msg{dvl_msg_buffer_.back().zDVl_};
            const Matrix3d R_dvl{dvl_msg_buffer_.back().R_dvl_};
            const double delta_t_dvl{1.0/8.0};
            TMOvelocityMeasurements(dvl_msg,R_dvl);
            emptyDVLBuffer();
        }

        
        
        attitudePredictionAhead(gyroRectified);
        posVelPredictionAhead(accelerationRectified);
    }
   
}

void NLO::firstAttitudeVectorPair(const Eigen::Vector3d& f_nb_b)
{
  



    Vector3d v_b1{f_nb_b/f_nb_b.norm()};
    Vector3d f_ib_n_hat{Vector3d::Zero()};
    Vector3d v_n1{Vector3d::Zero()};

    if(use_feedback_interconnection_ == true)
    {
        f_ib_n_hat = current_Update_.R_hat*f_nb_b + current_Update_.xi_hat;
        v_n1 = f_ib_n_hat/f_ib_n_hat.norm();
    }
    else
    {
        v_n1 = -g_b_n_/g_b_n_.norm();
    }

    Vector3d v_b1_hat{current_Update_.R_hat.transpose()*v_n1};

    // TODO: Check if crossproductmatrix is the same as cross in Matlab
    //Vector3d sigma_hat{Eigen::Vector3d::Zero()};
    Vector3d sigma_hat;
    sigma_hat.setZero();
    sigma_hat = crossProductMatrix(k1_*v_b1)*v_b1_hat; // k1_*

    optparams_.v_b1 = v_b1;
    //std::cout<<"V_b1: "<<v_b1<<std::endl;
    optparams_.v_b1_hat = v_b1_hat;
    //std::cout<<"V_b1_hat: "<<v_b1_hat<<std::endl;
    optparams_.sigma_hat = sigma_hat;
    //std::cout<<"Sigma hat: "<<sigma_hat<<std::endl;
    optparams_.v_n1 = v_n1;
    //std::cout<<"V_n1:  "<<v_n1<<std::endl;

}

void NLO::secondAttitudeVectorPair(const double& delta_t_dvl, const Eigen::Vector3d dvl_measurement)
{
  const double dvl_scale{delta_t_dvl/Ts_};
  //std::cout<<"dvl_scale: "<<dvl_scale<<std::endl;
  Vector3d v_b_dvl{dvl_measurement};
  //std::cout<<"v_b_dvl: "<<v_b_dvl<<std::endl;
  Vector3d v_b2 = v_b_dvl/std::max(v_b_dvl.norm(),0.001); // 0.001
  //std::cout<<"V_b2: "<<v_b2<<std::endl;
  v_b2 = crossProductMatrix(optparams_.v_b1)*v_b2;
  //std::cout<<"v_b2: "<<v_b2<<std::endl;

  Vector3d v_n2{current_Update_.v_hat/std::max(current_Update_.v_hat.norm(),0.001)};
  //std::cout<<"v_n2"<<v_n2<<std::endl;
  v_n2 = crossProductMatrix(optparams_.v_n1)*v_n2;
  //std::cout<<"v_n2"<<v_n2<<std::endl;
  Vector3d v_b2_hat{current_Update_.R_hat.transpose()*v_n2};
  //std::cout<<"v_b2_hat"<<v_b2_hat<<std::endl;
  optparams_.sigma_hat = optparams_.sigma_hat + (dvl_scale*crossProductMatrix(k2_*v_b2)*v_b2_hat); // k2_*
  //std::cout<<"sigma_hat: "<<optparams_.sigma_hat<<std::endl;
}


void NLO::attitudeCorrection()
{
  Vector3d delta_sigma{Ts_*optparams_.sigma_hat};
  //std::cout<<"delta_sigma: "<<delta_sigma<<std::endl;
  double delta_sigma_norm{delta_sigma.norm()};
  //std::cout<<"delta_sigma_norm: "<<delta_sigma_norm<<std::endl;
  Vector3d sinus_sigma{sin(delta_sigma(0)/2.0),sin(delta_sigma(1)/2.0),sin(delta_sigma(2)/2.0)};
  //std::cout<<"sinus_sigma: "<<sinus_sigma<<std::endl;
  double delta_sigma_norm_product{sinus_sigma.transpose()*(delta_sigma/delta_sigma_norm)};
  //std::cout<<"delta_sigma_norm_product: "<<delta_sigma_norm_product<<std::endl;
  if(delta_sigma_norm > 1e-10)
  {
    //Quaterniond q_simga{cos(delta_sigma(0)/2.0),cos(delta_sigma(1)/2.0),cos(delta_sigma(2)/2.0),delta_sigma_norm_product};
     
    Quaterniond q_simga{cos(delta_sigma_norm/2.0),sin(delta_sigma_norm/2.0)*delta_sigma(0)/delta_sigma_norm,sin(delta_sigma_norm/2.0)*delta_sigma(1)/delta_sigma_norm,sin(delta_sigma_norm/2.0)*delta_sigma(2)/delta_sigma_norm};
    //std::cout<< "q_sigma.w" <<q_simga.w()<< "q_sigma.x" <<q_simga.x() << "q_sigma.y" <<q_simga.y() << "q_sigma.z" <<q_simga.z()<<std::endl;
    current_Update_.q_hat = quaternionHamiltonProduct(current_Update_.q_hat,q_simga);
    //std::cout<< "q_hat.w" <<current_Update_.q_hat.w()<< "q_hat.x" <<current_Update_.q_hat.x() << "q_hat.y" <<current_Update_.q_hat.y() << "q_hat.z" <<current_Update_.q_hat.z()<<std::endl; 
    current_Update_.q_hat.normalize();

  }
  current_Update_.R_hat = quat2RotMat_fast(current_Update_.q_hat);
  //std::cout<<"R_hat: "<<current_Update_.R_hat<<std::endl;
}

void NLO::ARSBiasCorrection()
{
  Vector3d b_ars_b_hat_dot{-KI_*optparams_.sigma_hat};
  //std::cout<<"b_ars_b_hat_dot: "<<b_ars_b_hat_dot<<std::endl;
  double b_hat_square = current_Update_.b_ars_b_hat.transpose()*current_Update_.b_ars_b_hat;
  //std::cout<<"b_hat_square: "<<b_hat_square<<std::endl;
  if(b_hat_square > Mb_*Mb_ && current_Update_.b_ars_b_hat.transpose()*b_ars_b_hat_dot > 0)
  {
    double c{1};
    b_ars_b_hat_dot = (I_3x3_-c*(current_Update_.b_ars_b_hat*current_Update_.b_ars_b_hat.transpose())/b_hat_square)*b_ars_b_hat_dot;
    //std::cout<<"b_ars_b_hat: "<<b_ars_b_hat_dot<<std::endl;
  }
  current_Update_.b_ars_b_hat = current_Update_.b_ars_b_hat + (Ts_*b_ars_b_hat_dot);
  //std::cout<<"b_ars_bat: "<<current_Update_.b_ars_b_hat<<std::endl;
}

void NLO::TMOPositionMeasurements(const double& pressureZvalue, const double& pressureZleverarm, const double& R_pressure_Z)
{
  
  double y{pressureZvalue};
  double y_hat = current_Update_.p_hat(2);
  //std::cout<<"y_hat: "<<y_hat<<std::endl;
  Matrix<double,1,9> H;
  H.setZero();
  H << 0,0,1,0,0,0,0,0,0;
  MatrixXd Kalman_gain = (current_Update_.P_hat*H.transpose())/(H*current_Update_.P_hat*H.transpose() + R_pressure_Z);
  //std::cout<<"Kalman gain:"<<Kalman_gain<<std::endl;
  current_Update_.X_hat = current_Update_.X_hat + Kalman_gain*(y-y_hat);
  //std::cout<<"X_hat: "<<current_Update_.X_hat<<std::endl;
  current_Update_.P_hat =(I_9x9_ - Kalman_gain*H)*current_Update_.P_hat*(I_9x9_-Kalman_gain*H).transpose() + Kalman_gain*R_pressure_Z*Kalman_gain.transpose();
  //std::cout<<"P_hat"<<current_Update_.P_hat<<std::endl;
}

void NLO::TMOvelocityMeasurements(const Eigen::Vector3d& dvl_measurement, const Eigen::Matrix3d& R_dvl)
{
  Vector3d y_vel{dvl_measurement};
  Vector3d y_hat_vel{current_Update_.R_hat.transpose()*current_Update_.v_hat};
  Matrix<double,3,9> H;
  H.setZero();
  H << Zeros_3x3_,current_Update_.R_hat.transpose(),Zeros_3x3_;
  MatrixXd Kalman_gain = (current_Update_.P_hat*H.transpose())*(H*current_Update_.P_hat*H.transpose() + R_dvl).inverse();
  //std::cout<<"Kalman gain:"<<Kalman_gain<<std::endl;
  current_Update_.X_hat = current_Update_.X_hat + Kalman_gain*(y_vel-y_hat_vel);
  //std::cout<<"X_hat: "<<current_Update_.X_hat<<std::endl;
  current_Update_.P_hat =(I_9x9_ - Kalman_gain*H)*current_Update_.P_hat*(I_9x9_-Kalman_gain*H).transpose() + Kalman_gain*R_dvl*Kalman_gain.transpose();
  //std::cout<<"P_hat"<<current_Update_.P_hat<<std::endl;
}

void NLO::attitudePredictionAhead(const Eigen::Vector3d omega_imu_b_mes)
{
  Vector3d delta_att{Ts_*(omega_imu_b_mes - current_Update_.b_ars_b_hat)}; //- current_Update_.b_ars_b_hat)};  /// --------------NBNBNBNB
  //std::cout<<"delta_att: "<<delta_att<<std::endl;
  double delta_att_norm{delta_att.norm()};
  //std::cout<<"delta_att_norm: "<<delta_att_norm<<std::endl;
  Vector3d sinus_sigma{sin(delta_att(0)/2.0),sin(delta_att(1)/2.0),sin(delta_att(2)/2.0)};
  //std::cout<<"sinus_sigma: "<<sinus_sigma<<std::endl; 
  double delta_sigma_norm_product{sinus_sigma.transpose()*(delta_att/delta_att_norm)};
  //std::cout<<"delta_sigma_norm_product: "<<delta_sigma_norm_product<<std::endl;
  if(delta_att_norm > 1e-10)
  {
    Quaterniond delta_q{cos(delta_att_norm/2.0),sin(delta_att_norm/2.0)*delta_att(0)/delta_att_norm,sin(delta_att_norm/2.0)*delta_att(1)/delta_att_norm,sin(delta_att_norm/2.0)*delta_att(2)/delta_att_norm};
    //std::cout<< "delta_q.w" <<delta_q.w()<< "delta_q.x: " <<delta_q.x() << "delta_q.y" <<delta_q.y() << "delta_q.z" <<delta_q.z()<<std::endl;
    current_Update_.q_hat = quaternionHamiltonProduct(current_Update_.q_hat,delta_q);
    //std::cout<< "q_hat.w" <<current_Update_.q_hat.w()<< "q_hat.x" <<current_Update_.q_hat.x() << "q_hat.y" <<current_Update_.q_hat.y() << "q_hat.z" <<current_Update_.q_hat.z()<<std::endl;
    next_Update_.q_hat_next = current_Update_.q_hat.normalized();
  } 
  else
  {
    next_Update_.q_hat_next = current_Update_.q_hat;
    //std::cout<<"q_hat_next.w: "<<next_Update_.q_hat_next.w()<<std::endl;
  }

  next_Update_.R_hat_next = quat2RotMat_fast(next_Update_.q_hat_next);
  //std::cout<<next_Update_.R_hat_next<<std::endl;
}

void NLO::posVelPredictionAhead(const Eigen::Vector3d& f_nb_b)
{
  Matrix<double,9,9> Phi;
  Phi.setZero();

  Phi.block<3,3>(0,0) = I_3x3_;
  Phi.block<3,3>(0,3) = Ts_*I_3x3_;
  Phi.block<3,3>(0,6) = Ts_*Ts_/2.0*I_3x3_;
  Phi.block<3,3>(3,0) = Zeros_3x3_;
  Phi.block<3,3>(3,3) = Zeros_3x3_;
  Phi.block<3,3>(3,6) = Ts_*I_3x3_;
  Phi.block<3,3>(6,0) = Zeros_3x3_;
  Phi.block<3,3>(6,3) = Zeros_3x3_;
  Phi.block<3,3>(6,6) = I_3x3_;

  //std::cout<<"Phi: "<<Phi<<std::endl;

  Matrix<double,9,6> Bd;
  Bd.setZero();

  Bd.block<3,3>(0,0) = (current_Update_.R_hat*Ts_*Ts_)/2.0;
  Bd.block<3,3>(0,3) = (current_Update_.R_hat*Ts_*Ts_*Ts_)/6.0;
  Bd.block<3,3>(3,0) = (current_Update_.R_hat*Ts_);
  Bd.block<3,3>(3,3) = (current_Update_.R_hat*Ts_*Ts_)/2.0;
  Bd.block<3,3>(6,0) = Zeros_3x3_;
  Bd.block<3,3>(6,3) = current_Update_.R_hat*Ts_;

  //std::cout<<"Bd: "<<Bd<<std::endl;

  int xi_scale{0};

  if(use_feedback_interconnection_ == true)
  {
    xi_scale = 1;
  }
  else
  {
    xi_scale = 0;
    Phi.block<6,3>(0,6) = Matrix<double,6,3>::Zero();
  }

  //std::cout<<"Phi :"<<Phi<<std::endl;

  VectorXd u{Eigen::VectorXd::Zero(6)};

  u << f_nb_b, -crossProductMatrix(optparams_.sigma_hat)*f_nb_b*xi_scale;

  //std::cout<<"u: "<<u<<std::endl;

  VectorXd Dd{Eigen::VectorXd::Zero(STATESIZE)};

  Dd << (g_b_n_*Ts_*Ts_)/2.0, g_b_n_*Ts_,Zeros_3x1_;

  //std::cout<<"Dd: "<<Dd<<std::endl;

  next_Update_.X_hat_next = Phi*current_Update_.X_hat + Bd*u + Dd;

  //std::cout<<"X_hat_next: "<<next_Update_.X_hat_next<<std::endl;

  Matrix<double,9,6> G{Matrix<double,9,6>::Zero()};

  G.block<3,3>(0,0) = Zeros_3x3_;
  G.block<3,3>(0,3) = Zeros_3x3_;
  G.block<3,3>(3,0) = current_Update_.R_hat;
  G.block<3,3>(3,3) = Zeros_3x3_;
  G.block<3,3>(6,0) = Zeros_3x3_;
  G.block<3,3>(6,3) = current_Update_.R_hat;

  //std::cout<<"G: "<<G<<std::endl;

  Matrix<double,9,6> G_next{Matrix<double,9,6>::Zero()};

  G_next.block<3,3>(0,0) = Zeros_3x3_;
  G_next.block<3,3>(0,3) = Zeros_3x3_;
  G_next.block<3,3>(3,0) = next_Update_.R_hat_next;
  G_next.block<3,3>(3,3) = Zeros_3x3_;
  G_next.block<3,3>(6,0) = Zeros_3x3_;
  G_next.block<3,3>(6,3) = next_Update_.R_hat_next;

  //std::cout<<"G_next: "<<G_next<<std::endl;

  Matrix<double,9,9> Qd{Matrix<double,9,9>::Zero()};

  Qd = (0.5)*(Phi*G*Q_*G.transpose()*Phi.transpose() + G_next*Q_*G_next.transpose())*Ts_;

  //std::cout<<"Qd:" <<Qd<<std::endl;

  next_Update_.P_hat_next = Phi*current_Update_.P_hat*Phi.transpose() + Qd;
  //std::cout<<"P_hat_next: "<<next_Update_.P_hat_next<<std::endl;
  next_Update_.P_hat_next = (next_Update_.P_hat_next + next_Update_.P_hat_next.transpose())/2.0;
  //std::cout<<"P_hat_next: "<<next_Update_.P_hat_next<<std::endl;



  states_.p_nb_n_hat = next_Update_.X_hat_next.block<3,1>(0,0);
  //std::cout<<"p_nb_n_hat: "<<states_.p_nb_n_hat<<std::endl; 
  states_.v_nb_n_hat = next_Update_.X_hat_next.block<3,1>(3,0);
  //std::cout<<"v_nb_n_hat: "<<states_.v_nb_n_hat<<std::endl;
  states_.xi_nb_n = next_Update_.X_hat_next.block<3,1>(6,0);
  //std::cout<<"xi_nb_n: "<<states_.xi_nb_n<<std::endl;
  states_.P_hat = next_Update_.P_hat_next;
  //std::cout<<"P_hat: "<<states_.P_hat<<std::endl;
  states_.q_nb_hat = next_Update_.q_hat_next;
  //std::cout<<"q_nb: "<<states_.q_nb_hat.w()<<std::endl;
  states_.R_nb_hat = next_Update_.R_hat_next;
  //std::cout<<"R_nb_hat: "<<states_.R_nb_hat<<std::endl;
  states_.b_ars_b_hat = current_Update_.b_ars_b_hat;
  //std::cout<<"b_ars_b_hat: "<<states_.b_ars_b_hat<<std::endl;
}