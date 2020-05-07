
#include "nonlinear_observer.h"
#include <Eigen/Core>
#include "ros_node.h"
#include "chrono"

using namespace Eigen;
using namespace nlo;


NLO_Node::NLO_Node(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_{ pnh }
  , init_{ false }
  , nlo_{ loadParametersFromYamlFile() }
  , publish_execution_time_imu_{true}
  , publish_execution_time_dvl_{true}
  , publish_execution_time_PressureZ_{true}
{
  R_dvl_.setZero();
  R_pressureZ_.setZero();
  std::string imu_topic{ "" };
  std::string dvl_topic{ "" };
  std::string pressureZ_topic{ "" };
  int publish_rate{ 125 };


  setRdvlFromYamlFile(R_dvl_);
  setRpressureZFromYamlFile(R_pressureZ_);
  setIMUTopicNameFromYaml(imu_topic);
  setDVLTopicNameFromYawl(dvl_topic);
  setPressureZTopicNameFromYaml(pressureZ_topic);
  setPublishrateFromYaml(publish_rate);

  ROS_INFO("Subscribing to IMU topic: %s", imu_topic.c_str());
  // Subscribe to IMU
  subscribeIMU_ = nh_.subscribe<sensor_msgs::Imu>(imu_topic, 1000, &NLO_Node::imuCallback, this,
                                                  ros::TransportHints().tcpNoDelay(true));
  // Subscribe to DVL
  ROS_INFO("Subscribing to DVL: %s", dvl_topic.c_str());
  subcribeDVL_ = nh_.subscribe<nav_msgs::Odometry>(dvl_topic, 1000, &NLO_Node::dvlCallback, this,
                                                   ros::TransportHints().tcpNoDelay(true));
  // Subscribe to Pressure sensor
  ROS_INFO("Subscribing to pressure sensor: %s", pressureZ_topic.c_str());
  subscribePressureZ_ = nh_.subscribe<nav_msgs::Odometry>(pressureZ_topic, 1000, &NLO_Node::pressureZCallback, this,
                                                          ros::TransportHints().tcpNoDelay(true));

  ROS_INFO("Publishing State");
  publishPose_ = nh_.advertise<nav_msgs::Odometry>("pose", 1);

  pubTImer_ = nh_.createTimer(ros::Duration(1.0f / publish_rate), &NLO_Node::publishPoseState, this);
}

// IMU Subscriber
void NLO_Node::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_Message_data)
{

  //double Ts{ 0 };
  int imu_publish_rate{ DEFAULT_IMU_RATE };
  Vector3d raw_acceleration_measurements = Vector3d::Zero();
  Vector3d raw_gyro_measurements = Vector3d::Zero();
  Matrix3d R_acc = Matrix3d::Zero();
  Matrix3d R_gyro = Matrix3d::Zero();

  //Ts = (1.0 / imu_publish_rate);
  raw_acceleration_measurements << imu_Message_data->linear_acceleration.x, imu_Message_data->linear_acceleration.y,
    imu_Message_data->linear_acceleration.z;

  for (size_t i = 0; i < R_acc.rows(); i++)
  {
    for (size_t j = 0; j < R_acc.cols(); j++)
    {
      R_acc(i, j) = imu_Message_data->linear_acceleration_covariance[3 * i + j];
    }
  }

  raw_gyro_measurements << imu_Message_data->angular_velocity.x, imu_Message_data->angular_velocity.y,
    imu_Message_data->angular_velocity.z;

  for (size_t i = 0; i < R_gyro.rows(); i++)
  {
    for (size_t j = 0; j < R_gyro.cols(); j++)
    {
      R_gyro(i, j) = imu_Message_data->angular_velocity_covariance[3 * i + j];
    }
  }

  // ROS_INFO("Acceleration_x: %f",imu_Message_data->linear_acceleration.x);

  if (previousTimeStampIMU_.sec != 0)
  {
    const double deltaIMU = (imu_Message_data->header.stamp - previousTimeStampIMU_).toSec();

    //ROS_INFO("TimeStamps: %f",delta);

    if(init_ == false)
    {
      initialIMUTimestamp_ = imu_Message_data->header.stamp.toSec();
      init_ = true;
      ROS_INFO("NLO initilized");
    }

    const double ros_timeStampNow = imu_Message_data->header.stamp.toSec() - initialIMUTimestamp_; 

    //ROS_INFO("IMU_timeStamp: %f",ros_timeStampNow);

    // Execution time
    auto start = std::chrono::steady_clock::now();
    //nlo_.bufferIMUMessages(raw_acceleration_measurements,raw_gyro_measurements,ros_timeStampNow,deltaIMU,R_acc,R_gyro);
    nlo_.predict(raw_gyro_measurements,raw_acceleration_measurements,ros_timeStampNow); 
	auto end = std::chrono::steady_clock::now();

	auto diff = end - start;

	auto diff_in_ms = std::chrono::duration <double, std::milli> (diff).count();

    //std::cout<<diff_in_ms<<std::endl;
    if(execution_time_vector_imu_.size() == 1000 && publish_execution_time_imu_ == true)
    {
      std::cout<<"Max value of IMU: "<<maxOfVector(execution_time_vector_imu_)<<std::endl;
      std::cout<<"Mean value of IMU: "<<meanOfVector(execution_time_vector_imu_)<<std::endl;
      std::cout<<"STD value of IMU: "<<stanardDeviationOfVector(execution_time_vector_imu_)<<std::endl;
      publish_execution_time_imu_ = false;
    }
    else
    {
      execution_time_vector_imu_.push_back(diff_in_ms);
    }

    //eskf_.predict();
  }

  previousTimeStampIMU_ = imu_Message_data->header.stamp;

  
}

// DVL subscriber
void NLO_Node::dvlCallback(const nav_msgs::Odometry::ConstPtr& dvl_Message_data)
{
  Vector3d raw_dvl_measurements = Vector3d::Zero();
  Matrix3d R_dvl = Matrix3d::Zero();

  raw_dvl_measurements << dvl_Message_data->twist.twist.linear.x, dvl_Message_data->twist.twist.linear.y,
    dvl_Message_data->twist.twist.linear.z;

  for (size_t i = 0; i < R_dvl.rows(); i++)
  {
    for (size_t j = 0; j < R_dvl.cols(); j++)
    {
      R_dvl(i, j) = dvl_Message_data->twist.covariance[3 * i + j];
    }
  }

  
  const double ros_timeStampNow = 1.0/8.0;//dvl_Message_data->header.stamp.toSec() - initialIMUTimestamp_; 

  auto start = std::chrono::steady_clock::now();
  nlo_.bufferDVLMessages(raw_dvl_measurements,ros_timeStampNow,R_dvl_);
  nlo_.UpdateDVl(raw_dvl_measurements,R_dvl_);
  auto end = std::chrono::steady_clock::now();

  auto diff = end - start;

  auto diff_in_ms = std::chrono::duration <double, std::milli> (diff).count();

  //std::cout<<diff_in_ms<<std::endl;
  if(execution_time_vector_dvl_.size() == 500 && publish_execution_time_dvl_ == true)
  {
    std::cout<<"Max value of DVL: "<<maxOfVector(execution_time_vector_dvl_)<<std::endl;
    std::cout<<"Mean value of DVL: "<<meanOfVector(execution_time_vector_dvl_)<<std::endl;
    std::cout<<"STD value of DVL: "<<stanardDeviationOfVector(execution_time_vector_dvl_)<<std::endl;
    publish_execution_time_dvl_ = false;
  }
  else
  {
    execution_time_vector_dvl_.push_back(diff_in_ms);
  }
}

// PressureZ subscriber
void NLO_Node::pressureZCallback(const nav_msgs::Odometry::ConstPtr& pressureZ_Message_data)
{
  Matrix<double, 1, 1> RpressureZ;
  const double raw_pressure_z = pressureZ_Message_data->pose.pose.position.z;

  RpressureZ(0) = pressureZ_Message_data->pose.covariance[0];

  // std::cout<<RpressureZ<<std::endl;
  // const double R_pressureZ = 2.2500;

  
  
  const double ros_timeStampNow = pressureZ_Message_data->header.stamp.toSec() - initialIMUTimestamp_; 

  //ROS_INFO("PressureZ_timeStamp: %f",ros_timeStampNow);

  auto start = std::chrono::steady_clock::now();
  //nlo_.bufferPressureZMessages(raw_pressure_z,ros_timeStampNow,R_pressureZ_);
  nlo_.UpdatePressureZ(raw_pressure_z,R_pressureZ_);
  auto end = std::chrono::steady_clock::now();

  auto diff = end - start;

  auto diff_in_ms = std::chrono::duration <double, std::milli> (diff).count();

  //std::cout<<diff_in_ms<<std::endl;
  if(execution_time_vector_pressureZ_.size() == 500 && publish_execution_time_PressureZ_ == true)
  {
    std::cout<<"Max value of PressureZ: "<<maxOfVector(execution_time_vector_pressureZ_)<<std::endl;
    std::cout<<"Mean value of PressureZ: "<<meanOfVector(execution_time_vector_pressureZ_)<<std::endl;
    std::cout<<"STD value of PressureZ: "<<stanardDeviationOfVector(execution_time_vector_pressureZ_)<<std::endl;
    publish_execution_time_PressureZ_ = false;
  }
  else
  {
    execution_time_vector_pressureZ_.push_back(diff_in_ms);
  }
}

void NLO_Node::publishPoseState(const ros::TimerEvent&)
{

  //nlo_.updateFilter();

  nav_msgs::Odometry odom_msg;
  static size_t trace_id{ 0 };

  const Vector3d& position = nlo_.getPosition();
  const Vector3d& velocity = nlo_.getVelocity();
  Quaterniond quaternion = nlo_.getQuaternion();

  const MatrixXd& errorCovariance = nlo_.getErrorCovariance();

  Matrix<double,3,3> position_error_covariance;
  position_error_covariance.setZero();
  position_error_covariance = errorCovariance.block<3,3>(0,0);
  Matrix<double,3,3> velocity_error_covariance;
  velocity_error_covariance.setZero();
  velocity_error_covariance = errorCovariance.block<3,3>(3,3);
  Matrix<double,3,3> attitude_error_covariance;
  attitude_error_covariance.setZero();
  attitude_error_covariance = errorCovariance.block<3,3>(6,6);

  odom_msg.header.frame_id = "/eskf_link";
  odom_msg.header.seq = trace_id++;
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.pose.pose.position.x = position(0);  // NEDpose(StateMemberX);
  odom_msg.pose.pose.position.y = position(1);  // NEDpose(StateMemberY)
  odom_msg.pose.pose.position.z = position(2);  // NEDpose(StateMemberZ);
  odom_msg.twist.twist.linear.x = velocity(0);             // NEDpose(StateMemberVx);
  odom_msg.twist.twist.linear.y = velocity(1);             // NEDpose(StateMemberVy);
  odom_msg.twist.twist.linear.z = velocity(2);             // NEDpose(StateMemberVz);
  odom_msg.pose.pose.orientation.w = quaternion.w();       // pose(StateMemberQw);
  odom_msg.pose.pose.orientation.x = quaternion.x();       // pose(StateMemberQx);
  odom_msg.pose.pose.orientation.y = quaternion.y();       // pose(StateMemberQy);
  odom_msg.pose.pose.orientation.z = quaternion.z();       // pose(StateMemberQz);
  // odom_msg.pose.covariance

  // Position covariance
  for(size_t i = 0; i < NOMINAL_POSITION_STATE_SIZE;i++)
  {
      for(size_t j = 0; j<NOMINAL_POSITION_STATE_SIZE;j++)
      {
          odom_msg.pose.covariance[NOMINAL_POSITION_STATE_SIZE*i+j] = position_error_covariance(i,j);
      }
  }

  
  for(size_t i = 0; i < NOMINAL_VELOCITY_STATE_SIZE; i++)
  {
      for(size_t j = 0; j< NOMINAL_VELOCITY_STATE_SIZE; j++)
      {
          odom_msg.twist.covariance[(NOMINAL_VELOCITY_STATE_SIZE*i+j)] = velocity_error_covariance(i,j);
      }
  }
  
    for(size_t i = 0; i < NOMINAL_QUATERNION_STATE_SIZE-1; i++)
  {
      for(size_t j = 0; j< NOMINAL_QUATERNION_STATE_SIZE-1; j++)
      {
          odom_msg.pose.covariance[((NOMINAL_QUATERNION_STATE_SIZE-1)*i+j)+9] = attitude_error_covariance(i,j);
      }
  }


  // ROS_INFO("StateX: %f",odom_msg.pose.pose.position.x);
  publishPose_.publish(odom_msg);

  // std::cout<<pose<<std::endl;
  // std::cout<<std::endl;
}


parametersInNLO NLO_Node::loadParametersFromYamlFile()
{
  parametersInNLO parameters;
  parameters.R_pressureZ.setZero();
  parameters.Sr_to_ned_accelerometer.setZero();
  parameters.Sr_to_ned_gyro.setZero();
  parameters.Sr_accelerometer_aligment.setZero();
  parameters.Sr_gyro_aligment.setZero();
  parameters.Sr_to_ned_dvl.setZero();
  parameters.Sr_dvl_alignment.setZero();
  parameters.S_dvl.setZero();


  parameters.use_ENU = true;
  parameters.initial_covariance.setZero();


  parameters.p_init.setZero();
  parameters.v_init.setZero();
  parameters.q_init.setIdentity();
  parameters.b_acc_init.setZero();
  parameters.b_ars_init.setZero();
  parameters.R_init.setZero();


  parameters.Q.setZero();
  parameters.k1 = 0;
  parameters.k2 = 0;
  parameters.ki = 0;
  parameters.mb = 0;
  
  parameters.use_feedback_interconnection = true;



  // parameters.initial_pose(16);
  // parameters.initial_pose.setZero();

  // std::cout<<parameters.initial_pose<<std::endl;
  // Eigen::MatrixXd R_acc(3,3);
  // R_acc.setZero();

  XmlRpc::XmlRpcValue R_pressureZConfig;
  XmlRpc::XmlRpcValue St_to_ned_accelerometer_Config;
  XmlRpc::XmlRpcValue St_to_ned_gyro_Config;
  XmlRpc::XmlRpcValue St_accelerometer_alignment_Config;
  XmlRpc::XmlRpcValue St_gyro_alignment_Config;
  XmlRpc::XmlRpcValue S_dvlConfig;
  XmlRpc::XmlRpcValue p_initConfig;
  XmlRpc::XmlRpcValue v_initConfig;
  XmlRpc::XmlRpcValue q_initConfig;
  XmlRpc::XmlRpcValue b_acc_initConfig;
  XmlRpc::XmlRpcValue b_ars_initConfig;
  XmlRpc::XmlRpcValue QConfig;
  XmlRpc::XmlRpcValue R_initConfig;
  XmlRpc::XmlRpcValue initialCovarianceConfig;
  XmlRpc::XmlRpcValue St_to_ned_DVL_Config;
  XmlRpc::XmlRpcValue St_dvl_alignment_Config;


  

  if (ros::param::has("/sr_accelerometer_to_NED"))
  {
    ros::param::get("/sr_accelerometer_to_NED", St_to_ned_accelerometer_Config);
    int matrix_size = parameters.Sr_to_ned_accelerometer.rows();

    for (int i = 0; i < matrix_size; i++)
    {
      std::ostringstream ostr;
      ostr <<St_to_ned_accelerometer_Config[i];
      std::istringstream istr(ostr.str());
      istr >> parameters.Sr_to_ned_accelerometer(i);
    }
  }
  else
  {
    ROS_FATAL("No static transform for accelerometer (St_acc) set in parameter file");
    ROS_BREAK();
  }

  if (ros::param::has("/sr_gyro_to_NED"))
  {
    ros::param::get("/sr_gyro_to_NED", St_to_ned_gyro_Config);
    int matrix_size = parameters.Sr_to_ned_gyro.rows();

    for (int i = 0; i < matrix_size; i++)
    {
      std::ostringstream ostr;
      ostr << St_to_ned_gyro_Config[i];
      std::istringstream istr(ostr.str());
      istr >> parameters.Sr_to_ned_gyro(i);
    }
  }
  else
  {
    ROS_FATAL("No static transform for gyro (St_gyro) set in parameter file");
    ROS_BREAK();
  }

  if (ros::param::has("/sr_accelerometer_alignment"))
  {
    ros::param::get("/sr_accelerometer_alignment", St_accelerometer_alignment_Config);
    int matrix_size = parameters.Sr_accelerometer_aligment.rows();

    for (int i = 0; i < matrix_size; i++)
    {
      std::ostringstream ostr;
      ostr << St_accelerometer_alignment_Config[i];
      std::istringstream istr(ostr.str());
      istr >> parameters.Sr_accelerometer_aligment(i);
    }
  }
  else
  {
    ROS_FATAL("No static transform for gyro (St_gyro) set in parameter file");
    ROS_BREAK();
  }



  if (ros::param::has("/sr_gyro_alignment"))
  {
    ros::param::get("/sr_gyro_alignment", St_gyro_alignment_Config);
    int matrix_size = parameters.Sr_gyro_aligment.rows();

    for (int i = 0; i < matrix_size; i++)
    {
      std::ostringstream ostr;
      ostr << St_gyro_alignment_Config[i];
      std::istringstream istr(ostr.str());
      istr >> parameters.Sr_gyro_aligment(i);
    }
  }
  else
  {
    ROS_FATAL("No static transform for gyro (St_gyro) set in parameter file");
    ROS_BREAK();
  }

  if(ros::param::has("/p_init"))
  {
      ros::param::get("/p_init",p_initConfig);
      int matrix_size = parameters.p_init.rows();

    for (int i = 0; i < matrix_size; i++)
    {
      std::ostringstream ostr;
      ostr << p_initConfig[i];
      std::istringstream istr(ostr.str());
      istr >> parameters.p_init(i);
    }
  }
  else
  {
      ROS_FATAL("No inital position set (p_init) set in parameter file");
      ros::shutdown();
  }

  if(ros::param::has("/v_init"))
  {
      ros::param::get("/v_init",v_initConfig);
      int matrix_size = parameters.v_init.rows();

    for (int i = 0; i < matrix_size; i++)
    {
      std::ostringstream ostr;
      ostr << v_initConfig[i];
      std::istringstream istr(ostr.str());
      istr >> parameters.v_init(i);
    }
  }
  else
  {
      ROS_FATAL("No inital velocity set (v_init) set in parameter file");
      ros::shutdown();
  }

  if(ros::param::has("/q_init"))
  {
    ros::param::get("/q_init",q_initConfig);
    int matrix_size = 4;

    std::ostringstream ostr1;
    ostr1 << q_initConfig[0];
    std::istringstream istr1(ostr1.str());
    istr1 >> parameters.q_init.w();

    std::ostringstream ostr2;
    ostr2 << q_initConfig[1];
    std::istringstream istr2(ostr2.str());
    istr2 >> parameters.q_init.x();

    std::ostringstream ostr3;
    ostr3 << q_initConfig[2];
    std::istringstream istr3(ostr3.str());
    istr3 >> parameters.q_init.y();

    std::ostringstream ostr4;
    ostr4 << q_initConfig[3];
    std::istringstream istr4(ostr4.str());
    istr4 >> parameters.q_init.z();
  }
  else
  {
      ROS_FATAL("No inital quaternion set (q_init) set in parameter file");
      ros::shutdown();
  }

   if(ros::param::has("/b_acc_init"))
  {
      ros::param::get("/b_acc_init",b_acc_initConfig);
      int matrix_size = parameters.b_acc_init.rows();

    for (int i = 0; i < matrix_size; i++)
    {
      std::ostringstream ostr;
      ostr << b_acc_initConfig[i];
      std::istringstream istr(ostr.str());
      istr >> parameters.b_acc_init(i);
    }
  }
  else
  {
      ROS_FATAL("No inital b_acc set (b_acc_init) set in parameter file");
      ros::shutdown();
  }

  if(ros::param::has("/b_ars_init"))
  {
      ros::param::get("/b_ars_init",b_ars_initConfig);
      int matrix_size = parameters.b_ars_init.rows();

    for (int i = 0; i < matrix_size; i++)
    {
      std::ostringstream ostr;
      ostr << b_ars_initConfig[i];
      std::istringstream istr(ostr.str());
      istr >> parameters.b_ars_init(i);
    }
  }
  else
  {
      ROS_FATAL("No inital b_ars set (b_ars_init) set in parameter file");
      ros::shutdown();
  }

   /*
    if (ros::param::has("/R_init"))
  {
    ros::param::get("/R_init", R_initConfig);
    int matrix_size = parameters.R_init.rows();

    for (int i = 0; i < matrix_size; i++)
    {
      for (int j = 0; j < matrix_size; j++)
      {
        std::ostringstream ostr;
        ostr << R_initConfig[matrix_size * i + j];
        std::istringstream istr(ostr.str());
        istr >> parameters.R_init(i, j);
      }
    }
  }
  else
  {
    ROS_FATAL("No initial Rotation matrix set in parameter file");
    ros::shutdown();
  }
   */

  if (ros::param::has("/k1"))
  {
    ros::param::get("/k1", parameters.k1);
  }
  else
  {
    ROS_FATAL("No k1 set in parameter file");
    ros::shutdown();
  }

   if (ros::param::has("/k2"))
  {
    ros::param::get("/k2", parameters.k2);
  }
  else
  {
    ROS_FATAL("No k2 set in parameter file");
    ros::shutdown();
  }

   if (ros::param::has("/ki"))
  {
    ros::param::get("/ki", parameters.ki);
  }
  else
  {
    ROS_FATAL("No ki set in parameter file");
    ros::shutdown();
  }

   if (ros::param::has("/mb"))
  {
    ros::param::get("/mb", parameters.mb);
  }
  else
  {
    ROS_FATAL("No mb set in parameter file");
    ros::shutdown();
  }

 if (ros::param::has("/use_feedback_interconnection"))
  {
    ros::param::get("/use_feedback_interconnection", parameters.use_feedback_interconnection);
  }
  else
  {
    ROS_FATAL("No bool value set for use_feedback interconnection in parameter file ");
    ros::shutdown();
  }



  if (ros::param::has("/Q"))
  {
    ros::param::get("/Q", QConfig);
    int matrix_size = parameters.Q.rows();

    for (int i = 0; i < matrix_size; i++)
    {
      for (int j = 0; j < matrix_size; j++)
      {
        std::ostringstream ostr;
        ostr << QConfig[matrix_size * i + j];
        std::istringstream istr(ostr.str());
        istr >> parameters.Q(i, j);
      }
    }
  }
  else
  {
    ROS_FATAL("No Q set in parameter file");
    ros::shutdown();
  }


  if (ros::param::has("/publish_in_ENU"))
  {
    ros::param::get("/publish_in_ENU", parameters.use_ENU);
  }
  else
  {
    ROS_FATAL("No bool value set for publish_in_ENU in parameter file! ");
    ROS_BREAK();
  }

  if (ros::param::has("/R_pressureZ"))
  {
    ros::param::get("/R_pressureZ", R_pressureZConfig);
    int matrix_size = parameters.R_pressureZ.rows();

    for (int i = 0; i < matrix_size; i++)
    {
      std::ostringstream ostr;
      ostr << R_pressureZConfig[i];
      std::istringstream istr(ostr.str());
      istr >> parameters.R_pressureZ(i);
    }
  }
  else
  {
    ROS_FATAL("No measurement covariance for pressure sensor (R_pressureZ) set in parameter file!");
    ROS_BREAK();
  }

    if (ros::param::has("/sr_dvl_alignment"))
  {
    ros::param::get("/sr_dvl_alignment", St_dvl_alignment_Config);
    int matrix_size = parameters.Sr_dvl_alignment.rows();

    for (int i = 0; i < matrix_size; i++)
    {
      std::ostringstream ostr;
      ostr << St_dvl_alignment_Config[i];
      std::istringstream istr(ostr.str());
      istr >> parameters.Sr_dvl_alignment(i);
    }
  }
  else
  {
    ROS_FATAL("No static rotation for alignment of Dvl set in parameter file");
    ROS_BREAK();
  }

   if (ros::param::has("/sr_dvl_to_NED"))
  {
    ros::param::get("/sr_dvl_to_NED", St_to_ned_DVL_Config);
    int matrix_size = parameters.Sr_to_ned_dvl.rows();

    for (int i = 0; i < matrix_size; i++)
    {
      std::ostringstream ostr;
      ostr << St_to_ned_DVL_Config[i];
      std::istringstream istr(ostr.str());
      istr >> parameters.Sr_to_ned_dvl(i);
    }
  }
  else
  {
    ROS_FATAL("No static rotation for DVL (sr_dvl_to_NED) set in parameter file");
    ROS_BREAK();
  }


  if (ros::param::has("/initial_covariance"))
  {
    ros::param::get("/initial_covariance", initialCovarianceConfig);
    int matrix_size = parameters.initial_covariance.rows();

    for (int i = 0; i < matrix_size; i++)
    {
      for (int j = 0; j < matrix_size; j++)
      {
        std::ostringstream ostr;
        ostr << initialCovarianceConfig[matrix_size * i + j];
        std::istringstream istr(ostr.str());
        istr >> parameters.initial_covariance(i, j);
      }
    }
  }
  else
  {
    ROS_FATAL("No initial covariance (initial_covariance) set in parameter file!");
    ROS_BREAK();
  }

  return parameters;
}

void setIMUTopicNameFromYaml(std::string& imu_topic_name)
{
  if (ros::param::has("/imu_topic"))
  {
    ros::param::get("/imu_topic", imu_topic_name);
  }
  else
  {
    ROS_WARN("No IMU topic set in yaml file");
  }
}

void setDVLTopicNameFromYawl(std::string& dvl_topic_name)
{
  if (ros::param::has("/dvl_topic"))
  {
    ros::param::get("/dvl_topic", dvl_topic_name);
  }
  else
  {
    ROS_WARN("No DVL topic set in yaml file");
  }
}

void setPressureZTopicNameFromYaml(std::string& pressure_Z_topic_name)
{
  if (ros::param::has("/pressureZ_topic"))
  {
    ros::param::get("/pressureZ_topic", pressure_Z_topic_name);
  }
  else
  {
    ROS_WARN("No PressureZ topic set in yaml file");
  }
}

void setPublishrateFromYaml(int& publish_rate)
{
  if (ros::param::has("/publish_rate"))
  {
    ros::param::get("/publish_rate", publish_rate);
  }
  else
  {
    ROS_WARN("No publish rate set, using default: %i ", publish_rate);
  }
}

void setRdvlFromYamlFile(Matrix3d& R_dvl)
{
  XmlRpc::XmlRpcValue R_dvlConfig;
  if (ros::param::has("/R_dvl"))
  {
    ros::param::get("/R_dvl", R_dvlConfig);
    int matrix_size = R_dvl.rows();

    for (int i = 0; i < matrix_size; i++)
    {
      for (int j = 0; j < matrix_size; j++)
      {
        std::ostringstream ostr;
        ostr << R_dvlConfig[matrix_size * i + j];
        std::istringstream istr(ostr.str());
        istr >> R_dvl(i, j);
      }
    }
  }
  else
  {
    ROS_FATAL("No measurement covariance for DVL (R_dvl) set in parameter file");
    ROS_BREAK();
  }
}

void setRpressureZFromYamlFile(Matrix<double, 1, 1>& R_pressureZ)
{
  XmlRpc::XmlRpcValue R_pressureZConfig;

  if (ros::param::has("/R_pressureZ"))
  {
    ros::param::get("/R_pressureZ", R_pressureZConfig);
    int matrix_size = R_pressureZ.rows();

    for (int i = 0; i < matrix_size; i++)
    {
      std::ostringstream ostr;
      ostr << R_pressureZConfig[i];
      std::istringstream istr(ostr.str());
      istr >> R_pressureZ(i);
    }
  }
  else
  {
    ROS_FATAL("No measurement covariance for pressure sensor (R_pressureZ) set in parameter file!");
    ROS_BREAK();
  }
}





double meanOfVector(const std::vector<double>& vec)
{
  double sum = 0;

  for (auto& each : vec)
    sum += each;

  return sum / vec.size();
}

double maxOfVector(const std::vector<double>& vec)
{
  double max = *std::max_element(vec.begin(), vec.end());

  return max;
}

double stanardDeviationOfVector(const std::vector<double>& vec)
{
  double square_sum_of_difference = 0;
  double mean_var = meanOfVector(vec);
  auto len = vec.size();

  double tmp;
  for (auto& each : vec)
  {
    tmp = each - mean_var;
    square_sum_of_difference += tmp * tmp;
  }

  return std::sqrt(square_sum_of_difference / (len - 1));
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "nlo");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  NLO_Node nlo_node(nh, pnh);
  ros::spin();
  return 0;
}
