#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;
ros::Publisher odom_pub;
MatrixXd Q = MatrixXd::Identity(12, 12); //预测的噪声
MatrixXd Rt = MatrixXd::Identity(6, 6);  //测量的噪声

//定义状态与协方差
VectorXd X_t = VectorXd::Zero(15); //类似于EKF包，
MatrixXd Var_t = MatrixXd::Identity(15, 15);

//Rotation from the camera frame to the IMU frame
Eigen::Matrix3d Rcam;

//时间
double last_time = -1;
//重力加速度
double g = 9.8;

//*********************************************************
//引入 inline 关键字的原因 在 c/c++ 中,为了解决一些频繁调用的小函数大量消耗栈空间(栈内存)的问题,
// 特别的引入了 inline 修饰符,表示为内联函数。
inline Eigen::Vector3d rotationMatrix2EulerVector_zxy(Eigen::Matrix3d R)
{
    Eigen::Vector3d xyz_vector;
    double phi = asinf64(R(2, 1));
    double theta = atan2f64(-R(2, 0), R(2, 2));
    double psi = atan2f64(-R(0, 1), R(1, 1));
    xyz_vector << phi, theta, psi;

    // Eigen::Vector3d zxy_vector = R.eulerAngles(2, 0, 1);
    // Eigen::Vector3d temp;
    // temp << zxy_vector(1), zxy_vector(2), zxy_vector(0);
    // cout << "check eigen euler" << endl;
    // cout << xyz_vector << ";" << temp << endl;
    // xyz_vector <<
    return xyz_vector;
}

inline Eigen::Matrix3d delGinv_delroll(double roll, double pitch)
{
    // \frac{\partial G}{\partial \phi}
    Eigen::Matrix3d deltaG;
    double theta = pitch;
    double phi = roll;
    double cos2_1 = 1.0 / (cos(phi) * cos(phi) + 1e-8);
    deltaG << 0, 0, 0,
        sin(theta) * cos2_1, 0, -cos(theta) * cos2_1,
        -sin(phi) * sin(theta) * cos2_1, 0, cos(theta) * sin(phi) * cos(phi);
    return deltaG;
}

inline Eigen::Matrix3d delGinv_delpitch(double roll, double pitch)
{
    // \frac{\partial G}{\partial \theta}
    Eigen::Matrix3d deltaG;
    double theta = pitch;
    double phi = roll;
    double cos_1 = 1.0 / (cos(phi) + 1e-8);
    deltaG << -sin(theta), 0, cos(theta),
        cos(theta) * sin(phi) * cos_1, 0, sin(theta) * sin(phi) * cos_1,
        -cos(theta) * cos_1, 0, -sin(theta) * cos_1;
    return deltaG;
}

inline Eigen::Matrix3d deltaR_deltaroll(double roll, double pitch, double yaw)
{
    // \frac{\partial R}{\partial \phi}
    Eigen::Matrix3d deltaR;
    double theta = pitch;
    double phi = roll;
    double psi = yaw;

    deltaR << -cos(phi) * sin(psi) * sin(theta), sin(phi) * sin(psi), cos(phi) * cos(theta) * sin(psi),
        cos(phi) * cos(psi) * sin(theta), -cos(psi) * sin(phi), -cos(phi) * cos(psi) * cos(theta),
        sin(phi) * sin(theta), cos(phi), -cos(theta) * sin(phi);

    return deltaR;
}

inline Eigen::Matrix3d deltaR_deltapitch(double roll, double pitch, double yaw)
{
    // \frac{\partial R}{\partial \phi}
    Eigen::Matrix3d deltaR;
    double theta = pitch;
    double phi = roll;
    double psi = yaw;

    deltaR << -cos(psi) * sin(theta) - cos(theta) * sin(phi) * sin(psi), 0, cos(psi) * cos(theta) - sin(phi) * sin(psi) * sin(theta),
        cos(psi) * cos(theta) * sin(phi) - sin(psi) * sin(theta), 0, cos(theta) * sin(psi) + cos(psi) * sin(theta) * sin(phi),
        -cos(phi) * cos(theta), 0, -cos(phi) * sin(theta);
    return deltaR;
}

inline Eigen::Matrix3d deltaR_deltayaw(double roll, double pitch, double yaw)
{
    // \frac{\partial R}{\partial \phi}
    Eigen::Matrix3d deltaR;
    double theta = pitch;
    double phi = roll;
    double psi = yaw;

    deltaR << -cos(theta) * sin(psi) - cos(psi) * sin(phi) * sin(theta), -cos(phi) * cos(psi), cos(psi) * cos(theta) * sin(phi) - sin(psi) * sin(theta),
        cos(psi) * cos(theta) - sin(phi) * sin(psi) * sin(theta), -cos(phi) * sin(psi), cos(psi) * sin(theta) + sin(phi) * sin(psi) * cos(theta),
        0, 0, 0;
    return deltaR;
}
//*********************************************************

void publish_ekf_msg(VectorXd X, std_msgs::Header header)
{

    // Eigen::Vector3d T_c_i;//camera position in the IMU frame
    // T_c_i<<0.05, 0.05, 0;

    // Matrix3d R1;//camera R in the world frame
    // R1=AngleAxisd(X(5),Vector3d::UnitZ())*
    //             AngleAxisd(X(3),Vector3d::UnitX())*
    //             AngleAxisd(X(4),Vector3d::UnitY());
    // //将IMU的结果转到world frame
    // // R1.transposeInPlace();//原地转置
    // Eigen::Matrix3d R_i_w=Rcam*R1;
    // Eigen::Vector3d T_i_w=T_c_i+Rcam*X.head(3);

    // Eigen::Matrix3d R_w_i=R_i_w.transpose();
    // X.segment<3>(3)=rotationMatrix2EulerVector_zxy(R_w_i);
    // X.segment<3>(0)=-R_w_i*T_i_w;

    Eigen::Vector3d T_i_c;
    T_i_c << 0.0, 0.0, 0.0;

    Matrix3d R_1;
    R_1 = AngleAxisd(X(5), Vector3d::UnitZ()) *
          AngleAxisd(X(3), Vector3d::UnitX()) *
          AngleAxisd(X(4), Vector3d::UnitY());
    R_1.transposeInPlace();
    Matrix3d R_cw = Rcam.transpose() * R_1;
    X.segment<3>(3) = rotationMatrix2EulerVector_zxy(R_cw);
    X.segment<3>(0) = Rcam.transpose() * (-R_1 * X.segment<3>(0) - T_i_c);

    nav_msgs::Odometry new_msg;

    double roll = X(3);
    double pitch = X(4);
    double yaw = X(5);
    Matrix3d R; //当前状态对应的旋转矩阵   旋转矩阵（3×3） Eigen::Matrix3d
    R = AngleAxisd(yaw, Vector3d::UnitZ()) *
        AngleAxisd(roll, Vector3d::UnitX()) *
        AngleAxisd(pitch, Vector3d::UnitY());

    Quaternion<double> q;
    q = R;
    new_msg.header = header;
    new_msg.header.frame_id = "world";
    new_msg.pose.pose.position.x = X(0);
    new_msg.pose.pose.position.y = X(1);
    new_msg.pose.pose.position.z = X(2);
    new_msg.pose.pose.orientation.w = q.w();
    new_msg.pose.pose.orientation.x = q.x();
    new_msg.pose.pose.orientation.y = q.y();
    new_msg.pose.pose.orientation.z = q.z();
    new_msg.twist.twist.linear.x = X(6);
    new_msg.twist.twist.linear.y = X(7);
    new_msg.twist.twist.linear.z = X(8);
    new_msg.twist.twist.angular.x = X(9);
    new_msg.twist.twist.angular.y = X(10);
    new_msg.twist.twist.angular.z = X(11);

    //x, y, z,
    //roll, pitch, yaw,
    //vx, vy, vz,
    //vroll, vpitch, vyaw,
    //ax, ay, az
    odom_pub.publish(new_msg);
}

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    //your code for propagation
    //400HZ

    //关于sensor message可以参考http://wiki.ros.org/sensor_msgs
    //而其中的IMU在http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html
    //对于其中的stamp有注释如下：
    // # Two-integer timestamp that is expressed as:
    // # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    // # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    // # time-handling sugar is provided by the client library
    //那么定义的时间应该为：
    double time = msg->header.stamp.sec + msg->header.stamp.nsec * 1e-9; //单位为秒
    double dt = 0;
    if (last_time > 0)
    {
        dt = time - last_time;
    }
    last_time = time;

    //IMU message：http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html
    //定义的消息如下：
    // std_msgs/Header header
    // geometry_msgs/Quaternion orientation
    // float64[9] orientation_covariance
    // geometry_msgs/Vector3 angular_velocity
    // float64[9] angular_velocity_covariance
    // geometry_msgs/Vector3 linear_acceleration
    // float64[9] linear_acceleration_covariance

    Vector3d omega_imu; //角速度
    omega_imu << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    // omega_m=msg->angular_velocity; (没有这个运算符)

    Vector3d a_imu; //线加速度
    a_imu << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

    double roll = X_t(3);
    double pitch = X_t(4);
    double yaw = X_t(5);
    //Rotation Matrix
    Matrix3d R; //当前状态对应的旋转矩阵   旋转矩阵（3×3） Eigen::Matrix3d
    //旋转向量（3×1）Eigen::AngleAxisd
    //AngleAxisd(yaw,Vector3d::UnitZ())初始化旋转向量，角度为yaw，旋转轴为Z
    //**********************
    // 欧拉角转到旋转矩阵的做法：
    // Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle(2),Vector3d::UnitX()));
    // Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle(1),Vector3d::UnitY()));
    // Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle(0),Vector3d::UnitZ()));

    // Eigen::Matrix3d rotation_matrix;
    // rotation_matrix=yawAngle*pitchAngle*rollAngle;
    //**********************
    R = AngleAxisd(yaw, Vector3d::UnitZ()) * AngleAxisd(roll, Vector3d::UnitX()) * AngleAxisd(pitch, Vector3d::UnitY());

    //状态的变化
    VectorXd dotX_t = VectorXd::Zero(15);
    dotX_t.head(3) = X_t.segment<3>(6); //参考：https://blog.csdn.net/shuzfan/article/details/52367329
    //x.head<n>()// x(1:n)
    // x.segment<n>(i)// x(i+1 : i+n)   X_t(6:9)为//vx, vy, vz,

    //x, y, z,
    //roll, pitch, yaw,
    //vx, vy, vz,
    //vroll, vpitch, vyaw,
    //ax, ay, az

    //angular velocity change （角速度的变化）
    //angular velocity in the body frame (IMU) =matrix (G) * world angular velocity
    // /matrix (G)为：
    Matrix3d G, G_inverse;
    G << cos(pitch), 0, -cos(roll) * sin(pitch),
        0, 1, sin(roll),
        sin(pitch), 0, cos(roll) * cos(pitch);

    G_inverse = G.inverse();
    dotX_t.segment<3>(3) = G_inverse * (omega_imu - X_t.segment<3>(9));
    dotX_t.segment<3>(6) = R * (a_imu - X_t.segment<3>(12));
    dotX_t(8) = dotX_t(8) + g;

    //计算状态转移矩阵的雅可比矩阵
    MatrixXd F = MatrixXd::Identity(15, 15);

    // Derivatives for G, R
    MatrixXd dGinv_droll = delGinv_delroll(roll, pitch);
    MatrixXd dGinv_dpitch = delGinv_delpitch(roll, pitch);
    MatrixXd dR_droll = deltaR_deltaroll(roll, pitch, yaw);
    MatrixXd dR_dpitch = deltaR_deltapitch(roll, pitch, yaw);
    MatrixXd dR_dyaw = deltaR_deltayaw(roll, pitch, yaw);

    F.block<3, 3>(0, 6) = MatrixXd::Identity(3, 3) * dt; // position -> speed

    F.block<3, 3>(3, 9) -= G_inverse * dt;                                      // orientation -> bg
    F.block<3, 1>(3, 3) += dGinv_droll * (omega_imu - X_t.segment<3>(9)) * dt;  // orientation -> roll
    F.block<3, 1>(3, 4) += dGinv_dpitch * (omega_imu - X_t.segment<3>(9)) * dt; // orientation -> pitch

    F.block<3, 1>(6, 3) += dR_droll * (a_imu - X_t.segment<3>(12)) * dt; // velocity -> roll/pitch/yaw
    F.block<3, 1>(6, 4) += dR_dpitch * (a_imu - X_t.segment<3>(12)) * dt;
    F.block<3, 1>(6, 5) += dR_dyaw * (a_imu - X_t.segment<3>(12)) * dt;
    F.block<3, 3>(6, 12) -= R * dt;

    MatrixXd U = MatrixXd::Identity(15, 12);
    //P.block<rows, cols>(i, j) // P(i+1 : i+rows, j+1 : j+cols)
    U.block<3, 3>(3, 0) = G_inverse; //U(3+1:3+3. 0+1:0+3)
    U.block<3, 3>(6, 3) = R;
    MatrixXd V = MatrixXd::Identity(15, 12);
    V = U * dt;

    //EKF的状态预测
    X_t = X_t + dt * dotX_t;
    Var_t = F * Var_t * F.transpose() + V * Q * V.transpose();
    //需要将当前的预测X_t发布嘛？？？
    //在此处发布而不在measurement处发布的原因在于此次的频率更高，而measurement会自动更新状态
    publish_ekf_msg(X_t, msg->header);
}

void odom_callback(const geometry_msgs::PoseStamped::ConstPtr &msg) //marker
{
    //your code for update
    // camera position in the IMU frame = (0.05, 0.05, 0)
    // camera orientaion in the IMU frame = Quaternion(0, 1, 0, 0); w x y z, respectively
    //					   RotationMatrix << 1, 0, 0,
    //							             0, -1, 0,
    //                                       0, 0, -1;

    //20 HZ

    //get measurements
    Eigen::Matrix3d R_c_w; //camera在world frame下的旋转矩阵
    Eigen::Vector3d T_c_w;
    //赋值
    T_c_w << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    Eigen::Quaterniond q(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z);
    R_c_w = q;

    //Transform back
    Eigen::Vector3d T_c_i; //camera position in the IMU frame
    T_c_i << 0.0, 0.03, 0.08;

    //IMU在world frame下的pose
    Eigen::Matrix3d R_i_w = Rcam * R_c_w;
    Eigen::Vector3d T_i_w = T_c_i + Rcam * T_c_w;

    Eigen::Matrix3d R_w_i = R_i_w.transpose();
    Eigen::Vector3d euler_meas = rotationMatrix2EulerVector_zxy(R_w_i);
    Eigen::Vector3d T_w_i = -R_w_i * T_i_w;

    VectorXd measurement_different(6);
    measurement_different.head(3) = T_w_i - X_t.head(3);
    measurement_different.segment<3>(3) = euler_meas - X_t.segment<3>(3);
    //对角度进行归一化到-pi~pi区间
    for (int i = 3; i < 6; i++)
    {
        if (measurement_different(i) > M_PI)
        {
            measurement_different(i) = measurement_different(i) - 2 * M_PI;
        }
        else if (measurement_different(i) < -M_PI)
        {
            measurement_different(i) = measurement_different(i) + 2 * M_PI;
        }
    }

    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(6, 15);
    C.block<6, 6>(0, 0) = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd W = Eigen::MatrixXd::Identity(6, 6);

    Eigen::MatrixXd K = Var_t * C.transpose() * (C * Var_t * C.transpose() + W * Rt * W.transpose()).inverse();

    //measuremnet update
    X_t = X_t + K * measurement_different;
    Var_t = Var_t - K * C * Var_t;

    for (int i = 3; i < 6; i++)
    {
        if (X_t(i) > M_PI)
        {
            X_t(i) = X_t(i) - 2 * M_PI;
        }
        else if (X_t(i) < -M_PI)
        {
            X_t(i) = X_t(i) + 2 * M_PI;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");
    ros::Subscriber s1 = n.subscribe("/mavros/imu/data", 1000, imu_callback);       //做预测
    ros::Subscriber s2 = n.subscribe("/mavros/vision_pose/pose", 1000, odom_callback); //做测量更新，虽然称呼为odom，但实际上是基于PNP的3D-2D pose estimation
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 100);
    Rcam = Quaterniond(0, 1, 0, 0).toRotationMatrix(); //camera orientaion in the IMU frame
    cout << "R_cam" << endl
         << Rcam << endl;

    // Q imu covariance matrix; Rt visual odomtry covariance matrix
    // You should also tune these parameters

    // Q.topLeftCorner(6, 6) = 0.01 * Q.topLeftCorner(6, 6);
    // Q.bottomRightCorner(6, 6) = 0.01 * Q.bottomRightCorner(6, 6);
    // Rt.topLeftCorner(3, 3) = 0.1 * Rt.topLeftCorner(3, 3);
    // Rt.bottomRightCorner(3, 3) = 0.1 * Rt.bottomRightCorner(3, 3);
    // Rt.bottomRightCorner(1, 1) = 0.1 * Rt.bottomRightCorner(1, 1);

    Var_t.block<6, 6>(9, 9) = MatrixXd::Identity(6, 6) * 0.01;

    Q.topLeftCorner(6, 6) = 0.1 * Q.topLeftCorner(6, 6);
    Q.bottomRightCorner(6, 6) = 0.005 * Q.bottomRightCorner(6, 6);
    Rt.topLeftCorner(3, 3) = 0.1 * Rt.topLeftCorner(3, 3);
    Rt.bottomRightCorner(3, 3) = 0.1 * Rt.bottomRightCorner(3, 3);
    Rt.bottomRightCorner(1, 1) = 0.1 * Rt.bottomRightCorner(1, 1);

    ros::spin();
}