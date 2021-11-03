#include <iostream>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>

#define MATRIX_SIZE 50


int main(int argc, char **argv)
{
    clock_t time_stt; 

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matrix_100 = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Random(100, 100);
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> v_100 = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Random(100, 1);
    time_stt = clock();
    Eigen::Matrix<float, 100, 1> x_100;
    x_100 = matrix_100.colPivHouseholderQr().solve(v_100);
    std::cout << "Qr decomposition time " << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << std::endl;

    time_stt = clock();
    x_100 = matrix_100.llt().solve(v_100);
    std::cout << "LLT decomposition time " << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << std::endl;



    Eigen::Quaterniond q1(0.55, 0.3, 0.2, 0.2);
    q1.normalize();
    Eigen::Vector3d t1(0.7, 1.1, 0.2);
    Eigen::Quaterniond q2(-0.1, 0.3, -0.7, 0.2);
    
    q2.normalize();
    Eigen::Vector3d t2(-0.1, 0.4, 0.8);
    Eigen::Vector3d p1(0.5, -0.1, 0.2);

    Eigen::Matrix4d T_c1_w = Eigen::Matrix4d::Identity();
    T_c1_w.block<3,3>(0,0) = q1.normalized().toRotationMatrix();
    T_c1_w.block<3,1>(0,3) = t1;

    Eigen::Matrix4d T_c2_w = Eigen::Matrix4d::Identity();
    T_c2_w.block<3,3>(0,0) = q2.normalized().toRotationMatrix();
    T_c2_w.block<3,1>(0,3) = t2;

    Eigen::Vector4d p(p1.x(), p1.y(), p1.z(), 1);
    auto p_w = T_c1_w.inverse() * p;
    auto p_c2 = T_c2_w * p_w;

    std::cout<<"P2 : "<<p_c2.block<3,1>(0,0).transpose()<<std::endl;
    return 0;
}
