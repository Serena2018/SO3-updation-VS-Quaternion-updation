//
// Created by tp on 6/16/19.
//

#include <iostream>
#include <sophus/so3.h>
using namespace std;

int main()
{
    //先定义一个沿Z轴旋转90度的初始旋转矩阵
    Eigen::Matrix3d R0 = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,1,0)).toRotationMatrix();
    //将这个初始矩阵转换为SO3的形式
    Sophus::SO3 SO3_R(R0);

    std::cout<<"R0 in matrix form: \n"<<R0<<std::endl;
    std::cout<<"SO3_R: \n"<<SO3_R.matrix()<<std::endl;
    cout<<"\n***********************************************\n"<<endl;

    //给出位姿变换微小增量
    Eigen::Vector3d update_so3(0.01,0.02,0.03);
    std::cout<<"update_so3 : \n"<<update_so3.transpose()<<std::endl;
    cout<<"\n***********************************************\n"<<endl;


    //从初始位姿经过一次微小的变换之后得到的新的位姿
    Sophus::SO3 SO3_updated = Sophus::SO3::exp(update_so3)*SO3_R;
    std::cout<<" SO3 updated: \n"<<SO3_updated.matrix()<<std::endl;
    cout<<"\n***********************************************\n"<<endl;

    //将初始位姿用四元数形式表示
    Eigen::Quaterniond q0(R0);
    ///输出顺序为 x y z w
    cout<<"coefficients in q0: \n"<<q0.coeffs().transpose()<<endl;
    cout<<"\n***********************************************\n"<<endl;

    //给出的微小位姿变换是旋转向量的形式 （0.01， 0.02， 0.03）
    // 首先将其表示成轴角的形式，
    // 旋转向量的方向为轴的方向，
    // 长度为旋转的角度
    Eigen::Vector3d rv_norm = Eigen::Vector3d(0.01/sqrt(0.01*0.01+0.02*0.02+0.03*0.03),0.02/sqrt(0.01*0.01+0.02*0.02+0.03*0.03),0.03/sqrt(0.01*0.01+0.02*0.02+0.03*0.03));
    std::cout<<"norm: \n"<<rv_norm[0]<<" "<<rv_norm[1]<<" "<<rv_norm[2]<<std::endl;
    cout<<"\n***********************************************\n"<<endl;


    double rv_angle = sqrt(0.01*0.01+0.02*0.02+0.03*0.03);
    std::cout<<"rv_angle: \n"<<rv_angle<<std::endl;
    cout<<"\n***********************************************\n"<<endl;

    //由轴角形式构建对应的四元数
    Eigen::AngleAxisd updated_AngleAxisd(rv_angle, Eigen::Vector3d(rv_norm[0],rv_norm[1],rv_norm[2]));
    Eigen::Quaterniond update_q = Eigen::Quaterniond(updated_AngleAxisd);
    std::cout<<"coefficients in update_q: \n"<<update_q.coeffs().transpose()<<endl;
    cout<<"\n***********************************************\n"<<endl;

    //接下来计算两个四元数的乘法
    double q_new_w = q0.w()*update_q.w() - q0.x()*update_q.x() - q0.y()*update_q.y() - q0.z()*update_q.z();
    double q_new_x = q0.w()*update_q.x() + q0.x()*update_q.w() + q0.y()*update_q.z() - q0.z()*update_q.y();
    double q_new_y = q0.w()*update_q.y() - q0.x()*update_q.z() + q0.y()*update_q.w() + q0.z()*update_q.x();
    double q_new_z = q0.w()*update_q.z() + q0.x()*update_q.y() - q0.y()*update_q.x() + q0.z()*update_q.w();

    Eigen::Quaterniond q_updated(q_new_w,q_new_x,q_new_y, q_new_z);
    //只有单位四元数才可以用来表示旋转矩阵
    Eigen::Quaterniond q_updated_norm = q_updated.normalized();
    Eigen::Matrix3d SO3_from_q = q_updated_norm.toRotationMatrix();

    cout<<"coefficients in q0: \n"<<q0.coeffs().transpose()<<endl;
    cout<<"\n***********************************************\n"<<endl;

    std::cout<<"coefficients in update_q: \n"<<update_q.coeffs().transpose()<<endl;
    cout<<"\n***********************************************\n"<<endl;

    cout<<" coeffs in q_updated: \n"<<q_updated.coeffs().transpose()<<endl;
    cout<<"\n***********************************************\n"<<endl;

    cout<<"通过 R<-R0exp(w) 更新方式得到的新的位姿：\n"<<endl;
    std::cout<<" SO3_from_R: \n"<<SO3_updated.matrix()<<std::endl;
    cout<<"\n***********************************************\n"<<endl;

    cout<<"通过 q<-q0|+|w 更新方式得到的新的位姿：\n"<<endl;
    cout<<"SO3_from_q: \n"<<q_updated_norm.matrix()<<endl;
    cout<<"\n***********************************************\n"<<endl;







}
