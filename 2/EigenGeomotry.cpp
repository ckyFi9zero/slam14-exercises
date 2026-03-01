#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <iostream>
#include <vector>   

using namespace Eigen;
using namespace std;


int main(int argc, char** argv) {

    Quaterniond q1 = Quaterniond(0.55, 0.3, 0.2, 0.2); // 小萝卜1的位姿
    Vector3d t1(0.7, 1.1, 0.2); // 小萝卜1的平移向量
    Quaterniond q2 = Quaterniond(-0.1, 0.3, -0.7, 0.2); // 小萝卜2的位姿
    Vector3d t2(-0.1, 0.4, 0.8); // 小萝卜2的平移向量
    //归一化
    q1.normalize();
    q2.normalize();

    Vector3d p1(0.5, -0.1, 0.2); // 小萝卜1坐标系下的p点

    //求小萝卜2坐标系下的p点坐标
    //pw = Tw2 * p2 = Tw1 * p1
    //p2 = Tw2.inverse() * Tw1 * p1

    //欧氏变换矩阵
    // Isometry3d T1w = Isometry3d::Identity();
    // T1w.rotate(q1);// 可以是 1.四元数 2.旋转轴+角度 3.旋转矩阵
    // T1w.pretranslate(t1);// 平移向量

    // Isometry3d T2w = Isometry3d::Identity();
    // T2w.rotate(q2);// 可以是 1.四元数 2.旋转轴+角度 3.旋转矩阵
    // T2w.pretranslate(t2);// 平移向量


    //将q1转换为旋转矩阵
    Matrix3d R1 = q1.toRotationMatrix();
    //对R1求转置
    Matrix3d R1t = R1.transpose();
    //计算平移部分
    Vector3d t1_translated = -R1t * t1;
    Isometry3d Tw1 = Isometry3d::Identity();
    Tw1.rotate(R1t);
    Tw1.pretranslate(t1_translated);


    //将q2转换为旋转矩阵
    Matrix3d R2 = q2.toRotationMatrix();
    //对R2求转置
    Matrix3d R2t = R2.transpose();
    //计算平移部分
    Vector3d t2_translated = -R2t * t2;
    Isometry3d Tw2 = Isometry3d::Identity();
    Tw2.rotate(R2t);
    Tw2.pretranslate(t2_translated);


    //求小萝卜2坐标系下的p点坐标
    Vector3d p2 = Tw2.inverse() * Tw1 * p1;


    cout << "p2:" << p2.transpose() << endl;
    return 0;
}