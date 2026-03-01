#include <Eigen/Dense>
#include <iostream>
 

using namespace Eigen;
using namespace std;

int main(int argc, char** argv) {
  //不能用auto，因为Eigen的MatrixXd是一个模板类
  //如果用auto，编译器会推导出一个具体的类型，而不是MatrixXd
  //需要显式指定类型
  MatrixXd A = MatrixXd::Random(100, 100);
  A = A * A.transpose(); //使A为对称正定矩阵
  //QR分解
  MatrixXd b = MatrixXd::Random(100, 1);
  //求解Ax=b
  cout << "QR分解" << endl;
  MatrixXd x = A.colPivHouseholderQr().solve(b);
  //输出x
  cout << "x:" << x.transpose() << endl;
  cout << "Cholesky分解" << endl;
  //Cholesky分解
  MatrixXd y = A.llt().solve(b);
  //输出y
  cout << "y:" << y.transpose() << endl;

return 0;
}