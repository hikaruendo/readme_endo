/*
FileName: GeometrySample.cpp
Discription: EigenのGeometry関連の関数のサンプル 
Author: Atsushi Sakai
Update: 2013/03/16
*/

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry> //EigenのGeometry関連の関数を使う場合，これが必要

//データ表示用マクロ
#define PRINT_MAT(X) cout << #X << ":\n" << X << endl << endl

using namespace std;
using namespace Eigen;

void Sample1(){
  cout<<"====1. 2次元空間における回転行列を使用した点の回転====="<<endl;
  //変換したい点を作成
  VectorXf point_in(2), point_out(2);
  point_in<<1,1;
  PRINT_MAT(point_in);

  //回転行列の作成
  Matrix2f rot;
  rot=Rotation2Df(M_PI);  //90度反時計回りに回転
  PRINT_MAT(rot);

  //回転行列をかけて回転
  point_out=rot*point_in;
  PRINT_MAT(point_out);

}

void Sample2(){
  cout<<"====2. 3次元空間における軸回転関数を使用した点の回転====="<<endl;
  //変換したい点を作成
  VectorXf point_in_3d(3), point_out_3d(3);
  point_in_3d<<1,1,1; //[x y z]
  PRINT_MAT(point_in_3d);

  //ある軸に対する回転行列を作成
  Matrix3f AxisAngle;
  Vector3f axis;
  axis<<0,0,1;  //z軸を指定
  AxisAngle=AngleAxisf(M_PI,axis);  //Z軸周りに90度反時計回りに回転
  PRINT_MAT(AxisAngle);

  //回転行列をかけて回転
  point_out_3d=AxisAngle*point_in_3d;
  PRINT_MAT(point_out_3d);

}

void Sample3(){
  cout<<"====3. クォータニオンを使用した点の回転====="<<endl;
  //変換したい点を作成
  VectorXf point_in_3d(3), point_out_3d(3);
  point_in_3d<<1,1,1; //[x y z]
  PRINT_MAT(point_in_3d);

  //ある軸に対する回転のクォータニオンを作成
  Quaternionf quat;
  Vector3f axis;
  axis<<0,1,0;  //Y軸を指定 --> -1,1,-1
  axis<<0,2,0;  //Y軸を指定
  quat=AngleAxisf(M_PI,axis);  //Y軸周りに90度反時計回りに回転
  //  std::cout << "q2.Angle:" << q2.angle() << std::endl;
  //PRINT_MAT(quat);    //Quaternion型はcoutで出力できない

  //クォータニオンを回転行列に変換する方法
  MatrixXf t;
  t=quat.matrix();
  PRINT_MAT(t);    //Quaternion型はcoutで出力できない
  //回転行列をかけて回転
  point_out_3d=quat*point_in_3d;
  PRINT_MAT(point_out_3d);
  std::cout << "quat.norm:" << quat.norm() << std::endl;
  std::cout << "quat(w,x,y,z):" << quat.w() << "," << quat.x() << "," << quat.y() << "," << quat.z() <<std::endl;
}

void Sample4(){
  cout<<"====4. 3次元ベクトルのスケール変更====="<<endl;
  //変換したい点を作成
  VectorXf point_in_3d(3), point_out_3d(3);
  point_in_3d<<1,1,2; //[x y z]
  PRINT_MAT(point_in_3d);

  point_out_3d=2.0*point_in_3d; //スケールを変える時には定数をかければOK
  //point_out_3d=Scaling(2.0)*point_in_3d; //これでもOK
  PRINT_MAT(point_out_3d);

}

void Sample5(){
  cout<<"====5. 逆変換====="<<endl;
  //変換したい点を作成
  VectorXf point_in_3d(3), point_out_3d(3);
  point_in_3d<<1,1,2; //[x y z]
  PRINT_MAT(point_in_3d);

  //ある軸に対する回転行列を作成
  Matrix3f AxisAngle;
  Vector3f axis;
  axis<<1,0,0;  //x軸を指定
  AxisAngle=AngleAxisf(M_PI,axis);  //x軸周りに90度反時計回りに回転
  PRINT_MAT(AxisAngle);

  //回転行列をかけて回転
  point_out_3d=AxisAngle*point_in_3d;
  PRINT_MAT(point_out_3d);

  //回転行列の逆行列からもとに戻す
  point_out_3d=AxisAngle.inverse()*point_out_3d;
  PRINT_MAT(point_out_3d);

  Quaternionf q2(1.000000, 0.707107, -0.707107,0.000000);
  q2.w()=-0.000300;
  q2.x()=0.708333;
  q2.y()=-0.705878;
  q2.z()=-0.000281;
  //  q2<<0.003496,-0.705992,0.708191,0.005318;
  std::cout << "q2(w,x,y,z):" << q2.w() << "," << q2.x() << "," << q2.y() << "," << q2.z() << q2.norm()<<std::endl;
  std::cout << "q2.norm:" << q2.norm() << std::endl;
  std::cout << "q2.norm1:" << sqrt(q2.w()*q2.w()+q2.x()*q2.x()+q2.y()*q2.y()+q2.z()*q2.z()) << std::endl;

}

int main()
{
  cout<<"Eigen Geometry Sample Code"<<endl;

  Sample1();
  Sample2();
  Sample3();
  Sample4();
  Sample5();
  
}

