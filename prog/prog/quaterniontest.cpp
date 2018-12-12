#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry> //Eigen��Geometry��Ϣ�δؿ���Ȥ���硤���줬ɬ��

int main() {
  using namespace Eigen;

  // �ѿ���������ͤ�̤���
  Quaternionf q1;

  // ���󥹥ȥ饯������(w, x, y, z)���Ϥ��ƽ����
  Quaternionf q2(1.0f, 0.0f, 0.25f, 0.5f);
  Quaternionf q2(0.000000, 0.707107, -0.707107, 0.000000);
  std::cout << "q2(w,x,y,z):"<< q2.w() << "," << q2.x() << "," << q2.y() << "," << q2.z() << std::endl;
  std::cout << "q2.norm:" << q2.norm() << std:endl;
  // ���󥹥ȥ饯���ǳ��٤ȥ٥��ȥ���Ϥ��ƽ����
  Quaternionf q3(AngleAxisf(0.1f, Vector3f::UnitY()));

  // ���ĤΥ٥��ȥ뤫�饯�������˥�������
  Quaternionf q4 = Quaternionf::FromTwoVectors(Vector3f::UnitX(), Vector3f::UnitZ());

  // ñ�̥��������˥���(w��1��¾��0)
  q4 = Quaternionf::Identity();
  std::cout << q4.w() << "," << q4.x() << "," << q4.y() << "," << q4.z() << std::endl;

  // �軻
  Quaternionf q_mul = q2 * q3;

  // �ե��������˥���
  Quaternionf q_inv = q4.inverse();

  // ���򥯥������˥�������
  Quaternionf q_conj = q4.conjugate();

  // ����
  float dot = q3.dot(q4);

  // ��ž�٥��ȥ��Ĺ��
  //  float norm = q3.norm();
  float norm = q2.norm();

  // ������
  q3.normalize();
  Quaternionf q_normalized = q4.normalized();

  // �����������
  // q3��q4�� t[0, 1.0] ����֤���
  float t = 0.5f;
  Quaternionf q_slerp = q3.slerp(t, q4);
}
#ifdef ATODE
int main() {
  using namespace Eigen;

// �ѿ���������ͤ�̤���
  Quaternionf q1;

  // ���󥹥ȥ饯������(w, x, y, z)���Ϥ��ƽ����
  Quaternionf q2(1.0f, 0.0f, 0.25f, 0.5f);
  Quaternionf qq(0.000000, 0.707107, -0.707107, 0.000000)
  std::cout << q2.w() << "," << q2.x() << "," << q2.y() << "," << q2.z() << std::endl;

  // ���󥹥ȥ饯���ǳ��٤ȥ٥��ȥ���Ϥ��ƽ����
  Quaternionf q3(AngleAxisf(0.1f, Vector3f::UnitY()));

  // ���ĤΥ٥��ȥ뤫�饯�������˥�������
  Quaternionf q4 = Quaternionf::FromTwoVectors(Vector3f::UnitX(), Vector3f::UnitZ());
#endif
