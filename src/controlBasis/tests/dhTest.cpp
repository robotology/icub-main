#include <ControlBasisResource.h>
#include <ForwardKinematics.h>

using namespace KIN;

int main(int argc, char *argv[]) {

  Matrix T1(4,4);
  Matrix T2(4,4);
  Matrix T3(4,4);

  ForwardKinematics *fk = new ForwardKinematics();

  double a1,a2,d1,d2,alpha1,alpha2,theta1,theta2;

  a1 = 0;
  a2 = 0;
  d1 = 0;
  d2 = 0;

  alpha1 = 90*TORAD;
  alpha2 = 0;
  theta1 = 0;
  theta2 = 90*TORAD;;

  fk->GetHomogeneousTransform(alpha1, a1, d1, theta1, T1);
  fk->GetHomogeneousTransform(alpha2, a2, d2, theta2, T2);

  T3 = T1*T2;

  printf("T1:\n");
  for(int i=0; i<4; i++) {
    printf("%.3f  %.3f  %.3f  %.3f\n", T1(i,0),T1(i,1),T1(i,2),T1(i,3));
  }

  printf("T2:\n");
  for(int i=0; i<4; i++) {
    printf("%.3f  %.3f  %.3f  %.3f\n", T2(i,0),T2(i,1),T2(i,2),T2(i,3));
  }

  printf("T3:\n");
  for(int i=0; i<4; i++) {
    printf("%.3f  %.3f  %.3f  %.3f\n", T3(i,0),T3(i,1),T3(i,2),T3(i,3));
  }

  delete fk;
  return 1;
}
