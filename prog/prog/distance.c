#include<stdio.h>
#include<math.h>
#include<string.h>

void calcdis(FILE* fp,STATE state,double x,double y,int nS,int i){
  double dist;
  double x2;
  double y2;
  
  x2=x-(state.pos.x/1000.0);
  y2=y-(state.pos.y/1000.0);
  x2=pow(x2,2);
  y2=pow(y2,2);
  dist=x2+y2;
  dist=sqrt(dist); 

  fprintf(fp,"%.3f %d %d\n",dist,nS,i);
}
