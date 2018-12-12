#include<stdio.h>
#include<math.h>
#include<string.h>

double vecabs(double vec[2]){
  return pow(vec[0]*vec[0]+vec[1]*vec[1],0.5);
}
double dot(double vec[2],double vec2[2]){
  return vec[0]*vec2[0]+vec[1]*vec2[1];
}
double cross(double vec[2],double vec2[2]){
  return vec[0]*vec2[1]-vec[1]*vec2[0];
}
double distance(double vec[2],double vec2[2]){
  return (fabs(cross(vec,vec2)))/vecabs(vec2);
}

double distance_ls(double point[2],double point1[2],double point2[2]){
  double vec[2],vec2[2];
  vec[0]=point[0]-point1[0];//ac
  vec[1]=point[1]-point1[1];//ac

  vec2[0]=point2[0]-point1[0];//ab
  vec2[1]=point2[1]-point1[1];//ab
  
  if(dot(vec,vec2)<0.0) return vecabs(vec);
  else if(dot(vec,vec2)==0.0) return 0.0;
  vec[0]=point[0]-point2[0];//bc
  vec[1]=point[1]-point2[1];

  vec2[0]=point1[0]-point2[0];//ba
  vec2[1]=point1[1]-point2[1];
  
  if(dot(vec,vec2)<0.0) return vecabs(vec);
  
  return distance(vec,vec2);
}
double dis_min(double *xta,double *yta,double point[1][2],int i){
  double point3[2],point1[2],point2[2];
  double dis[2];
  double dismin = 100;
  
  point3[0]=point[0][0];
  point3[1]=point[0][1];
  int j;
  for(j=0;j<i-1;j++){
    point1[0]=xta[j];
    point1[1]=yta[j];
    point2[0]=xta[j+1];
    point2[1]=yta[j+1];
    dis[j%2]=distance_ls(point3,point1,point2);
    //printf("dismin=%lf\n\n",dis[j%2]);
    if(j>0){
      if(dis[j%2]<dismin) dismin=dis[j%2];
    }else{
      dismin=dis[j%2];
    }
  }  
  //printf("dismin=%lf\n\n",dismin);
//exit(0);
  return dismin;
}
double calcalfa_k3d(FILEDATA *F,double *xt,double *yt,double x,double y,double z){
  int n;
  double dmin=1e10;
  for(n=0;n<F->ndata-1;n++){
    double dx0=x-xt[n];
    double dy0=y-yt[n];
    double dz0=z-0;
    double dx1=x-xt[n+1];
    double dy1=y-yt[n+1];
    double dz1=z-0;
    double dx2=xt[n+1]-xt[n];
    double dy2=yt[n+1]-yt[n];
    double dz2=0;
    double n0=sqrt(dx0*dx0+dy0*dy0+dz0*dz0);
    double n1=sqrt(dx1*dx1+dy1*dy1+dz1*dz1);
    double n2=sqrt(dx2*dx2+dy2*dy2+dz2*dz2);
    double dx0dx2=dx0*dx2+dy0*dy2+dz0*dz2;
    double cost=dx0dx2/(n0*n2);
    double d;
    if(n0*cost>n2){
      d=n0;
    }
    else if(cost<0){
      d=n1;
    }
    else{
      d=n0*sqrt(1.-cost*cost);
    }
    if(d<dmin) dmin=d;
  }
  return dmin;
}
double calcalfa_k2d(FILEDATA *F,double *xt,double *yt,double x,double y){
  int n;
  double dmin=1e10;
  for(n=0;n<F->ndata-1;n++){
    double dx0=x-xt[n];
    double dy0=y-yt[n];
    double dx1=x-xt[n+1];
    double dy1=y-yt[n+1];
    double dx2=xt[n+1]-xt[n];
    double dy2=yt[n+1]-yt[n];
    double n0=sqrt(dx0*dx0+dy0*dy0);
    double n1=sqrt(dx1*dx1+dy1*dy1);
    double n2=sqrt(dx2*dx2+dy2*dy2);
    double x_y=dx0*dx2+dy0*dy2;
    double cost=x_y/(n0*n2);
    double d;
    if(n0*cost>n2){
      d=n0;
    }
    else if(cost<0){
      d=n1;
    }
    else{
      d=n0*sqrt(1.-cost*cost);
    }
    if(d<dmin) dmin=d;
  }
  return dmin;
}

double calcalfa(FILEDATA *F,double *xt,double *yt,double x_pos,double y_pos){
  double point[1][2];//c,a,b
  double *xta;
  double *yta;
  xta = (double *)malloc(sizeof(double)*F->ndata);
  yta = (double *)malloc(sizeof(double)*F->ndata);
  int n;
  
  for(n=0;n<F->ndata;n++){
    xta[n]=xt[n];
    yta[n]=yt[n];
  }
  
  double dis;
  point[0][0] = x_pos;
  point[0][1] = y_pos;
  dis=dis_min(xta,yta,point,F->ndata);
  free(xta);
  free(yta);
  //printf("dis=%lf\n\n",dis);
  return dis;
}
