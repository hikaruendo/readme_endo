///////////////////////////////////////////////////
//         OFFLINE SLAM PROGRAM
//
//         2018.01 Hikaru Endo
//
///////////////////////////////////////////////////
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <cv.h>       // OpenCV
#include <highgui.h>  // OpenCV
#include <sys/types.h> //for opendirgps.
#include <dirent.h>   //for opendir
#include <string.h>
#undef PI
#define PI 3.141592653589793
double PI2=6.28318530717959; //PI*2;
double PI_2=1.5707963267949;//PI/2;
double D2R=0.0174532925199433; //(PI/180.);
double R2D=57.2957795130823; //(180./PI);
double m2mm=1000;
int DEBUG=0;
#define buffsize 256
//  北緯 2*6378137*PI/360;
//  東経 cos(PI*33.9/180)*40075017/360;
double GPSy=111319.490793274;//(6378137*D2R);
double GPSx=92396.5456539047;//(cos(33.9*D2R)*40075017/360);
int NofParticles=100;
int input_case=0;
//#define NofParticles 400//400//200//100//300 
#define lrf_data_num 750
#define MAPSIZE_W 800  //normal map size
#define MAPSIZE_H 800 
//#define MAPSIZE_W 500  //normal map size
//#define MAPSIZE_H 500 
//#define MAPSIZE_W 300  //small map size 
//#define MAPSIZE_H 200 
#define NofParam 7 
#define CUT 20.0
#define FSETP 0.5
#define MBSETP 0.5
#define EESETP 0.5
#define LRFH 510.0
#define YSTD 1500.0
#define RSTD 10.0
#define ANG 0
double DELTA=0.1;
//#define PFMAP //coment out for execution without PF
int PFMAP=1;//1 for use PF, 0 for without PF
//#define LMETHOD 2 //1 for max-likelihood, 2 for mean 
int LMETHOD=2;
#define MMETHOD 1 //1 for ? 
double A1=0.002; //v_sigma=A1*fabs(input.v);
double A2=3.5; // w_sigma=A2*fabs(input.omega);
double A3=0.01;
double A4=0.01;

double sigma_v=1e-6;
double sigma_w=1e-6;
double sigma_g=1e-6;
double sigma_x=1e-6;
double sigma_y=1e-6;
double sigma_theta=1e-6;

#define PERCENT_RE 0.3

#define ZMTRAND
//#undef ZMTRAND
#ifdef ZMTRAND
#include "share/zmtrand.c"
#endif
//int robotorientation0=0;
int DISP=0;
#include "_slam.h"
#include "_slam.c"
int main(int argc, char *argv[]){
  //コマンドライン引数の1つ目を"-help"とするとヘルプが表示される
  if(strncmp(argv[1],"-help",5)==0 || argc<7) {
    help();
    return -1;
  }
  //初期姿勢をセット
  //  STATE offset=setInitdrawpos(argv);//
  PARAM par[1];
  par->nZ=0;
  par->robotang0=0;
  par->kinectang0=0;
  par->DELTA=0;
  par->gpsA=-1000.0;
  par->orsA=-1000.0;
  par->kinectA=-1000.0;
  par->gps_theta=-1000.0;
  par->ft=0;
  double tsnap=-1;
  //  STATE offset=par->offset;
  int i;
  for(i=0;i<argc;i++){
    if(strncmp(argv[i],"NP:",3)==0){
      sscanf(&argv[i][3],"%d",&NofParticles);
      if(NofParticles<1) NofParticles=1;
    }
    else if(strncmp(argv[i],"DISP:",5)==0){
      sscanf(&argv[i][5],"%d",&DISP);
    }
    else if(strncmp(argv[i],"usePF:",6)==0){
      if(argv[i][6]=='0') PFMAP=0;
      else PFMAP=1;
    }    
    else if(strncmp(argv[i],"tsnap:",6)==0){
      sscanf(&argv[i][6],"%lf",&tsnap);
    }    
    else if(strncmp(argv[i],"sigma:",6)==0){
      sscanf(&argv[i][6],"%lf:%lf:%lf",&sigma_v,&sigma_w,&sigma_g);
    }
    else if(strncmp(argv[i],"x0:",3)==0){
      sscanf(&argv[i][3],"%lf:%lf:%lf",&par->offset.pos.x,&par->offset.pos.y,&par->offset.theta);
      par->offset.theta=0;
    }
    else if(strncmp(argv[i],"X0:",3)==0){
      sscanf(&argv[i][3],"%lf:%lf:%lf",&par->offset.pos.x,&par->offset.pos.y,&par->offset.theta);
    }
    else if(strncmp(argv[i],"d:",2)==0){
      char *p=par->dir=&argv[i][2];
      //remove last slash of dir (argv[1]
      for(;;p++){if(*p==0) break;}
      for(p--;;p--){if(*p=='/') *p=0;else break;}
    }
    else if(strncmp(argv[i],"fz:",3)==0){//fz=GPS
      FILEDATA *Z=&(par->Z[0]);
      FILEDATA *T=&(par->T[0]);
      sscanf(&argv[i][3],"%d:%lf",&(Z->class),&(Z->T));//class=1:GPS,2:ORS
      char *p=&argv[i][4];
      for(;;p++) if(*p==':') {p++;break;}
      for(;;p++) if(*p==':') {p++;break;}
      sprintf(Z->fn,"%s/%s",par->dir,p);//y
      sprintf(T->fn,"%s/%s",par->dir,p);
      par->nZ++;
    }

    else if(strncmp(argv[i],"fz1:",4)==0){//fz1=ORS
      FILEDATA *Z1=&(par->Z[1]);
      sscanf(&argv[i][4],"%d:%lf",&(Z1->class),&(Z1->T));
      char *p=&argv[i][5];
      for(;;p++) if(*p==':') {p++;break;}
      for(;;p++) if(*p==':') {p++;break;}
      sprintf(Z1->fn,"%s/%s",par->dir,p);
      par->nZ++;
    }
    else if(strncmp(argv[i],"ft:",3)==0){//理想軌道
      FILEDATA *F=&(par->F[0]);
      char *p=&argv[i][3];
      sprintf(F->fn,"%s/%s",par->dir,p);
      par->ft=1;
    }

    else if(strncmp(argv[i],"fU:",3)==0){
      FILEDATA *U=&(par->U[0]);
      sscanf(&argv[i][3],"%d:%lf:%lf",&(U->class),&(U->T),&(U->a0));
      par->robotang0=U->a0;
      char *p=&argv[i][4];
      for(;;p++) if(*p==':') {p++;break;}
      for(;;p++) if(*p==':') {p++;break;}
      for(;;p++) if(*p==':') {p++;break;}
      sprintf(U->fn,"%s/%s",par->dir,p);
    }


    /*************kinect追加***************/


    else if(strncmp(argv[i],"fK:",3)==0){
      FILEDATA *K=&(par->K[0]);
      sscanf(&argv[i][3],"%d:%lf:%lf",&(K->class),&(K->T),&(K->k0));
      par->kinectang0=K->k0;
      char *p=&argv[i][4];
      for(;;p++) if(*p==':') {p++;break;}
      for(;;p++) if(*p==':') {p++;break;}
      for(;;p++) if(*p==':') {p++;break;}
      sprintf(K->fn,"%s/%s",par->dir,p);
    }


    /*************kinect追加***************/

    else if(strncmp(argv[i],"fu:",3)==0){
      FILEDATA *U=&(par->U[0]);
      sscanf(&argv[i][3],"%d:%lf",&(U->class),&(U->T));
      par->robotang0=U->a0=90;
      char *p=&argv[i][4];
      for(;;p++) if(*p==':') {p++;break;}
      for(;;p++) if(*p==':') {p++;break;}
      sprintf(U->fn,"%s/%s",par->dir,p);
      //  printf("par->robotang0=%lf\n",par->robotang0);
    }
    else if(strncmp(argv[i],"res:",4)==0){
      sscanf(&argv[i][4],"%d:%d",&par->MATCHres,&par->MAPres);
    }
    else if(strncmp(argv[i],"DT:",3)==0){
      sscanf(&argv[i][3],"%lf",&par->DELTA);
    }
    else if(strncmp(argv[i],"gpsA:",5)==0){
      sscanf(&argv[i][5],"%lf",&par->gpsA);
    }
    else if(strncmp(argv[i],"orsA:",5)==0){
      sscanf(&argv[i][5],"%lf",&par->orsA);
    }
    else if(strncmp(argv[i],"gps_theta:",10)==0){
      sscanf(&argv[i][10],"%lf",&par->gps_theta);
    }
    else if(strncmp(argv[i],"METHOD:",7)==0){
      sscanf(&argv[i][7],"%d",&LMETHOD);
    }
    else if(strncmp(argv[i],"input:",6)==0){
      sscanf(&argv[i][6],"%d",&input_case);
    }
     else if(strncmp(argv[i],"xysigma:",8)==0){
      sscanf(&argv[i][8],"%lf:%lf:%lf",&sigma_x,&sigma_y,&sigma_theta);
    }
  }
  //printf("par->robotang0=%lf\n",par->robotang0);
  //exit(0);

  PARTICLE *pos_k=(PARTICLE*)malloc(sizeof(PARTICLE)*NofParticles);//状態パーティクル
  PARTICLE *pos_predict=(PARTICLE*)malloc(sizeof(PARTICLE)*NofParticles);//状態予測パーティクル
  PARTICLE *pos_resample=(PARTICLE*)malloc(sizeof(PARTICLE)*NofParticles);//リサンプル後パーティクルS

#define useLRF
#ifdef useLRF
  SCAN scan_k[lrf_data_num]; //スキャン平面(2D)
  SCAN scan_kplus1[lrf_data_num]; //スキャン平面(2D)
  SCAN lrf_k[lrf_data_num]; //LRF座標系(3D)
  SCAN lrf_kplus1[lrf_data_num];   //次時刻計測のLRF座標系(3D)
  //Number of parameter check
//  if(argc!=NofParam){
//    printf("ERROR:THE NUMBER OF PARAMETER IS INCOLLECT\n");
//    exit(1);
//  }
  //設定内容を確認，実行許可を得る//file open?
    int totaltime=makesuretouser(par);//in _slam.c open par->ACTfp 
    //    int totaltime=makesuretouser(argv);//in _slam.c
#endif //#ifdef useLRF

  //////////////////////
  //SLAM フロー開始
  //////////////////////
  printf("FLOW START!!\n");
  FILEDATA *U=&(par->U[0]);
  rewind(U->fp);  U->l=0;  U->timeA=0;//  U->a0=-1000;////for initialization?
  FILEDATA *T=&(par->T[0]);
  if((T->fp=fopen(T->fn,"r"))==NULL){
    printf("\n\nERROR:CAN'T OPEN FILE (%s) :CHECK THE argv[1]\n",T->fn);
    exit(-1);
  }
  rewind(T->fp);

  ////////////////////理想軌道追加////////////////
  FILEDATA *F=&(par->F[0]);  
  if(((F->fp=fopen(F->fn,"r"))==NULL)&&(par->ft==1)){
    printf("\n\nERROR:CAN'T OPEN FILE (%s) :CHECK THE argv[1]\n",F->fn);
    exit(-1);
  }
  rewind(F->fp);
  double *xt;
  double *yt;
  double kinectAA;
  double disAA;
  xt = (double *)malloc(sizeof(double)*F->ndata);
  yt = (double *)malloc(sizeof(double)*F->ndata);
  int n;
  for(n=0;n<F->ndata;n++){
    fgets(F->fn,126,F->fp);
    int ret_f=sscanf(F->fn,"%lf %lf\n",&yt[n],&xt[n]);
    if(n!=0){
      xt[n]=(xt[n]-xt[0])*GPSx;
      yt[n]=(yt[n]-yt[0])*GPSy;
    }
    printf("n=%d\txt=%lf\tyt=%lf\n",n,xt[n],yt[n]);
  }  
  xt[0]=yt[0]=0;
  printf("xt=%lf\tyt=%lf\n",xt[0],yt[0]);
  ///////////////////////////////////////////////////


  if(DELTA<U->T) DELTA=U->T;//max ZU->Ty
  {
    //int n;
    for(n=0;n<par->nZ;n++){
      FILEDATA *Z=&(par->Z[n]);
      rewind(Z->fp); 
      Z->l=0;
      Z->nS0=-1000;//for initialization?
      if(Z->class==0){//LRF
	//初期計測(z_0)を取得
	getLRFdatafromfile(par->dir,scan_k,0);//LRFfunc.c
      }
      else if(Z->class==1){//GPS
	//	totaltime=Z->ndata;
	if(DELTA<Z->T) DELTA=Z->T;//max ZU->T
      }
      else if(Z->class==2){//ORS
	//	totaltime=Z->ndata;
	if(DELTA<Z->T) DELTA=Z->T;//max ZU->T
      }
    }
  }
  //printf("DELTA=%lf\n",DELTA);
  
  if(par->DELTA!=0)  DELTA=par->DELTA;
  if(DELTA<U->T) DELTA=par->DELTA=U->T;//must DELTA >= U->T
  {//for DELTA > U->T ; 
    U->lD=DELTA/U->T;
    totaltime=U->ndata*U->T/DELTA;
    printf("U->lD=%lf,totaltime=%d\n",U->lD,totaltime);
    int n;
    for(n=0;n<par->nZ;n++){
      FILEDATA *Z=&(par->Z[n]);
      Z->lD=DELTA/Z->T;
      printf("Z%d->=%lf\n",n,Z->lD);
    }
  }
  //printf("DELTA=%lf\n",DELTA);

  //rand関数の初期化
  initrand();//in init.h
  //パーティクルの値をオールリセット
  resetPFonstart(pos_k,par->robotang0); //in init.c
  //各種記録用ファイルを全消去して準備
  resetALLlogfile(par->dir);//in init.c
  //  resetALLlogfile(argv);//in init.c
  STATE robot=setInitrobotpos(par->robotang0);
  STATE nonnoisepos=setInitrobotpos(par->robotang0);
  //printf("par->robotang0=%lf",par->robotang0);



  //記録用ファイルをオープン/////////////////////////////////
  char buff[256];
  FILE *pos_file,*ptcl_file,*rawpos_file,*ess_file,*var_file,*map_file,*ors_file,*gps_file,*kinect_file,*input_file,*weight_file,*true_dis_file,*dis_gps_particle_file,*dis_particle_map_file,*dis_onlyu_map_file;;
  {
    //自己位置推定結果を保存するfileをopen
    sprintf(buff,"%s/position_usingPF.dat",par->dir);
    pos_file=fopen(buff,"w");
    //初期位置を書いておく
    recordInitrobotstate(pos_file,par->robotang0);
    //入力のみを用いた自己位置推定結果を保存するfileをopen
    sprintf(buff,"%s/position_onlyU.dat",par->dir);
    rawpos_file=fopen(buff,"w");
    //初期位置を書いておく
    recordInitrobotstate(rawpos_file,par->robotang0);
    //リサンプリング後のパーティクルの軌跡（全体）を保存するfileをopen
    sprintf(buff,"%s/log/ptclresampleAlltime.dat",par->dir);
    ptcl_file=fopen(buff,"w");
    //有効サンプルサイズ(ESS)を保存するfileをopen
    sprintf(buff,"%s/log/ESS.dat",par->dir);
    ess_file=fopen(buff,"w");
    //分散データ保存するfileをopen
    sprintf(buff,"%s/log/VARIANCE.dat",par->dir);
    var_file=fopen(buff,"w");
    //マップ(3D)を保存するfileをopen
    sprintf(buff,"%s/log/MAP.dat",par->dir);
    map_file=fopen(buff,"w");
    //ORSデータ(変換後)を保存するfileをopen
    sprintf(buff,"%s/log/ORS.dat",par->dir);
    ors_file=fopen(buff,"w");
    //GPS
    //sprintf(buff,"%s/log/GPS.dat",par->dir);
    gps_file=fopen(buff,"w");
    recordInitrobotstate(gps_file,par->robotang0);
    //Kinect
    sprintf(buff,"%s/log/KINECT.dat",par->dir);
    kinect_file=fopen(buff,"w");
    //input
    sprintf(buff,"%s/log/INPUT.dat",par->dir);
    input_file=fopen(buff,"w");
    //weight
    sprintf(buff,"%s/log/ALFA.dat",par->dir);
    weight_file=fopen(buff,"w");
    //true_dis
    sprintf(buff,"%s/log/TRUE_DIS.dat",par->dir);
    true_dis_file=fopen(buff,"w");
    //dis_gps_particle
    sprintf(buff,"%s/log/PAR_GPS_DIS.dat",par->dir);
    dis_gps_particle_file=fopen(buff,"w");
    //dis_particle_map
    sprintf(buff,"%s/log/PAR_MAP_DIS.dat",par->dir);
    dis_particle_map_file=fopen(buff,"w");
        //dis_particle_map
    sprintf(buff,"%s/log/AonlyU_MAP_DIS.dat",par->dir);
    dis_onlyu_map_file=fopen(buff,"w");
    //dis_onlyU_map
    sprintf(buff,"%s/log/AonlyU_MAP_DIS.dat",par->dir);
    dis_onlyu_map_file=fopen(buff,"w");
  }

  FILE *true_file;
  char buff2[256];
  sprintf(buff2,"%s/log/TRUE.dat",par->dir);
  true_file=fopen(buff2,"w");
  for(n=0;n<F->ndata;n++){
    fprintf(true_file,"%.3f %.3f\n",xt[n],yt[n]);
  }





  int j;//  int j,interval=DELTA*10;
  int map_img_w=MAPSIZE_W;
  int map_img_h=MAPSIZE_H;

  int lrf_img_w=400/par->MATCHres*2;//  int lrf_img_w=4000/atoi(argv[5])*2;
  int scan_img_w=20;
  int spa_img_w=20;      //「SOURCE」,「PREDICT」,「ACTUALLY」各ウィンドウの拡大縮小後のウィンドウ幅(H23,3)
//  int lrf_img_w=4000/par->MATCHres*2;//  int lrf_img_w=4000/atoi(argv[5])*2;
//  int scan_img_w=200;
//  int spa_img_w=200;
  ///////////////////////
  //その他画像系の定義
  ///////////////////////
 
  IplImage *map_img=cvCreateImage(cvSize(map_img_w,map_img_h),8,3);
  IplImage *scan_img=cvCreateImage(cvSize(scan_img_w,scan_img_w),8,3);
#ifdef useLRF
  IplImage *lrf_k_img=cvCreateImage(cvSize(lrf_img_w,lrf_img_w),8,1);
  IplImage *lrf_predict_img=cvCreateImage(cvSize(lrf_img_w,lrf_img_w),8,1);
  IplImage *lrf_kplus1_img=cvCreateImage(cvSize(lrf_img_w,lrf_img_w),8,1);
  // IplImage *ptcl_img=cvCreateImage(cvSize(scan_img_w,scan_img_w),8,3);
#endif //#ifdef useLRF
  SetImageconfig(map_img,"SLAM",0,0);
  // SetImageconfig(ptcl_img,"PARTICLE",map_img_w+10,0);
  SetImageconfig(scan_img,"SCAN",map_img_w+20,0);
  SetImageconfig(lrf_k_img,"SOURCE",map_img_w+10,scan_img_w+60);
  SetImageconfig(lrf_predict_img,"PREDICT",map_img_w+spa_img_w+20,scan_img_w+60);
  SetImageconfig(lrf_kplus1_img,"ACTUALLY",map_img_w+2*spa_img_w+30,scan_img_w+60);
  IplImage *robotonmap_img=cvCreateImage(cvSize(map_img_w,map_img_h),8,3);
  IplImage *maponmap_img=cvCreateImage(cvSize(map_img_w,map_img_h),8,3);
  cvSet(maponmap_img,cvScalarAll(128),NULL);  //イメージをグレーに変換
  
  ////////////////////////////
  //再帰的アルゴリズム開始
  ////////////////////////////
  //  for(i=0;i<U->ndata;i++){
  GPSDATA gps={-1000,-1000,-1000,-1000};
  KINECTDATA kinect={-1000,-1000,-1000};
    for(i=0;i<totaltime;i++){//totaltime=par->Z[0].ndata// number of GPS data

    //    if(1==1){//if(i%interval==0){
    //t>0のとき前時刻のデータをコピー
    if(i!=0){
      copyParticle(pos_k,pos_resample);
      copyLRF(scan_k,scan_kplus1);
    }
     
    ////////////////////////
    //next STATE prediction
    ////////////////////////  
    //入力をファイルから読み込む
    // get mean(average)-input for U->lD data;

    /************追加**************/ 
    FILEDATA *T=&(par->T[0]);
    char buff[64];
    int sec,usec,min;
    int gpstime;
    float tmp;
   
    //if(i==0) {fgets(buff,64,T->fp);} 
    fgets(buff,64,T->fp);
    sscanf(buff,"%f %f %f %f:%d:%d.%d %f %f",&tmp,&tmp,&tmp,&tmp,&min,&sec,&usec,&tmp,&tmp);
    if(feof(T->fp)) break;
    gpstime= min*1000000+sec*1000 + usec;
    // printf("gpstime=%d\n",gpstime);
    INPUT input;
    switch(input_case){
    case 1:
      getInput1(&input,U,gpstime,i);
      break;
    default:
      getInput(&input,U,gpstime,i);
      break;
    }
    double t=input.DELTA*i;
    /****************************/

    //INPUT input=getInput(par->dir,i);//pfprocess.c
    //姿勢予測
    //pos_predict[i].state (.pos.x, .pos.x, .theta)
    predict_pos(input,pos_k,pos_predict);//pfprocess.c

    //入力のみを用いた推定姿勢を計算しファイルに記録

    nonnoisepos=calcposNonNoise(input,nonnoisepos);//pfprocess.c
    { 
      fprintf(stderr,"#t%g xya(%g,%g,%g) vw(%g,%g) xyaodm(%g,%g,%g)\n",
	      t,nonnoisepos.pos.x/m2mm,nonnoisepos.pos.y/m2mm,nonnoisepos.theta,input.v,input.omega*R2D,U->x0/m2mm,U->y0/m2mm,U->a0);
    }
    recordRobotstateALL(rawpos_file,nonnoisepos,i); //fileRorW.c
    //    calc_var(nonnoisepos,pos_predict);//no effect?
    //    fprintf(stderr,"#variance=%g %g %g\n",variance.pos.x,variance.pos.y,variance.theta);

    //    STATE variance_pre=calc_var(nonnoisepos,pos_predict);
    
    ////////////////////////////////////////////
    //next OBSERVATION prediction, calc weight
    ////////////////////////////////////////////
    //GPSDATA gps={-1000,-cc1000,-1000,-1000};
    double angORS=-1000;
    double wsum=0;
    int obsflag=0;
    {
      FILEDATA *Z=&(par->Z[0]);
      FILEDATA *ZG=&(par->Z[1]);
      FILEDATA *F=&(par->F[0]);
      FILEDATA *K=&(par->K[0]);
      rewind(K->fp);
      //char buff[64];for(;Z->l<(int)t1/Z-T;) {fgets(buff,64,Z-fp);Z->l++;}
      /* if(par->ft==0){
	getGPSdatafromfile(&gps,Z,par->robotang0);
      } else {
	getGPSdatafromfile1(&gps,Z,par->robotang0,F);
      }     //printf("gps->theta=%lf\n",gps.theta);
      fprintf(stderr,"%g %g #gps x y[m]\n",gps.x,gps.y);
      //obsflag=1;
      //char buff[64];//for(;Z->l<(int)t1/Z->T;) {fgets(buff,64,Z->fp);Z->l++;}*/
      getORSdatafromfile(&angORS,ZG,gpstime,i,par->robotang0);
      fprintf(stderr,"%g #ORS[deg]\n",angORS);
      getKinectdatafromfile(&kinect,K,gpstime,par->kinectang0);
      obsflag=1;printf("\\\\\\\\22517\n");
      if(obsflag){

	      //尤度計算
              //double S_gps=3260*exp(-0.85*gps.nS);//A
	      //double S=3260.0*exp(-0.85*6.5);   //for test
	      double smallw_gps=0;
	      double smallw_kinect=0;
	      double S_kinect=16;//実験で求めるD
	      //	  double smallw=1e-4/NofParticles;
	      double S_ors=16;//実験で求めるC
	      double smallw_ors=1e-4/NofParticles;
	      //double kinectA=0.5;
	      double gpsA;/*if(par->gpsA>=0.0)*/
	      // gpsA=par->gpsA;
	      double orsA;/*if(par->orsA>=0.0)*/ 
	      orsA=par->orsA;
	      double kinectA;/*if(par->kinectA>=0.0)*/ 
	      kinectA=par->kinectA;
	      // double gps_theta;
	      // if(par->gps_theta>=0.0) gps_theta=par->gps_theta; 

	      double angORS1=angORS-360;
	      double angORS2=angORS+360;
	      double gpstheta1=gps.theta-360;
	      double gpstheta2=gps.theta+360;
	      // printf("gps.d=%lf\n",gps.d);
	      for(j=0;j<NofParticles;j++){

		double a=pos_predict[j].state.theta;
		double da=fabs(a-angORS);
		//double da_gps=fabs(a-gps.theta);
		if(da>fabs(a-angORS1)) da=fabs(a-angORS1);
		else if(da>fabs(a-angORS2)) da=fabs(a-angORS2);
		//if(da_gps>fabs(a-gpstheta1)) da_gps=fabs(a-gpstheta1);
		//else if(da_gps>fabs(a-gpstheta2)) da_gps=fabs(a-gpstheta2);

		double wj_ors=exp(-da*da/S_ors)+smallw_ors;
		//double wj_a_gps;
		/*if(gps.d == 0.0){
		  gps_theta = 0.0;
		  wj_a_gps=exp(-da_gps*da_gps)/S_gps;
		  //printf("j=%d\n",j);
		}else{
		  wj_a_gps=exp((-da_gps*da_gps)/(1/gps.d));
		  }*/
		//pos_predict[j].weight=wj;
		//double dx=pos_predict[j].state.pos.x/m2mm-gps.x;
		//double dy=pos_predict[j].state.pos.y/m2mm-gps.y;
		//double wj_gps=exp(-(dx*dx+dy*dy)/S_gps)+smallw_gps;

		double dkx=pos_predict[j].state.pos.x/m2mm-kinect.x;
		double dky=pos_predict[j].state.pos.y/m2mm-kinect.y;
		double wj_kinect=exp(-(dkx*dkx+dky*dky)/S_kinect)+smallw_kinect;

		double dis_true=calcalfa(F,xt,yt,kinect.x,kinect.y);//kinect計測値と理想軌道との距離
		disAA=dis_true;
		//	printf("dis_true=%lf\n\n",dis_true);
		//kinectA=0.9;
		/*gpsA=(((sqrt(16.0*PI))*(exp(-dis_true*dis_true/16.0)/(sqrt(16.0*PI))))/(8*sqrt(PI)))*0.1;//alfaの計算
		if(dis_true*dis_true<=2.0){
		gpsA=0.1;
		gpsAA=gpsA;
		orsA=0.1-gpsA;
		}*/
		//kinectA=((sqrt(16.0*PI))*(exp(-dis_true*dis_true/16.0)/(sqrt(16.0*PI))))/(8*sqrt(PI));//alfaの計算
		//kinectA=0.5;
		
		if(dis_true*dis_true<=3.0){
		kinectA=1;
		kinectAA=kinectA;
		}else {
		  kinectA=((sqrt(16.0*PI))*(exp(-dis_true*dis_true/16.0)/(sqrt(16.0*PI))))/(8*sqrt(PI));
		  kinectAA=kinectA;
		}
		orsA=1-kinectA;
		
		//	printf("kinectA=%lf\n\n",kinectA);
		//printf("gpsA=%lf\n\n",gpsA);
		//printf("orsA=%lf\n\n",orsA);
		
		//double wj=(wj_ors * orsA) + (wj_gps * gpsA)+(wj_a_gps * gps_theta) + (wj_kinect * kinectA);
		double wj=(wj_ors * orsA) + (wj_kinect * kinectA);

		pos_predict[j].weight=wj;
		wsum+=wj;
	      }
	      //	  fprintf(stderr,"\n");
	       weightnormalize(pos_predict,wsum);
	    }
    }
    //サンプリングデータと重みをファイルに記録
    recordSamplingandWeightLog(par->dir,pos_predict,i);
    //パーティクル分布を描画
    //    DrawParticleonRobotWindow(ptcl_img,input,robot,pos_predict,variance_pre,2);
    ///////////////////////
    //resampling
    ///////////////////////
    //    if(obsflag)
    resampling(ess_file,pos_predict,pos_resample,i);
    //      int resampling_flag=resampling(ess_file,pos_predict,pos_resample,i);
    recordResampledataatTIME(par->dir,pos_resample,i+1);
    recordPtclLogatALL(ptcl_file,pos_resample);
    
    ////////////////////////////
    //Localization and DrawMap
    ////////////////////////////    
    //自己位置推定値を決定しファイルに記録
    robot=Localization(pos_resample,LMETHOD);
    recordRobotstateALL(pos_file,robot,i);
    //calcdis(dis_gps_particle_file,robot,gps.x,gps.y,gps.nS,i);
    //ORSファイルに記録
    recordORSdata(ors_file,angORS,i);
    //GPS
    //recordGPSdata(gps_file,gps.x,gps.y,gps.theta,i);
    //KINECT
    recordKINECTdata(kinect_file,kinect.x,kinect.y,i,j);
    //input
    recordInputdata(input_file,input,i);
    recordweightdata(weight_file,kinectAA,i);
    recordtruedisdata(true_dis_file,disAA,gps.nS,i);
    double par_map_dis=calcalfa(F,xt,yt,robot.pos.x/1000.0,robot.pos.y/1000.0);
    //recordtruedisdata(dis_particle_map_file,par_map_dis,gps.nS,i);
    double onlyu_map_dis=calcalfa(F,xt,yt,nonnoisepos.pos.x/1000.0,nonnoisepos.pos.y/1000.0);
    //recordtruedisdata(dis_onlyu_map_file,onlyu_map_dis,gps.nS,i);
    //分散を計算して記録
    STATE variance_res;
    calc_var1(robot,pos_resample,&variance_res);
    //    STATE variance_res=calc_var(robot,pos_resample);
    //    fprintf(stderr,"#variance=%g %g %g\n",variance_res.pos.x,variance_res.pos.y,variance_res.theta);
    recordVarianceALL(var_file,variance_res,i);
    
    STATE ROBOTSTATE;
    cvZero(robotonmap_img);
    //#ifdef PFMAP
    if(PFMAP){//usePF
      //パーティクルを描画(@robotonmap_img)
      for(j=0;j<NofParticles;j++){
	STATE particle;
	particle.pos.x=pos_predict[j].state.pos.x;
	particle.pos.y=pos_predict[j].state.pos.y;
	particle.theta=pos_predict[j].state.theta;
	DrawRobotmarkonmap(2,robotonmap_img,particle,par->offset,CV_RGB(0,0,255),(double)par->MAPres,i);  
	//	  DrawRobotmarkonmap(2,robotonmap_img,particle,par->offset,CV_RGB(0,0,255),atof(argv[6]),i);  
      }
      //分散楕円を描画
      DrawVarellipsonmap(robotonmap_img,robot,par->offset,variance_res,(double)par->MAPres,i);  
      //	DrawVarellipsonmap(robotonmap_img,robot,par->offset,variance_res,atof(argv[6]),i);  
      ROBOTSTATE=robot;
    }
    else{
      //#else 
      ROBOTSTATE=nonnoisepos;
    }
    //#endif
    //推定位置を描画(@robotonmap_img)
    DrawRobotmarkonmap(1,robotonmap_img,ROBOTSTATE,par->offset,CV_RGB(255,0,0),(double)par->MAPres,i);  
    {
      STATE bigorient;
      bigorient.pos.x=0; 
      bigorient.pos.y=0;
      bigorient.theta=ROBOTSTATE.theta;
      DrawORSmarkonmap(2000,robotonmap_img,bigorient,par->offset,CV_RGB(255,0,0),(double)par->MAPres,i);  
    }
    /*if(gps.nS>-1000){
      DrawGPSmarkonmap(robotonmap_img,gps,par->offset,CV_RGB(0,255,0),(double)par->MAPres,i);  
      //      STATE gpsstate;
      //      gpsstate.pos.x=gps.x*m2mm; 
      //      gpsstate.pos.y=gps.y*m2mm;
      //      gpsstate.theta=0;
      //      DrawRobotmarkonmap(1,robotonmap_img,gpsstate,par->offset,CV_RGB(0,255,0),(double)par->MAPres,i);  
      }*/
    if(angORS>-1000){
      STATE orsstate;
      orsstate.pos.x=5000; 
      orsstate.pos.y=0;
      orsstate.theta=angORS;
      DrawORSmarkonmap(2000,robotonmap_img,orsstate,par->offset,CV_RGB(0,255,0),(double)par->MAPres,i);  
    }
    //      DrawRobotmarkonmap(1,robotonmap_img,ROBOTSTATE,par->offset,CV_RGB(255,0,0),atof(argv[6]),i);  
    //地図を描画(@maponmap_img)
    //    if(1==0) DrawLrfdataonmap(maponmap_img,ROBOTSTATE,par->offset,lrf_k,ANG,CUT,(double)par->MAPres,i);
    //      DrawLrfdataonmap(maponmap_img,ROBOTSTATE,par->offset,lrf_k,ANG,CUT,atof(argv[6]),i);
    //robotonmap_imgとmaponmap_imgを合成して表示(@"SLAM"window)
    superpositionRandL(robotonmap_img,maponmap_img,map_img);

    //3Dmapをdatファイルに保存
    record3DMAPdata(map_file,robot,lrf_k,2);
    if(tsnap>=t && tsnap<t+DELTA){//kuro
      sprintf(buff,"mkdir ../result > /dev/null 2>&1");system(buff);
      sprintf(buff,"../result/map_t%g.png",tsnap);
      cvSaveImage(buff,robotonmap_img,0);
      fprintf(stderr,"#Image %s is saved\n",buff);
      char KEY2[2];
      //      fprintf(stdout,"HIT 'c' KEY to continue.");fflush(stdout);fflush(stdin);
      fprintf(stdout,"HIT A KEY to continue.");fflush(stdout);fflush(stdin);
      for(;;){
	char *ret=fgets(KEY2,2,stdin);
	if(ret!=NULL) break;//	if(KEY2[0]=='c') break;
	//	fgets(KEY2,2,stdin);//  ret=fgets(KEY2,2,stdin);
	//break;//	if(KEY2[0]=='c') break;
      }
    }
    int code = cvWaitKey (10);
    if( code == 27 || code == 'q' || code == 'Q' ) break;
/**/  }//  for(i=0;i<totaltime-1;i++){//totaltime=par->Z[0].ndata// number of GPS data
  
  fclose(pos_file);
  fclose(ptcl_file);
  fclose(rawpos_file);
  fclose(ess_file);
  fclose(var_file);
  fclose(map_file);
  fclose(ors_file);
  //fclose(gps_file);
  fclose(kinect_file);
  fclose(input_file);
  fclose(true_file);
  fclose(weight_file);
  fclose(true_dis_file);
  fclose(dis_gps_particle_file);
  fclose(dis_particle_map_file);
  //地図を保存
  sprintf(buff,"%s/MAPIMAGE.png",par->dir);
  //  sprintf(buff,"%s/MAPIMAGE.jpg",par->dir);
  cvSaveImage(buff,maponmap_img,0);

  cvDestroyWindow("SLAM"); cvReleaseImage(&map_img);   

  cvDestroyWindow("SCAN"); cvReleaseImage(&scan_img);   
  cvDestroyWindow("SOURCE"); cvReleaseImage(&lrf_k_img);   
  cvDestroyWindow("PREDICT"); cvReleaseImage(&lrf_predict_img);   
  cvDestroyWindow("ACTUALLY"); cvReleaseImage(&lrf_kplus1_img);   

  // cvDestroyWindow("PARTICLE"); cvReleaseImage(&ptcl_img);   
  cvReleaseImage(&robotonmap_img);   
  cvReleaseImage(&maponmap_img);   

  printf("MISSION COMPLETED!!\n");
  for(i=0;i<8;i++) Plotresult2Dline(i,par,totaltime,DISP);
  if(DISP>0){
    char KEY2[2];
    fprintf(stdout,"HIT A KEY to quit.");fflush(stdout);
    fgets(KEY2,2,stdin);//  ret=fgets(KEY2,2,stdin);
  }

  SaveExperimentdata(par);

  free(xt);
  free(yt);
  return 0;
}
