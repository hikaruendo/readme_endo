/////////////////////////////////////////////////
//入力ファイルのオープンチェック，行数を数える
/////////////////////////////////////////////////
//#define tenmillion   1000000
//#define sixtymillion 6000000
#define tenmillion   10000000
#define sixtymillion 60000000
#define mag_min100 1000000.0
#define mag_min6 60000.0
int i_min_sec_usec(double min,int sec,int usec)
{
  //  return(min* 60000+sec*1000 + usec);
  //  return(min*   600000+sec*1000 + usec);
  return(min*mag_min100+sec*1000 + usec);//what does this mean?
}
int CheckandcountFILEDATA(FILEDATA *UZ){
  //int CheckandcountINPUT(char *CL[]){
  //  FILE *fp;
  char buff[64];
  //  sprintf(buff,"%s/%s",par->dir,par->ACTfn);
  printf("\nInput INPUT file is %s",UZ->fn);
  if((UZ->fp=fopen(UZ->fn,"r"))==NULL){
    printf("\n\nERROR:CAN'T OPEN FILE (%s) :CHECK THE argv[1]\n",UZ->fn);
    exit(-1);
  }
  int cnt=0;//  int letter,cnt=0;
  for(;;){
    fgets(buff,64,UZ->fp);
    if(feof(UZ->fp)) break;
    if(strlen(buff)<2) continue;
    cnt++;
//    letter=fgetc(par->ACTfp);
//    if(letter=='\n') cnt++;
//    else if(letter==EOF) break;
  }
  printf("\nInput file as %d data",cnt);
  //  fclose(fp);
  return cnt;
}

int CheckandcountINPUT(FILEDATA *U){
  //int CheckandcountINPUT(char *CL[]){
  //  FILE *fp;
  char buff[64];
  //  sprintf(buff,"%s/%s",par->dir,par->ACTfn);
  //  FILEDATA *U=&(par->U[0]);
  printf("\nInput INPUT file is %s",U->fn);
  if((U->fp=fopen(U->fn,"r"))==NULL){
    printf("\n\nERROR:CAN'T OPEN FILE (%s) :CHECK THE argv[1]\n",U->fn);
    exit(-1);
  }
  int cnt=0;//  int letter,cnt=0;
  for(;;){
    fgets(buff,64,U->fp);
    if(feof(U->fp)) break;
    if(strlen(buff)<2) continue;
    cnt++;
//    letter=fgetc(par->ACTfp);
//    if(letter=='\n') cnt++;
//    else if(letter==EOF) break;
  }
  printf("\nInput file unkoas %d data",cnt);
  //  fclose(fp);
  return cnt;
}

////////////////////////////////////////////////////
//LRFファイルのオープンチェック，ファイル数を数える
////////////////////////////////////////////////////
int CheckandcountLRF(PARAM *par){
  //int CheckandcountLRF(char *CL[]){
  DIR *pdir;
  char buff[256];
  sprintf(buff,"%s/LRF",par->dir);//  sprintf(buff,"%s/LRF",CL[1]);
  if((pdir=opendir(buff))==NULL) {
    char cmd[256];
    sprintf(cmd,"./sepLRFdat %s",par->dir);
    {int ret=system(cmd); if(ret<0) fprintf(stderr,"#error occurs at '%s'.\n",cmd);}
  } 
  else closedir(pdir);

  FILE *fp;
  //  char buff[256];
  int cnt=0;
  for(;;){
    sprintf(buff,"%s/LRF/LRF_%d.dat",par->dir,cnt);
    if((fp=fopen(buff,"r"))==NULL) break;
    else cnt++;
  }
  printf("\nThere are %d LRF files",cnt);
  return cnt;
}

/////////////////////////////////////////////////
//初期姿勢をファイルに書き込む
/////////////////////////////////////////////////
void recordInitrobotstate(FILE* fp,double ang){
  fprintf(fp,"0.0000 0.0000 %lf -1 #x,y,theta,t\n",ang);//modified by kuro
}

/////////////////////////////////////////////////
//推定姿勢をファイルに書き込む
/////////////////////////////////////////////////
void recordRobotstateALL(FILE* fp, STATE state, int t){
  fprintf(fp,"%.3f %.3f %.3f %d\n",state.pos.x,state.pos.y,state.theta,t);//modified by kuro
  //  fprintf(fp,"%.13e %.13e %.13e %d\n",state.pos.x,state.pos.y,state.theta,t);//modified by kuro
  //  fprintf(fp,"%.13e %.13e %.13e %d\n",state.pos.x,state.pos.y,state.theta,t);//modified by kuro
  //  fprintf(fp,"%d %.13e %.13e %.13e\n",t,state.pos.x,state.pos.y,state.theta);
}
///////////////////////////////////////////////////////////
//ORS(変換後)のデータをファイルに書き込む
///////////////////////////////////////////////////////////
void recordORSdata(FILE* fp, double ang, int t){
  fprintf(fp,"%.3f %d\n",ang,t);
}
///////////////////////////////////////////////////////////
//GPS
///////////////////////////////////////////////////////////
void recordGPSdata(FILE* fp, double x, double y,double theta, int t){
  fprintf(fp,"%.3f %.3f %.3f %d\n",x,y,theta,t);
}

///////////////////////////////////////////////////////////
//KINECT
///////////////////////////////////////////////////////////
void recordKINECTdata(FILE* fp, double x, double y,int t,int s){
  fprintf(fp,"%.3f %.3f %d %d\n",x,y,t,s);
}

///////////////////////////////////////////////////////////
//input
///////////////////////////////////////////////////////////
void recordInputdata(FILE* fp,INPUT input,int t){
  fprintf(fp,"%.3f %.3f %.3f %d\n",input.v,input.omega,input.DELTA,t);
}
void recordweightdata(FILE* fp,double i,int t){
  fprintf(fp,"%.3f %d\n",i,t);
}
void recordtruedisdata(FILE* fp,double i,int n,int t){
  fprintf(fp,"%.3f %d %d\n",i,n,t);
}
///////////////////////////////////////////////////////////
//リサンプル後のパーティクルの分布をファイルに書き込む(時間毎)
///////////////////////////////////////////////////////////
void recordResampledataatTIME(char FILEPASS[], PARTICLE* pos, int t){
  FILE *ptcl_file;
  char buff[256];
  int i;
  sprintf(buff,"%s/log/resampleat%d.dat",FILEPASS,t);
  ptcl_file=fopen(buff,"w");
  for(i=0;i<NofParticles;i++){
    fprintf(ptcl_file,"%.13e %.13e %.13e\n",pos[i].state.pos.x,pos[i].state.pos.y,pos[i].state.theta);
  }
  fclose(ptcl_file);
}

////////////////////////////////////////////////////
//パーティクルの分布をファイルに書き込む(全時刻分)
////////////////////////////////////////////////////
void recordPtclLogatALL(FILE* file, PARTICLE* pos){
  int i;
  for(i=0;i<NofParticles;i++){
    fprintf(file,"%.13e %.13e %.13e\n",pos[i].state.pos.x,pos[i].state.pos.y,pos[i].state.theta);
  }
}

////////////////////////////////////////////////
//サンプリングデータと尤度分布をファイルに書き込む
////////////////////////////////////////////////
void recordSamplingandWeightLog(char FILEPASS[], PARTICLE* pos,int t){
  FILE *weight_file;
  char buff[256];
  int i;
  sprintf(buff,"%s/log/ptclandweightat%d.dat",FILEPASS,t);
  weight_file=fopen(buff,"w");
  for(i=0;i<NofParticles;i++){
    fprintf(weight_file,"%d %.13e %.13e %.13e %.13e\n",i,pos[i].state.pos.x,pos[i].state.pos.y,pos[i].state.theta,pos[i].weight);
  }
  fclose(weight_file);
}

/////////////////////////////////////////////////
//分散値をファイルに書き込む
/////////////////////////////////////////////////
void recordVarianceALL(FILE* fp, STATE variance, int t){
  fprintf(fp,"%d %.13e %.13e %.13e\n",t,variance.pos.x,variance.pos.y,variance.theta);
}

///////////////////////////////////
//3次元地図データをファイルに記録
///////////////////////////////////
void record3DMAPdata(FILE* fp, STATE robot,SCAN LRFdata[],int interval){
  int i;
  CvPoint3D64f map;
  double p=CV_PI/180;
  for(i=0;i<lrf_data_num;i++){
    if(i%interval==0&&(LRFdata[i].pt.x!=0||LRFdata[i].pt.y)){
      map.x=cos((robot.theta-90)*p)*LRFdata[i].pt.x
	-sin((robot.theta-90)*p)*LRFdata[i].pt.y+robot.pos.x;
      map.y=sin((robot.theta-90)*p)*LRFdata[i].pt.x
	+cos((robot.theta-90)*p)*LRFdata[i].pt.y+robot.pos.y;
      map.z=LRFdata[i].pt.z;
      fprintf(fp,"%.13e %.13e %.13e \n",map.x,map.y,map.z);
    }
   }
  fprintf(fp,"\n");
}

/////////////////////////////////////
//ファイルから入力をとってくる
/////////////////////////////////////
char *getInput(INPUT *input,FILEDATA *U,int gpstime,int loopnum){
  char buff[64];
  char *ret;
  int lD=0,odotime,arraynum,flag=0,odofirst=0;
  double timedif=0.0;
  int dif[2];
  fpos_t fpos;
  
  if(loopnum!=0) odofirst = U->timeA; 
  fgetpos(U->fp,&fpos);//現在のファイル位置を取得

  for(;;){//do wile gpstime>odotime ??
    //  if(lD==22) exit(0);
    fgets(buff,64,U->fp);
    if(feof(U->fp)){
      printf("break");
      break;
    }
    double tmp;
    int sec,usec,min;
    sscanf(buff,"%lf%lf%lf %lf:%d:%d.%d\n",&tmp,&tmp,&tmp,&tmp,&min,&sec,&usec);
    lD++;
    odotime=i_min_sec_usec(min,sec,usec);//    odotime=min*1000000+sec*1000 + usec;
    if(gpstime<odotime) break;
  }
#ifdef ORIG

  for(;;){//do wile gpstime>odotime ??
    //  if(lD==22) exit(0);
    fgets(buff,64,U->fp);
    if(feof(U->fp)){
      printf("break");
      break;
    }
    double tmp;
    int sec,usec,min;
    sscanf(buff,"%lf%lf%lf %lf:%d:%d.%d\n",&tmp,&tmp,&tmp,&tmp,&min,&sec,&usec);
    odotime=i_min_sec_usec(min,sec,usec);//    odotime=min*1000000+sec*1000 + usec;
    if(lD!=0){
      arraynum =(lD + 1) % 2;
      switch(arraynum){
      case 0:
	dif[0] = abs( gpstime - odotime);
	//	printf("case0:dif[0]=%d,dif[1]=%d,gpstime,odotime=%d,%d,lD%d\n",dif[0],dif[1],gpstime,odotime,lD);
	if(dif[0]>tenmillion) dif[0]=abs(dif[0]-sixtymillion);
	if(dif[0]>dif[1]) flag = 1;
	break;
      case 1:
	dif[1] = abs( gpstime - odotime);
	//	printf("case1:dif[0]=%d,dif[1]=%d,gpstime,odotime=%d,%d,lD%d\n",dif[0],dif[1],gpstime,odotime,lD);
	if(dif[1]>tenmillion) dif[1]=abs(dif[1]-sixtymillion);
	if(dif[1]>dif[0]) flag = 1;
	break;
      default:
	printf("異常　　終了します。\n");
	exit(1);
	break;
      }
    }else{
      dif[1] = abs ( gpstime - odotime);
      if(dif[1]>tenmillion) dif[1]=abs(dif[1]-sixtymillion);
    }
    if(flag == 1) break;
    lD++;
  }
#endif //#ifdef ORIG
  //printf("lD=%d\n",lD);
  if(lD>=30){
    printf("LDerror\n");
    //exit(0);
  }
  fsetpos(U->fp,&fpos);

  double *aa=(double*)malloc(sizeof(double)*lD);
  if(U->class==1){//P3AT by yoshinaga
    double x=0,y=0,a=0,tmp;
    int l,sec,usec,odoend,min;
    for(l=0;l<lD;l++){
      ret=fgets(buff,64,U->fp); 
      if(feof(U->fp)) break;
      double _x,_y,_a;

      sscanf(buff,"%lf%lf%lf %lf:%d:%d.%d\n",&_x,&_y,&_a,&tmp,&min,&sec,&usec);
      {//if(1==0){
	for(;;){
	  if(_a<-180) _a+=360;
	  else if(_a>180) _a-=360;
	  else break;
	}
      }
      // printf("sec=%d,usec=%d",sec,usec);
      // -180 < _a < 180
      x+=_x;y+=_y;a+=_a;
      aa[l]=_a;
      U->l++;
    }
    odoend = U->timeA = i_min_sec_usec(min,sec,usec);// odoend = U->timeA = min*1000000+sec*1000 + usec;
    if(loopnum!=0){
      timedif=input->DELTA=(odoend - odofirst)/mag_min100 ;//timedif = input->DELTA = (odoend - odofirst) / 1000000.0 ;
      if(timedif<0.0){
	timedif+=mag_min6;
	input->DELTA+=mag_min6;
      }
    }
    //printf("odoend=%d,odofirst=%d,timedif=%lf,input->DELTA=%lf\n",odoend,odofirst,timedif,input->DELTA);
    a/=lD;    //    a*=-1.;//counter clockwise ?
    x/=lD;
    y/=lD;
    double amin=aa[0], amax=aa[0];
    for(l=1;l<lD;l++){
      if(amin>aa[l]) amin=aa[l];
      if(amax<aa[l]) amax=aa[l];
    }
    if(amax-amin>180){
      a=0;
      for(l=0;l<lD;l++) a+=(aa[l]>0)?aa[l]:aa[l]+360; // 0 < _a < 360
      a/=lD;
    }
    free(aa);

    if(loopnum == 0){//first line
      input->v=input->omega=input->DELTA=0;
    }
    else{
      double dx=x-U->x0;
      double dy=y-U->y0;
      double da=a-U->a0;
      {
	for(;;){
	  if(da<-180) da+=360;
	  else if(da>180) da-=360;
	  else break;
	}
      }
      da*=D2R;//[deg] to [rad]
      dx/=1000;//[mm] to [m]
      dy/=1000;//[mm] to [m]
      //input->v=sqrt(dx*dx+dy*dy)/(U->T*lD);//return value
      //input->omega=da/(U->T*lD);//return value
      input->v=(sqrt(dx*dx+dy*dy)/timedif)*0.88;//return value//*0.8//初期値
      input->omega=(da/timedif)*0.97;//*1.2
    }
    U->x0=x;U->y0=y;U->a0=a;
  }
  else if(U->class==0){
    double v=0,w=0;
    int n=0,l;
    for(l=0;l<U->lD;l++){
      ret=fgets(buff,64,U->fp); 
      if(feof(U->fp)) break;
      double _v,_w;
      sscanf(buff,"%lf %lf\n",&_v,&_w);
      v+=_v;w+=_w;
      n++;U->l++;
    }
    input->v=v/n/U->T;
    input->omega=w/n/U->T;
  }
  //  sprintf(buff,"%s/INPUT.dat",FILEPASS);
//  if((input_file=fopen(buff,"r"))== NULL){
//    printf("INPUTFILE OPEN ERROR\n");
//    exit(1);
//  }
//  int ret;
//  for(cnt=0;cnt<t+1;cnt++) ret=fscanf(input_file,"%lf %lf\n",&input.v,&input.omega);
//  fclose(input_file);
  return ret;
  //  return input;
}
char *getInput2(INPUT *input,FILEDATA *U,int gpstime,int loopnum){//for 3D kinect
  char buff[64];
  char *ret;
  int lD=0,odotime,arraynum,flag=0,odofirst=0;
  double timedif=0.0;
  int dif[2];
  fpos_t fpos;
  
  if(loopnum!=0) odofirst = U->timeA; 
  fgetpos(U->fp,&fpos);//現在のファイル位置を取得
  for(;;){
    //  if(lD==22) exit(0);
    fgets(buff,64,U->fp);
    if(feof(U->fp)){
      printf("break");
      break;
    }
    double tmp;
    int sec,usec,min;
    double odotime_d;//sec
    sscanf(buff,"%lf",&odotime_d);//kinect 3D
    //                                     t,   x,   y,   z,
    //sscanf(buff,"%lf%lf%lf %lf:%d:%d.%d\n",&tmp,&tmp,&tmp,&tmp,&min,&sec,&usec);
    //odotime=min*1000000+sec*1000 + usec;??odotime=min*60*1000+sec*1000 + usec;??
    odotime=odotime_d*1000.;//usec
    if(lD!=0){
      arraynum =(lD + 1) % 2;
      switch(arraynum){
      case 0:
	dif[0] = abs( gpstime - odotime);
	//printf("dif[0]=%d,dif[1]=%d\n",dif[0],dif[1]);
	if(dif[0]>tenmillion) dif[0]=abs(dif[0]-sixtymillion);
	if(dif[0]>dif[1]) flag = 1;
	break;
      case 1:
	dif[1] = abs( gpstime - odotime);
	//printf("dif[0]=%d,dif[1]=%d\n",dif[0],dif[1]);
	if(dif[1]>tenmillion) dif[1]=abs(dif[1]-sixtymillion);
	if(dif[1]>dif[0]) flag = 1;
	break;
      default:
	printf("異常　　終了します。\n");
	exit(1);
	break;
      }
    }else{
      dif[1] = abs ( gpstime - odotime);
      if(dif[1]>tenmillion) dif[1]=abs(dif[1]-sixtymillion); 
    }
    if(flag == 1) break;
    lD++;
  }
  //printf("lD=%d\n",lD);
  if(lD>=30){
    printf("LDerror\n");
    //exit(0);
  }
  fsetpos(U->fp,&fpos);



  double *aa=(double*)malloc(sizeof(double)*lD);
  if(U->class==1){//P3AT by yoshinaga
    double x=0,y=0,a=0,tmp;
    int l,sec,usec,odoend,min;
    for(l=0;l<lD;l++){
      ret=fgets(buff,64,U->fp); 
      if(feof(U->fp)) break;
      double _x,_y,_a;

      sscanf(buff,"%lf%lf%lf %lf:%d:%d.%d\n",&_x,&_y,&_a,&tmp,&min,&sec,&usec);
      {//if(1==0){
	for(;;){
	  if(_a<-180) _a+=360;
	  else if(_a>180) _a-=360;
	  else break;
	}
      }
      // printf("sec=%d,usec=%d",sec,usec);
      // -180 < _a < 180
      x+=_x;y+=_y;a+=_a;
      aa[l]=_a;
      U->l++;
    }
    odoend = U->timeA = i_min_sec_usec(min,sec,usec);//    odoend = U->timeA = min*1000000+sec*1000 + usec;
    if(loopnum!=0){
      timedif = input->DELTA = (odoend - odofirst)/mag_min100 ;// timedif = input->DELTA = (odoend - odofirst) / 1000000.0 ;
      if(timedif<0.0){
	timedif+=mag_min6;
	input->DELTA+=mag_min6;
      }
    }
    //printf("odoend=%d,odofirst=%d,timedif=%lf,input->DELTA=%lf\n",odoend,odofirst,timedif,input->DELTA);
    a/=lD;    //    a*=-1.;//counter clockwise ?
    x/=lD;
    y/=lD;
    double amin=aa[0], amax=aa[0];
    for(l=1;l<lD;l++){
      if(amin>aa[l]) amin=aa[l];
      if(amax<aa[l]) amax=aa[l];
    }
    if(amax-amin>180){
      a=0;
      for(l=0;l<lD;l++) a+=(aa[l]>0)?aa[l]:aa[l]+360; // 0 < _a < 360
      a/=lD;
    }
    free(aa);

    if(loopnum == 0){//first line
      input->v=input->omega=input->DELTA=0;
    }
    else{
      double dx=x-U->x0;
      double dy=y-U->y0;
      double da=a-U->a0;
      {
	for(;;){
	  if(da<-180) da+=360;
	  else if(da>180) da-=360;
	  else break;
	}
      }
      da*=D2R;//[deg] to [rad]
      dx/=1000;//[mm] to [m]
      dy/=1000;//[mm] to [m]
      //input->v=sqrt(dx*dx+dy*dy)/(U->T*lD);//return value
      //input->omega=da/(U->T*lD);//return value
      input->v=(sqrt(dx*dx+dy*dy)/timedif)*0.88;//return value//*0.8//初期値
      input->omega=(da/timedif)*0.97;//*1.2
    }
    U->x0=x;U->y0=y;U->a0=a;
  }
  else if(U->class==0){
    double v=0,w=0;
    int n=0,l;
    for(l=0;l<U->lD;l++){
      ret=fgets(buff,64,U->fp); 
      if(feof(U->fp)) break;
      double _v,_w;
      sscanf(buff,"%lf %lf\n",&_v,&_w);
      v+=_v;w+=_w;
      n++;U->l++;
    }
    input->v=v/n/U->T;
    input->omega=w/n/U->T;
  }
  //  sprintf(buff,"%s/INPUT.dat",FILEPASS);
//  if((input_file=fopen(buff,"r"))== NULL){
//    printf("INPUTFILE OPEN ERROR\n");
//    exit(1);
//  }
//  int ret;
//  for(cnt=0;cnt<t+1;cnt++) ret=fscanf(input_file,"%lf %lf\n",&input.v,&input.omega);
//  fclose(input_file);
  return ret;
  //  return input;
}
char *getInput1(INPUT *input,FILEDATA *U,int gpstime,int loopnum){
  char buff[64];
  char *ret;
  int lD=0,odotime,arraynum,flag=0,odofirst=0,beforetime=0;
  double timedif=0.0;
  int dif[2];
  fpos_t fpos;
  if(loopnum!=0){
    odofirst =  U->timeA;
    beforetime = U->timeA;
  }
  fgetpos(U->fp,&fpos);//現在のファイル位置を取得
  for(;;){
    fgets(buff,64,U->fp);
    if(feof(U->fp)) break;
    double tmp;
    int sec,usec,min;
   
    sscanf(buff,"%lf%lf%lf %lf:%d:%d.%d\n",&tmp,&tmp,&tmp,&tmp,&min,&sec,&usec);
    odotime=i_min_sec_usec(min,sec,usec);//    odotime=min*1000000+sec*1000 + usec;
      //    odotime=min*1000000+sec*1000 + usec;
    if(lD!=0){
      arraynum =(lD + 1) % 2;
      switch(arraynum){
      case 0:
	dif[0] = abs( gpstime - odotime);
	if(dif[0]>tenmillion) dif[0]=abs(dif[0]-sixtymillion);
	if(dif[0]>dif[1]) flag = 1;
	break;
      case 1:
	dif[1] = abs( gpstime - odotime);
	if(dif[1]>tenmillion) dif[1]=abs(dif[1]-sixtymillion);
	if(dif[1]>dif[0]) flag = 1;
	break;
      default:
	printf("異常　　終了します。\n");
      }
    }else{
      dif[1] = abs ( gpstime - odotime);
      if(dif[1]>tenmillion) dif[1]=abs(dif[1]-sixtymillion);
    }
    if(flag == 1) break;
    lD++;
  }
   if(lD>=30){
      printf("LDerror\n");
      exit(0);
    }

  fsetpos(U->fp,&fpos);

  double *aa=(double*)malloc(sizeof(double)*lD);
  if(U->class==1){//P3AT by yoshinaga
    double x=0,y=0,a=0,tmp,v=0,omega=0,dx=0,dy=0,da=0,delta_t=0;
    int l,sec,usec,min,odoend;
    for(l=0;l<lD;l++){
      ret=fgets(buff,64,U->fp); 
      if(feof(U->fp)) break;
      double _x,_y,_a,_v,_omega;
      //sscanf(buff,"%lf%lf%lf\n",&_x,&_y,&_a);// fscanf(input_file,"%lf%lf%lf\n",&x,&y,&a);
      sscanf(buff,"%lf%lf%lf %lf:%d:%d.%d\n",&_x,&_y,&_a,&tmp,&min,&sec,&usec);
      {//if(1==0){
	for(;;){
	  if(_a<-180) _a+=360;
	  else if(_a>180) _a-=360;
	  else break;
	}
      }
      // printf("sec=%d,usec=%d",sec,usec);
      // -180 < _a < 180
      if(l==0){
	x=U->x0;
	y=U->y0;
	a=U->a0;
      }
      dx=_x-x;
      dy=_y-y;
      da=_a-a;
      if(fabs(da)>fabs(_a-(a-360))) da=_a-(a-360);
      else if(fabs(da)>fabs(_a-(a+360))) da=_a-(a+360);
      x=_x;y=_y;a=_a;
      dx/=1000;
      dy/=1000;
      da*=D2R;
      delta_t= (i_min_sec_usec(min,sec,usec)-beforetime) / mag_min100;//delta_t= (min*1000000+sec*1000 + usec -beforetime) / 1000000.0;
      if(delta_t!=0.0){
	_v=sqrt(dx*dx+dy*dy)/delta_t;
	_omega=da/delta_t;
	v+=_v;omega+=_omega;
      }
      beforetime= i_min_sec_usec(min,sec,usec);
      U->l++;
    }
    odoend = U->timeA = i_min_sec_usec(min,sec,usec);//odoend = U->timeA = min*1000000+sec*1000 + usec;
    if(loopnum!=0){
      timedif = input->DELTA = (odoend - odofirst) / mag_min100 ;
      //      timedif = input->DELTA = (odoend - odofirst) / 1000000.0 ;
      if(timedif<0.0){
	timedif+=mag_min6;
	input->DELTA+=mag_min6;
      }
      //   if(timedif<=0.95) exit(0);
    }
    // printf("odoend=%d,odofirst=%d,timedif=%lf,input->DELTA=%lf\n",odoend,odofirst,timedif,input->DELTA);
    v/=lD;    //    a*=-1.;//counter clockwise ?
    omega/=lD;
    if(loopnum == 0){
      input->v=input->omega=input->DELTA=0;
    }
    else{
      input->v=v;
      input->omega=omega;
    }
    U->x0=x;U->y0=y;U->a0=a;
  }
  else if(U->class==0){
    double v=0,w=0;
    int n=0,l;
    for(l=0;l<U->lD;l++){
      ret=fgets(buff,64,U->fp); 
      if(feof(U->fp)) break;
      double _v,_w;
      sscanf(buff,"%lf %lf\n",&_v,&_w);
      v+=_v;w+=_w;
      n++;U->l++;
    }
    input->v=v/n/U->T;
    input->omega=w/n/U->T;
  }
  return ret;
}

char *getInput0(INPUT *input,FILEDATA *U){
  char buff[64];
  char *ret;
  if(U->class==1){
    double x=0,y=0,a=0;
    int n=0,l;
    for(l=0;l<U->lD;l++){
      ret=fgets(buff,64,U->fp); 
      if(feof(U->fp)) break;
      double _x,_y,_a;
      sscanf(buff,"%lf%lf%lf\n",&_x,&_y,&_a);
      {
	for(;;){
	  if(_a<-180) _a+=360;
	  else if(_a>180) _a-=360;
	  else break;
	}
      }
      x+=_x;y+=_y;a+=_a;
      U->l++;
      n++;
    }
    a/=n;    //    a*=-1.;//counter clockwise ?
    x/=n;
    y/=n;

    if(U->l == U->lD){
      input->v=input->omega=0;
    }
    else{
      double dx=x-U->x0;
      double dy=y-U->y0;
      double da=a-U->a0;
      if(1==0){
	for(;;){
	  if(da<-180) da+=360;
	  else if(da>180) da-=360;
	  else break;
	}
      }
      da*=D2R;//[deg] to [rad]
      dx/=1000;//[mm] to [m]
      dy/=1000;//[mm] to [m]
      input->v=sqrt(dx*dx+dy*dy)/(U->T*n);//
      input->omega=da/(U->T*n);
    }
    U->x0=x;U->y0=y;U->a0=a;
  }
  else if(U->class==0){
    double v=0,w=0;
    int n=0,l;
    for(l=0;l<U->lD;l++){
      ret=fgets(buff,64,U->fp); 
      if(feof(U->fp)) break;
      double _v,_w;
      sscanf(buff,"%lf %lf\n",&_v,&_w);
      v+=_v;w+=_w;
      n++;U->l++;
    }
    input->v=v/n/U->T;
    input->omega=w/n/U->T;
  }
  return ret;
}


int getGPSdatafromfile(GPSDATA *gps,FILEDATA *Z,double ang0){
  char buff[64];
  fgets(buff,64,Z->fp);
  double x,y,x2,y2,nS;
  float tmp;
  int ret=sscanf(buff,"%lf %lf %lf %f:%f:%f.%f %f %f\n",&y,&x,&nS,&tmp,&tmp,&tmp,&tmp,&tmp,&tmp);
  Z->l++;
  if(Z->nS0==-1000){
    Z->x0=x;
    Z->y0=y;
    Z->nS0=nS;
    gps->x=gps->y=0;
    gps->theta=ang0;
    gps->d=0.0;
  }
  else{
    x2=gps->x;
    y2=gps->y;
    gps->x=(x-Z->x0)*GPSx;
    gps->y=(y-Z->y0)*GPSy;
    gps->d=(gps->x-x2)*(gps->x-x2)+(gps->y-y2)*(gps->y-y2);
    if((gps->y-y2)!=0.0){
      gps->theta=atan2(gps->y-y2,gps->x-x2)*R2D;
    }else {
      if((gps->x-x2)>0.0){
	gps->theta=0;
      }else if((gps->x-x2)<0.0){
	gps->theta=180;
      }
    }
  }
 
  gps->nS=nS;
  return ret;
}
int getGPSdatafromfile1(GPSDATA *gps,FILEDATA *Z,double ang0,FILEDATA *F){
  char buff[64];
  fgets(buff,64,Z->fp);
  double x,y,x2,y2,nS;
  float tmp;
  int ret=sscanf(buff,"%lf %lf %lf %f:%f:%f.%f %f %f\n",&y,&x,&nS,&tmp,&tmp,&tmp,&tmp,&tmp,&tmp);
  Z->l++;
  if(Z->nS0==-1000){
    rewind(F->fp);
    fgets(buff,64,F->fp);
    int ret=sscanf(buff,"%lf %lf\n",&y,&x);
    Z->x0=x;
    Z->y0=y;
    Z->nS0=nS;
    gps->x=gps->y=0;
    gps->theta=ang0;
    gps->d=0.0;
  }
  else{
    x2=gps->x;
    y2=gps->y;
    gps->x=(x-Z->x0)*GPSx;
    gps->y=(y-Z->y0)*GPSy;
    gps->d=(gps->x-x2)*(gps->x-x2)+(gps->y-y2)*(gps->y-y2);
    if((gps->y-y2)!=0.0){
      gps->theta=atan2(gps->y-y2,gps->x-x2)*R2D;
    }else {
      if((gps->x-x2)>0.0){
	gps->theta=0;
      }else if((gps->x-x2)<0.0){
	gps->theta=180;
      }
    } 
  } 
  gps->nS=nS;
  return ret;
}


int getORSdatafromfile(double *angORS,FILEDATA *Z,int gpstime,int loopnum,double ang0){//OrientationSensor
  char buff[64];
  fpos_t fpos;
  int ln=0,arrynum,flag=0;
  for(;;){
    fgetpos(Z->fp,&fpos);
    if(feof(Z->fp)) break;
    double tmp;
    int sec,usec,min,orstime;
    int dif[2];
    fgets(buff,64,Z->fp);
    sscanf(buff,"%lf:%d:%d.%d\tcount:%lf,ORS:%lf,sleeptime:%lf",&tmp,&min,&sec,&usec,&tmp,&tmp,&tmp);
    orstime = i_min_sec_usec(min,sec,usec);//orstime = min*1000000+sec*1000 + usec;
    if(ln!=0){
      arrynum = (ln + 1) % 2;
      switch(arrynum){
      case 0:
	dif[0] = abs( gpstime - orstime);
	//printf("dif[0]=%d,dif[1]=%d\n",dif[0],dif[1]);
	if(dif[0]>=dif[1]) flag = 1;
	break;
      case 1:
	dif[1] = abs( gpstime - orstime);
	//printf("dif[0]=%d,dif[1]=%d\n",dif[0],dif[1]);
	if(dif[1]>=dif[0]) flag = 1;
	break;
      default:
	printf("エラー　終了します。\n");
	exit(0);
      }
    }else{
      dif[1] = abs( gpstime - orstime);
    }
    if(flag == 1) break;
    ln++;
  }
  fsetpos(Z->fp,&fpos);
  //  int cnt;
  fgets(buff,64,Z->fp);
  double ang,tmp;
  int ret = sscanf(buff,"%lf:%lf:%lf\tcount:%lf,ORS:%lf,sleeptime:%lf",&tmp,&tmp,&tmp,&tmp,&ang,&tmp);
  Z->l++;
   printf("ang=%lf\n\n",ang);
   ang-=Z->a0;

  if(loopnum==0){//first line
    Z->a0=ang;
    *angORS=ang0;
  }
  else{
    *angORS=ang0-ang;
  }
  for(;;){
    if(*angORS<-180) *angORS+=360;
    else if(*angORS>180) *angORS-=360;
    else break;
  }
  return ret;
}





/******************kinect追加***************/
int getKinectdatafromfile(KINECTDATA *kinect ,FILEDATA *K,int gpstime,double kinectang0){//OrientationSensor
  char buff[64];
  fpos_t fpos;
  int ln=0,arrynum,flag=0;
  for(;;){
    fgetpos(K->fp,&fpos);
    if(feof(K->fp)) break;
    double tmp;
    int sec,usec,min,kinecttime;
    int dif[2];
    fgets(buff,64,K->fp);
    sscanf(buff,"%lf\t%lf\t%d\t%d.%d\n",&tmp,&tmp,&min,&sec,&usec);
    kinecttime = i_min_sec_usec(min,sec,usec);//    kinecttime = min*1000000+sec*1000 + usec;
    if(ln!=0){
      arrynum = (ln + 1) % 2;
      switch(arrynum){
      case 0:
	dif[0] = abs( gpstime - kinecttime);
	//printf("dif[0]=%d,dif[1]=%d\n",dif[0],dif[1]);
	if(dif[0]>=dif[1]) flag = 1;
	break;
      case 1:
	dif[1] = abs( gpstime - kinecttime);
	//printf("dif[0]=%d,dif[1]=%d\n",dif[0],dif[1]);
	if(dif[1]>=dif[0]) flag = 1;
	break;
      default:
	printf("エラー　終了します。\n");
	exit(0);
      }
    }else{
      dif[1] = abs( gpstime - kinecttime);
    }
    if(flag == 1) break;
    ln++;
  }
  fsetpos(K->fp,&fpos);
  //  int cnt;
  fgets(buff,64,K->fp);
  double x,y,tmp;
  int ret = sscanf(buff,"%lf\t%lf\t%lf\t%lf.%lf\n",&x,&y,&tmp,&tmp,&tmp);
  // printf("kinect==%lf %lf\n\n",x,y);


  //printf("kinectang0==%lf\n\n",kinectang0);


  kinect->x=x*cos(kinectang0*PI/180)-y*sin(kinectang0*PI/180);
  kinect->y=x*sin(kinectang0*PI/180)+y*cos(kinectang0*PI/180);
  
  return ret;
}



///////////////////////////////////////////////
//実験結果（データ）の保存
///////////////////////////////////////////////
void SaveExperimentdata(PARAM *par){
  //void SaveExperimentdata(char *CL[]){
  FILE *fp;
  char buff[1000],buff2[256];
  //保存先ディレクトリを作成
  //  char *ret=NULL;
//  do{
//    ret=fgets(buff2,256,stdin);  //  scanf("%s",&buff2);
//  } while(ret=NULL);
//  printf("ENTER DIRECTRY NAME to save the result under ../result/ :");
  char *ret="\n";
//  char *ret=fgets(buff2,256,stdin);  //  scanf("%s",&buff2);
  if(ret[0]=='\n') sprintf(buff2,"%s","tmp");
//  fscanf(stdin,"%s",buff2);  //  scanf("%s",&buff2);
  //  int ret;ret=fscanf(stdin,"%s",buff2);  //  scanf("%s",&buff2);
  char cmd[256];
  char *dir="../result";
//  DIR *pdir;
//  if((pdir=opendir(dir))==NULL) {
//    char cmd[128];
//    sprintf(cmd,"mkdir %s",dir);
//    {int ret=system(cmd); if(ret<0) fprintf(stderr,"#error occured at '%s'.\n",cmd);}
//  } 
//  else closedir(pdir);
  sprintf(cmd,"mkdir %s >/dev/null 2>&1",dir);
  system(cmd);
  sprintf(buff,"%s/%s",dir,buff2);
  sprintf(cmd,"mkdir %s >/dev/null 2>&1",buff);
  system(cmd);
  printf("\nSaving to %s\n",buff);
  //地図画像の名前を変更
  sprintf(buff,"mv %s/MAPIMAGE.png %s/%s.png",par->dir,par->dir,buff2);
  //  sprintf(buff,"mv %s/MAPIMAGE.jpg %s/%s.jpg",CL[1],CL[1],buff2);
  system(buff);
  //データをコピー
  //  sprintf(buff,"cp -r %s/* ../EX_data/%s/",CL[1],buff2);  system(buff);
  //  sprintf(buff,"ln -s %s/* ../EX_data/%s/",CL[1],buff2);  system(buff);
  //不必要なデータを削除
  //  sprintf(buff,"rm -r ../EX_data/%s/LRF",buff2); system(buff);
  //  sprintf(buff,"rm ../EX_data/%s/INPUT.dat",buff2);  system(buff);
  //  sprintf(buff,"rm %s/%s.jpg",CL[1],buff2);  system(buff);

  //パラメータ記録用ファイルをOPEN
  sprintf(buff,"../result/%s/PARAM_DATA.dat",buff2);
  fp=fopen(buff,"w");
  //パラメータを書き込む
  fprintf(fp,"LRF_HEIGHT :%.0f[mm]\n",LRFH);
  fprintf(fp,"ANGLE      :%.0f[deg]\n",(double)ANG);
  fprintf(fp,"Particles  :%d\n",NofParticles);
  fprintf(fp,"ALPHA_1    :%.3f\n",A1);
  fprintf(fp,"ALPHA_2    :%.3f\n",A2);
  fprintf(fp,"ALPHA_3    :%.3f\n",A3);
  fprintf(fp,"ALPHA_4    :%.3f\n",A4);
  fprintf(fp,"BETA_F     :%.0f[mm]\n",CUT);
  fprintf(fp,"SIGMA2F    :P_H=%.2f at Z=%.0f[mm] -> %.2f[mm]\n",FSETP,LRFH,-pow(LRFH,2)/(2*log(FSETP)));
  fprintf(fp,"SIGMA2MB   :P_MB=%.2f at Y=%.0f[mm] -> %.2f[mm]\n",MBSETP,YSTD,-pow(YSTD,2)/(2*log(MBSETP)));
  fprintf(fp,"SIGMA2EE   :P_R=%.2f at R=%.0f[mm] -> %.2f[mm]\n",EESETP,RSTD,-pow(RSTD,2)/(2*log(EESETP)));
  fprintf(fp,"DELTA_T    :%.2f[sec]\n",DELTA);
  fprintf(fp,"RES(MATCH) :%d[mm]\n",par->MATCHres);//atoi(CL[5]));
  fprintf(fp,"RES(MAP)   :%d[mm]\n",par->MAPres);//atoi(CL[6]));
  //  fprintf(fp,"RES(MATCH) :%d[mm]\n",atoi(CL[5]));
  //  fprintf(fp,"RES(MAP)   :%d[mm]\n",atoi(CL[6]));
  //#ifdef PFMAP
  if(PFMAP){
    fprintf(fp,"MAP about  :USING PF\n");
  }
  else {
    //#else
  fprintf(fp,"MAP about  :ONLY INPUT\n");
  }
  //#endif
  switch(MMETHOD){
  case 1: fprintf(fp,"MATCHING   :SIMPLE AND\n");
  case 2: fprintf(fp,"MATCHING   :MEAN-NCC\n");
  case 3: fprintf(fp,"MATCHING   :M-INFORMATION\n");
  default: break;
  }
  fprintf(fp,"RESAMPLING :ESS<Particles*%.2f\n",PERCENT_RE);
  switch(LMETHOD){
  case 1: fprintf(fp,"LOCALIZED  :MOST VALUABLE LIKELIHOOD\n"); break;
  case 2: fprintf(fp,"LOCALIZED  :AVERAGE WITH LIKELIHOOD\n"); break;
  default: break;
  }
  fclose(fp);
  printf("a");
}
