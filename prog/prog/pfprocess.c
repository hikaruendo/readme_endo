////////////////////////////////////////////////////////////////////
//前時刻でリサンプル後のパーティクルを現時刻のパーティクルにコピー
////////////////////////////////////////////////////////////////////
void copyParticle(PARTICLE* pos_k,PARTICLE* pos_resample){
  int j;
  for(j=0;j<NofParticles;j++){
    pos_k[j].state.pos.x=pos_resample[j].state.pos.x;
    pos_k[j].state.pos.y=pos_resample[j].state.pos.y;
    pos_k[j].state.pos.z=pos_resample[j].state.pos.z;
    pos_k[j].state.theta=pos_resample[j].state.theta;
    pos_k[j].weight=pos_resample[j].weight;
  }  
}

/////////////////////////////////////////////////
//計測予測画像生成のための差分ベクトルの計算
/////////////////////////////////////////////////
STATE calcdiffforZpredict(STATE robot, PARTICLE* pos_predict, int num){
  STATE diff;
  diff.pos.x=pos_predict[num].state.pos.x-robot.pos.x;
  diff.pos.y=pos_predict[num].state.pos.y-robot.pos.y;
  diff.pos.z=pos_predict[num].state.pos.z-robot.pos.z;
  diff.theta=pos_predict[num].state.theta-robot.theta;
  return diff;
}

////////////////////////////////////////
//自己位置推定値の決定
////////////////////////////////////////
STATE Localization(PARTICLE* pos_resample,int method){
  STATE robot;
  robot.pos.x=robot.pos.y=robot.pos.z=0;
  robot.theta=90;
  //  double M=NofParticles;
  //尤度最大点を利用
  if(method==1){
    int i,max_weight_num;
    double max_weight;
    for(i=0,max_weight=0,max_weight_num=0;i<NofParticles;i++){
      if(max_weight<pos_resample[i].weight){
	max_weight=pos_resample[i].weight;
	max_weight_num=i;
      }
    }    
    robot.pos.x=pos_resample[max_weight_num].state.pos.x;
    robot.pos.y=pos_resample[max_weight_num].state.pos.y;
    robot.pos.z=pos_resample[max_weight_num].state.pos.z;
    robot.theta=pos_resample[max_weight_num].state.theta;
  }
  //平均を利用
  if(method==2){//default
    int i;
    double rad=0;
    double sum_x=0,sum_y=0,sum_z=0,sum_p_x=0,sum_p_y=0,sum_p_z=0;
    for(i=0;i<NofParticles;i++){
      sum_x+=pos_resample[i].weight*pos_resample[i].state.pos.x;
      sum_y+=pos_resample[i].weight*pos_resample[i].state.pos.y;
      sum_z+=pos_resample[i].weight*pos_resample[i].state.pos.z;
//      if(i<10){
//        printf("z=%.1f\n",pos_resample[i].state.pos.z);
//      }

      rad = pos_resample[i].state.theta * D2R;
      sum_p_x+=pos_resample[i].weight*cos(rad);
      sum_p_y+=pos_resample[i].weight*sin(rad);
      sum_p_z+=pos_resample[i].weight*sin(rad);
    }
    robot.pos.x=sum_x;
    robot.pos.y=sum_y;
    robot.pos.z=sum_z;
    //    printf("robotz=%.1f\n",robot.pos.z);o
    robot.theta=atan2(sum_p_y,sum_p_x) * R2D;
  }
  return robot;
}    

/////////////////////////////////////
//雑音がない場合の次時刻の姿勢を計算
/////////////////////////////////////
STATE calcposNonNoise(INPUT input, STATE robopos){//calculate dynamics
  STATE nextpos;
  double r;//  double r,p=CV_PI/180;
  if(input.type==2){
    nextpos.pos.x=robopos.pos.x+input.V[0]*input.DELTA;
    nextpos.pos.y=robopos.pos.y+input.V[1]*input.DELTA;
    nextpos.pos.z=robopos.pos.z+input.V[2]*input.DELTA;
    nextpos.ang.x=input.O[0];//3d orientation
    nextpos.ang.y=input.O[1];
    nextpos.ang.z=input.O[2];
    nextpos.theta=robopos.theta+input.omega*input.DELTA*R2D;//for 2D compatibility
    //    printf("nonoize:%g %g -> %g %g\n",robopos.pos.x,robopos.pos.y,nextpos.pos.x,nextpos.pos.y);
  }
  else{
    if(input.omega!=0){ 
      r=m2mm*input.v/input.omega;//(v,omega)[m,rad]-->pos,theta[mm,deg]???
      //    r=1000*input.v/input.omega;//(v,omega)[m,rad]-->[mm,rad]
      //x'=x+rhat*{sin(theta+omhat*DELTAT)-sin(theta)}
      nextpos.pos.x
        =robopos.pos.x
        -r*sin(robopos.theta*D2R)
        +r*sin(robopos.theta*D2R+input.omega*input.DELTA); //DELTA dt
      //y'=y+rhat*{-cos(theta+omhat*DELTAT)+cos(theta)}
      nextpos.pos.y
        =robopos.pos.y
        +r*cos(robopos.theta*D2R)
        -r*cos(robopos.theta*D2R+input.omega*input.DELTA);
      nextpos.pos.z
        =robopos.pos.z
        +r*cos(robopos.theta*D2R)
        -r*cos(robopos.theta*D2R+input.omega*input.DELTA);
      //theta'=thata+(omhat+gamhat)*DELTAT
      nextpos.theta=robopos.theta+input.omega*input.DELTA*R2D;
    }
    else{
      nextpos.pos.x
        =robopos.pos.x+cos(robopos.theta*D2R)*m2mm*input.v*input.DELTA; 
      nextpos.pos.y
        =robopos.pos.y+sin(robopos.theta*D2R)*m2mm*input.v*input.DELTA;
      nextpos.theta=robopos.theta;
      nextpos.pos.z
        =robopos.pos.z+sin(robopos.theta*D2R)*m2mm*input.v*input.DELTA;
      nextpos.theta=robopos.theta;
    }
    for(;;){
      if(nextpos.theta<-180) nextpos.theta+=360;
      else if(nextpos.theta>180) nextpos.theta-=360;
      else break;
    }
    //    printf("nonoize:%g %g -> %g %g\n",robopos.pos.x,robopos.pos.y,nextpos.pos.x,nextpos.pos.y);
  }
  return nextpos;
}

//////////////////////////////////////////
//リサンプリング
//////////////////////////////////////////
int resampling(FILE* fp,PARTICLE* pos_predict,PARTICLE* pos_resample,int t){
  int i,m,resampling_flag=0;
  double ESS=0,r,U,c,M=NofParticles;
  //リサンプル実行の可否を決める有効サンプルサイズ(ESS)を計算
  for(i=0;i<NofParticles;i++) ESS+=pow(pos_predict[i].weight,2);
  ESS=1/ESS;
  //ESSをファイルに記録
  //printf("AAABBBCCC\n");
  fprintf(fp,"%d %f\n",t,ESS);
  //ESS<パーティクル数の半分であればリサンプリング
  if(ESS<(M*PERCENT_RE)) resampling_flag=1;
  //リサンプリング
  if(resampling_flag==1){ 
    r=rand0to1(rand())/M;
    c=pos_predict[0].weight;
    i=0;
    for(m=0;m<NofParticles;m++){
      U=r+m/M;
      while(U>c){
	i++;
	c+=pos_predict[i].weight;
      }
      pos_resample[m].state.pos.x=pos_predict[i].state.pos.x;
      pos_resample[m].state.pos.y=pos_predict[i].state.pos.y;
      pos_resample[m].state.pos.z=pos_predict[i].state.pos.z;
      pos_resample[m].state.theta=pos_predict[i].state.theta;
      pos_resample[m].weight=1/M;
    }
  }
  //リサンプリングをしないとき
  else{
    for(i=0;i<NofParticles;i++){
      pos_resample[i].state.pos.x=pos_predict[i].state.pos.x;
      pos_resample[i].state.pos.y=pos_predict[i].state.pos.y;
      pos_resample[i].state.pos.z=pos_predict[i].state.pos.z;
//      if(i<10){
//        printf("z=%.1f\n",pos_resample[i].state.pos.z);
//      }
      pos_resample[i].state.theta=pos_predict[i].state.theta;
      pos_resample[i].weight     =pos_predict[i].weight; 
    }
  }
//  {//check by kuro
//    double sum_w=0;
//    for(i=0;i<NofParticles;i++){
//      sum_w+=pos_resample[i].weight;
//    }
//    fprintf(stderr,"check sum_w=%g\n",sum_w);
//  }

  return resampling_flag;
}

///////////////////
//尤度正規化
///////////////////
void weightnormalize(PARTICLE* ptcl,double sum){
  int i;
  for(i=0;i<NofParticles;i++){
    if(sum==0){
      //    if(1==0){
      printf("WEIGHTS DID NOT CALCULATED COLLECTLY. EXIT.\n");
      exit(1);
    }
    //sum:正規化前の尤度の総和
    else ptcl[i].weight=ptcl[i].weight/(sum+1e-20);
  }
}

//////////////////////////////////////////
//計測予測画像の生成
//////////////////////////////////////////
void makeimageforPredict(IplImage* k_img,IplImage* pred_img, STATE diff,double res){
  //計測予測画像の生成
  cvZero(pred_img);
  //回転
  int cx,cy;
  cx=k_img->width/(int)2;
  cy=k_img->height/(int)3;
  CvPoint2D32f center=cvPoint2D32f(cx,cy);
  CvMat *rotationMatrix=cvCreateMat(2,3,CV_32FC1); 
  cv2DRotationMatrix(center,-diff.theta,1,rotationMatrix);
  cvWarpAffine(k_img,pred_img,rotationMatrix,CV_INTER_LINEAR|CV_WARP_FILL_OUTLIERS,cvScalarAll(0));
  //並進
  CvPoint2D32f original[3],translate[3];
  double X=-diff.pos.x/res;
  double Y=-diff.pos.y/res;
  original[0]=cvPoint2D32f(0,0);
  original[1]=cvPoint2D32f(k_img->width,0);
  original[2]=cvPoint2D32f(0,k_img->height);
  translate[0]=cvPoint2D32f(X,Y);
  translate[1]=cvPoint2D32f(X+k_img->width,Y);
  translate[2]=cvPoint2D32f(X,Y+k_img->height);
  CvMat *affineMatrix=cvCreateMat(2,3,CV_32FC1);
  cvGetAffineTransform(original,translate,affineMatrix);
  cvWarpAffine(k_img,pred_img,affineMatrix,CV_INTER_LINEAR|CV_WARP_FILL_OUTLIERS,cvScalarAll(0));
}

///////////////////////////////////////////////////////////
//興味領域の設定(2枚の画像のマッチングに必要なROIを矩形で探索)
///////////////////////////////////////////////////////////
CvRect setROI(IplImage* predict_img,IplImage* kplus1_img){
  CvRect rect;
  IplImage *or_img=cvCloneImage(predict_img);
  cvOr(predict_img,kplus1_img,or_img,0);
  rect=cvBoundingRect(or_img,0);
  cvSetImageROI(predict_img,rect);
  cvSetImageROI(kplus1_img,rect);
  cvReleaseImage(&or_img);
  return rect;
}

//////////////////////////////////////////
//マッチングに使った2枚の画像のROIを解除
//////////////////////////////////////////
void resetROI(IplImage* imgA,IplImage* imgB){
  cvResetImageROI(imgA);
  cvResetImageROI(imgB);
}

//////////////////////////////////////////
//次時刻での姿勢予測
//////////////////////////////////////////
void predict_pos(INPUT input,PARTICLE* pos_k,PARTICLE* pos_predict){
  int i; 
  //  double p=CV_PI/180;
  double v_hat; //velocity included noise[mm/s]
  double w_hat; //angular velocity included noise[rad/s]
  double g_hat; //angular noise [rad/s]
  double v_sigma; //variance of v[m/s]
  double w_sigma; //variance of w[rad/s]
  double g_sigma; //variance of g[rad/s]
  double r_hat; 

  double x_slip;//slip
  double y_slip;//slip
  double theta_slip;//slip

  if(input.type==2){//3D
    double Vsigma[3]={
      sigma_v,//0.01;
      w_sigma=sigma_w,//0.1; //0.1 good
      g_sigma=sigma_g
    };//0.01;
    double Xsimga[3]={
      sigma_x,
      sigma_y,
      sigma_theta
    };
    for(i=0;i<NofParticles;i++){
//      pos_predict[i].state.pos.x=pos_k[i].state.pos.x
//	+m2mm*(input.DELTA*input.V[0]*(1.+randgauss(0,Vsigma[0],rand()))+Xsimga[0]);
//      pos_predict[i].state.pos.y=pos_k[i].state.pos.y
//	+m2mm*(input.DELTA*input.V[1]*(1.+randgauss(0,Vsigma[1],rand()))+Xsimga[1]);
//      pos_predict[i].state.pos.z=pos_k[i].state.pos.z
//	+m2mm*(input.DELTA*input.V[2]*(1.+randgauss(0,Vsigma[2],rand()))+Xsimga[2]);
      double *p1=&(pos_predict[i].state.pos.x);
      double *p0=&(pos_k[i].state.pos.x);
      int j;
      double theta=atan2(input.V[1],input.V[0]);
      double cost=cos(theta);
      double sint=sin(theta);
      double V01=sqrt(input.V[0]*input.V[0]+input.V[1]*input.V[1]);
      double phi=PI_2-atan2(V01,input.V[2]);
      double cosp=cos(phi);
      double sinp=sin(phi);
      double rv[3],rs[3],rnd0,rnd1,rnd2,rv0[3],rs0[3];
      //      double b0=0.1;//mu=V/2-b0
      //      double b1=1.-b0;
      //      double mu=-0.1;
      double mu=-0.1;
      rnd0=randgauss(Vmu[0],Vsigma[0],0);//randgauss(double mean, double sig, int rngpara) in share/easyfunc.c
      rnd1=randgauss(Vmu[1],Vsigma[1],0);
      rnd2=randgauss(Vmu[2],Vsigma[2],0);
      rv0[0]=cost*rnd0-sint*rnd1;
      rv0[1]=sint*rnd0+cost*rnd1;
      rv0[2]=rnd2;
      rv[0]=cosp*rv0[0]-sinp*rv0[2];
      rv[1]=rv0[1];
      rv[2]=sinp*rv0[0]+cosp*rv0[2];
      //
//      rnd0=randgauss(-b0*Xsimga[0],b1*Xsimga[0],rand());
//      rnd1=randgauss(-b0*Xsimga[1],b1*Xsimga[1],rand());
//      rnd2=randgauss(-b0*Xsimga[2],b1*Xsimga[2],rand());
      rnd0=randgauss(Xmu[0]*m2mm,Xsimga[0]*m2mm,0);
      rnd1=randgauss(Xmu[1]*m2mm,Xsimga[1]*m2mm,0);
      rnd2=randgauss(Xmu[2]*m2mm,Xsimga[2]*m2mm,0);
      rs0[0]=cost*rnd0-sint*rnd1;
      rs0[1]=sint*rnd0+cost*rnd1;
      rs0[2]=rnd2;
      rs[0]=cosp*rs0[0]-sinp*rs0[2];;
      rs[1]=rs0[1];
      rs[2]=sinp*rs0[0]+cosp*rs0[2];

      for(j=0;j<3;j++){
	p1[j]=p0[j]+input.DELTA*(input.V[j]*(1.+rv[j])+rs[j]);
	//	p1[j]=p0[j]+(input.DELTA*input.V[j]*(1.)+0);
	//p1[j]=p0[j]+(input.DELTA*input.V[j]*(1.+randgauss(0,Vsigma[j],rand()))+Xsimga[j]);
	//	p1[j]=p0[j]+m2mm*(input.DELTA*input.V[j]*(1.+randgauss(0,Vsigma[j],rand()))+Xsimga[j]);
	//	p1[j]=p0[j]+Vhat[j]=m2mm*(input.V[j]*(1.+randgauss(0,Vsigma[j],rand()))+Xsimga[j]);
      }
      pos_predict[i].state.theta=atan2(p1[1]-p0[1],p1[0]-p0[0])*R2D;
      //      pos_k[i].state.pos.z*=0.7;
      if(i<5){
	printf("PF%d:%.1f %.1f %.1f-> ",i,pos_k[i].state.pos.x,pos_k[i].state.pos.y,pos_k[i].state.pos.z);
	printf("%.1f %.1f %.1f %d;\n",pos_predict[i].state.pos.x,pos_predict[i].state.pos.y,pos_predict[i].state.pos.z,i);
      }
      //      printf("PF:%.1f %.1f-> ",pos_k[i].state.pos.x,pos_k[i].state.pos.y);
      //      printf("%.1f %.1f %d;\n",pos_predict[i].state.pos.x,pos_predict[i].state.pos.y,i);
      //      printf("Delta%.1f %.1f %.1f\n",input.DELTA*input.V[0],input.DELTA*input.V[1],input.DELTA*input.V[2]);
      //      printf("PF:%g %g -> %g %g;",robopos.pos.x,robopos.pos.y,nextpos.pos.x,nextpos.pos.y);
    }
  }
  else{//2D
    v_sigma=A1*fabs(input.v);
    w_sigma=A2*fabs(input.omega);
    g_sigma=A3*fabs(input.omega)+A4;
  
    v_sigma=sigma_v;//0.01;
    w_sigma=sigma_w;//0.1; //0.1 good
    g_sigma=sigma_g;//0.01;
    
    x_slip=sigma_x;
    y_slip=sigma_y;
    theta_slip=sigma_theta;
  
    for(i=0;i<NofParticles;i++){
      //addition of noise
      v_hat=m2mm*input.v+randgauss(0,v_sigma,rand());//v[m]->[mm]
      w_hat=input.omega+randgauss(0,w_sigma,rand());
      g_hat=randgauss(0,g_sigma,rand());
  
  //    v_hat=1000*(input.v+randgauss(0,v_sigma,rand()));//input.v[mm]?
  //    w_hat=input.omega+randgauss(0,w_sigma,rand());
  //    g_hat=randgauss(0,g_sigma,rand());
      if(w_hat!=0){ 
        r_hat=v_hat/w_hat;      
        //x'=x+rhat*{sin(theta+omhat*DELTAT)-sin(theta)}
        pos_predict[i].state.pos.x=pos_k[i].state.pos.x
  	-r_hat*sin(pos_k[i].state.theta*D2R)+r_hat*sin(pos_k[i].state.theta*D2R+w_hat*input.DELTA)+randgauss(0,x_slip,rand());
        //y'=y+rhat*{-cos(theta+omhat*DELTAT)+cos(theta)}
        pos_predict[i].state.pos.y=pos_k[i].state.pos.y
  	+r_hat*cos(pos_k[i].state.theta*D2R)-r_hat*cos(pos_k[i].state.theta*D2R+w_hat*input.DELTA)+randgauss(0,y_slip,rand());;
        //theta'=theta+(omhat+gamhat)*DELTAT
        pos_predict[i].state.theta=pos_k[i].state.theta + (w_hat+g_hat)*input.DELTA*R2D+randgauss(0,theta_slip,rand());;
      }
      else{
        pos_predict[i].state.pos.x=pos_k[i].state.pos.x+cos(pos_k[i].state.theta*D2R)*v_hat*input.DELTA+randgauss(0,x_slip,rand());; 
        pos_predict[i].state.pos.y=pos_k[i].state.pos.y+sin(pos_k[i].state.theta*D2R)*v_hat*input.DELTA+randgauss(0,y_slip,rand());;
        pos_predict[i].state.theta=pos_k[i].state.theta+g_hat*input.DELTA+randgauss(0,theta_slip,rand());;
      }
      pos_predict[i].weight=pos_k[i].weight;
      for(;;){
        if(pos_predict[i].state.theta<-180) pos_predict[i].state.theta+=360;
        else if(pos_predict[i].state.theta>180) pos_predict[i].state.theta-=360;
        else break;
      }
      printf("PF:%.1f %.1f-> ",pos_k[i].state.pos.x,pos_k[i].state.pos.y);
      printf("%.1f %.1f %d;\n",pos_predict[i].state.pos.x,pos_predict[i].state.pos.y,i);
    }
  } //else{
}

/////////////////
//解像度チェック
/////////////////
double calcDR(PARAM *par, int countU){  //double calcDR(char *CL[], int countU){
//  FILE *fp;
//  if((fp=fopen(par->ACTfn,"r"))==NULL){
//    printf("ERROR:CAN'T OPEN FILE (%s) :CHECK THE argv[1]\n",par->ACTfn);
//    exit(1);
//  }
  //  char buff[64];
  int i,cnt=0;
  //  sprintf(buff,"%s/INPUT.dat",CL[1]);
  double DR=0,Dw=0;
  //  int ret;
  FILEDATA *U=&(par->U[0]);
  rewind(U->fp);

  if(U->class==1){//x,y,ang
    double Dax=0,Day=0;int nDa=0;//kuro
    char buff[64];
    double x,y,a,x0,y0,a0;
    fgets(buff,64,U->fp);
    sscanf(buff,"%lf%lf%lf",&x,&y,&a);
    x0=x;y0=y;a0=a;
    int nv=0,nw=0;
    for(i=1;i<countU;i++){
      fgets(buff,64,U->fp);
      if(feof(U->fp)) break;
      sscanf(buff,"%lf %lf %lf",&x,&y,&a);
      double dx=x-x0,dy=y-y0;
      double dr=sqrt(dx*dx+dy*dy);
      if(dr>0){
	double v= dr;
	//	DR+=fabs(v);//DELTA=0.1
	DR+=m2mm*fabs(v)*DELTA;//DELTA=0.1
	nv++;
      }
      double da=a-a0;
      Dax+=cos(da*D2R);//kuro
      Day+=sin(da*D2R);//kuro
      nDa++;
      for(;;){
	if(da<-180) da+=360;
	else if(da>180) da-=360;
	else break;
      }
      double w=fabs(da);
      if(w>0){
	Dw+=w;
	nw++;
      }
      x0=x;y0=y;a0=a;
    }
    DR*=(0.001*DELTA/U->T);
    Dw*=(DELTA/U->T);
    Dw/=nw;
    DR/=nv;
    printf("\nINPUT AVERAGE(FABS):V=%.3f[m/s], OMEGA=%.3f(=%.3f)[deg/s]",DR,Dw,atan2(Day/nDa,Dax/nDa)*R2D*(DELTA/U->T));
  }
  else {//for iRobot v,omega ??
    double v,omega,barv=0,barom=0;
    for(i=0;i<countU;i++){
      fscanf(U->fp,"%lf %lf\n",&v,&omega);
      barv+=fabs(v); 
      barom+=fabs(omega);
      DR+=m2mm*fabs(v)*DELTA;
      if(omega!=0) cnt++;
    }
    barv/=countU;
    barom/=cnt;
    printf("\nINPUT AVERAGE(FABS):V=%.3f[m/s], OMEGA=%.3f[rad/s]",barv,barom);
    DR/=countU;
  }
  return DR;
}

///////////////////////////////////////////////////
//パーティクルのばらつきを計算
///////////////////////////////////////////////////
STATE calc_var(STATE meanstate, PARTICLE* pos){
  STATE variance;
  int i;
  double sig2x=0,sig2y=0,sig2p=0,M=NofParticles;
  for(i=0;i<NofParticles;i++){
    //xの分散
    sig2x+=pow((meanstate.pos.x-pos[i].state.pos.x),2);
    //yの分散
    sig2y+=pow((meanstate.pos.y-pos[i].state.pos.y),2);
    //thetaの分散
    sig2p+=pow((meanstate.theta-pos[i].state.theta),2);
  }
  variance.pos.x=sig2x/M;
  variance.pos.y=sig2y/M;
  variance.theta=sig2p/M;
  //  fprintf(stderr,"#variance=%g %g %g\n",variance.pos.x,variance.pos.y,variance.theta);
  //  fprintf(stdout,"##variance=%g %g %g\n",variance.pos.x,variance.pos.y,variance.theta);
  return variance;
}
STATE *calc_var1(STATE meanstate, PARTICLE* pos,STATE *variance){
  //  STATE variance;
  int i;
  double sig2x=0,sig2y=0,sig2p=0,M=NofParticles;
  for(i=0;i<NofParticles;i++){
    //xの分散
    sig2x+=pow((meanstate.pos.x-pos[i].state.pos.x),2);
    //yの分散
    sig2y+=pow((meanstate.pos.y-pos[i].state.pos.y),2);
    //thetaの分散
    sig2p+=pow((meanstate.theta-pos[i].state.theta),2);
  }
  variance->pos.x=sig2x/M;
  variance->pos.y=sig2y/M;
  variance->theta=sig2p/M;
  //  fprintf(stderr,"#variance=%g %g %g\n",variance->pos.x,variance->pos.y,variance->theta);
  //  fprintf(stdout,"##variance=%g %g %g\n",variance.pos.x,variance.pos.y,variance.theta);
  return variance;
}
