//////////////////////////////////////////////////
//CONFIGURE内容をユーザに確認し，実行許可を得る
//////////////////////////////////////////////////
int makesuretouser(PARAM *par){
  //int makesuretouser(char *CL[]){
  printf("\n********************CONFIGULATION********************");
  printf("\n\n*****INPUT DATA INFORMATION*****");
  FILEDATA *U=&(par->U[0]);//odometry
  FILEDATA *F=&(par->F[0]);//gps
  FILEDATA *K=&(par->K[0]);//kinect

//追加
  //入力ファイルのオープンチェック，行数を数える -> Openする．
  if(F->fn) F->ndata=CheckandcountINPUT(F);//gps
  if(K->fn) K->ndata=CheckandcountINPUT(K);//kinect

  //入力ファイルの平均を計算
  double DR;
  int totaltime;
  if(U->fn){
    U->ndata=CheckandcountINPUT(U);//in fileRorW.h
    totaltime=U->ndata*U->T;
    int countL;
    DR=calcDR(par,U->ndata);//in pfprocess.c   //  double DR=calcDR(CL,U->ndata);//in pfprocess
    {
      int n;
      for(n=0;n<par->nZ;n++){
	FILEDATA *Z=&(par->Z[n]);
	if(Z->class==1 || Z->class==2 ){
	  Z->ndata=CheckandcountFILEDATA(Z);//in fileRorW.h
	}
	else if(Z->class==0){
	  //LRFファイルのオープンチェック，ファイル数を数える
	  countL=CheckandcountLRF(par);//in fileRorW.h
	  totaltime=min(U->ndata,countL);
	}
      }
    }
    printf("\n\n*****LRF DATA INFORMATION*****");
    printf("\n\n==>Total processing time is %d =min(%d,%d)",totaltime,U->ndata,countL);
  }

  //set initial potition
  printf("\n\n*****INITIAL POSITION INFORMATION*****");
  printf("\nMAP WINDOW SIZE: WIDTH=%d[pixel], HEIGHT=%d[pixel]",MAPSIZE_W,MAPSIZE_H);
  if(par->offset.pos.x<0||par->offset.pos.x>MAPSIZE_W||par->offset.pos.y<0||par->offset.pos.y>MAPSIZE_H){
    //  if(atoi(CL[2])<0||atoi(CL[2])>MAPSIZE_W||atoi(CL[3])<0||atoi(CL[3])>MAPSIZE_H){
    printf("\n\nERROR:INITIAL POSITION IS OUT OF WINDOW\n");
    exit(1);
  }
  printf("\nX=%g[pixel]:Y=%g[pixel]:THETA=%g[degree from X axis] AT t=0",par->offset.pos.x,par->offset.pos.y,par->offset.theta);

  //other parameter
  if(1==0){
    printf("\n\n*****OTHER PARAMETER*****\n");
#ifdef useLRF
    printf("LRF: NUMBER OF SCAN=%d, HEIGHT=%.1f, ANGLE OF DEPRESSION=%.1f[deg]\n",lrf_data_num,LRFH,(double)ANG);
#endif
    printf("PF: NUMBER OF PARTICLES=%d, DELTA_T=%.2f[s], RAND_PARAM=[%.3f,%.3f,%.3f,%.3f]\n",NofParticles,DELTA,A1,A2,A3,A4);
    printf("MAKEIMG: SIG2Fset->P_F=%.2f AT Z=%.0f[mm],\n",FSETP,LRFH);
    printf("         SIG2MBset->P_MB=%.2f AT Y=%.0f[mm],\n",MBSETP,YSTD);
    printf("         SIG2EEsett->P_R=%.2f AT R=%.0f[mm]\n",EESETP,RSTD);
    printf("MATCHING: RESOLUTION=%d[mm], METHOD:",par->MATCHres);//  printf("MATCHING: RESOLUTION=%d[mm], METHOD:",atoi(CL[5]));
  }
  printf("MMETHOD(Map?)=");
  switch(MMETHOD){
  case 1: printf("SIMPLE AND\n"); break;
  case 2: printf("MEAN-NCC\n"); break;
  case 3: printf("M-INFORMATION\n"); break;
  case 4: printf("HU-MOMENT"); break;
  default: printf("ERROR:PARAMETER \"MMETHOD\" IS ONLY FROM 1 TO 4\n"); exit(1); break;
  }
  printf("RESAMPLING: WHEN ESS<(NUMBER OF PARTICLES)*%.0f[%%]\n",PERCENT_RE*100);
  printf("MAPPING: RESOLUTION=%d[mm]\n",par->MAPres);//  printf("MAPPING: RESOLUTION=%d[mm]\n",atoi(CL[6]));
  
  //自己位置推定手法
  printf("LMETHOD(Localization):");
  switch(LMETHOD){
  case 1: printf("MOST VALUABLE LIKELIHOOD\n"); break;
  case 2: printf("AVERAGE OF LIKELIHOOD\n"); break;
  default: printf("ERROR:PARAMETER \"LMETHOD\" IS ONLY 1 OR 2\n"); exit(1); break;
  } 
  //解像度が適切かどうかを計算
  if(DR<par->MATCHres) printf("**WARNING**:RESOLUTION(MATCHING) IS RECOMMEND OVER %.2f.\n",DR);
  //  if(DR<atof(CL[5])) printf("**WARNING**:RESOLUTION(MATCHING) IS RECOMMEND OVER %.2f.\n",DR);
  printf("\n*****************************************************\n"); 

  //Userの許可を得る
  printf("CONFIGULATION IS COMPLETED.\n");
  printf("THIS PROGRAM WILL BE STARTED WITH ABOVE CONFIGURATION.\n");
  if(1==0){
    printf("IF YOU ARE OK, PLEASE PUSH 's'+<ENTER>::");
    char KEY[2];//    char KEY[2],*ret;//
    fgets(KEY,2,stdin);
    //  scanf("%c",&KEY);
    if(KEY[0]!='s'){printf("CANCELED by typing 's'\n"); exit(1);}
  }
  return totaltime;
}

//////////////////////////////////////////////
//SLAMウィンドウにLRFデータを描画
///////////////////////////////////////////////
void DrawLrfdataonmap(IplImage* img,STATE robot,STATE offset,
	      SCAN lrf_pt[],double angle,double cutoff,double res,int time){
  img->origin=1; 
  CvPoint pt1,pt2;
  double dir;
  double p=CV_PI/180;
  if(time==0) dir=offset.theta;
  else dir=(robot.theta-90)+offset.theta;
  //robot position
  CvPoint2D64f RRz;
  RRz.x=(robot.pos.x*cos((offset.theta-90)*p)
	 -robot.pos.y*sin((offset.theta-90)*p))/res
    +offset.pos.x;
  RRz.y=(robot.pos.x*sin((offset.theta-90)*p)
	 +robot.pos.y*cos((offset.theta-90)*p))/res
    +offset.pos.y;
  pt1.x=(int)(RRz.x);
  pt1.y=(int)(RRz.y);
  //plot lrfdata
  int i;
  CvPoint2D64f LRz;
  for(i=0;i<lrf_data_num;i++){
      LRz.x=cos((dir-90)*p)*lrf_pt[i].pt.x
	-sin((dir-90)*p)*lrf_pt[i].pt.y;
      LRz.y=sin((dir-90)*p)*lrf_pt[i].pt.x
	+cos((dir-90)*p)*lrf_pt[i].pt.y;
      if((LRz.x!=0)&&(LRz.y!=0)){
	pt2.x=(int)(pt1.x+(LRz.x)/res);
	pt2.y=(int)(pt1.y+(LRz.y)/res);
	cvLine(img,pt1,pt2,CV_RGB(255,255,255),1,8,0); 
	cvCircle(img,pt2,0,CV_RGB(0,0,0),1,-1,0);
      }
  }
}

////////////////////////////////////////////////////////
//占有格子地図作成アルゴリズムによる環境地図の作成 H23,6,20
////////////////////////////////////////////////////////
void OccupancyGridMap(IplImage* img,STATE robot,STATE offset,
	      SCAN lrf_pt[],double angle,double cutoff,double res,int time){
  img->origin=1; 
  CvPoint pt1,pt2;
  double dir;
  double p=CV_PI/180;
  if(time==0) dir=offset.theta;
  else dir=(robot.theta-90)+offset.theta;
  //robot position
  CvPoint2D64f RRz;
  RRz.x=(robot.pos.x*cos((offset.theta-90)*p)
	 -robot.pos.y*sin((offset.theta-90)*p))/res
    +offset.pos.x;
  RRz.y=(robot.pos.x*sin((offset.theta-90)*p)
	 +robot.pos.y*cos((offset.theta-90)*p))/res
    +offset.pos.y;
  pt1.x=(int)(RRz.x);
  pt1.y=(int)(RRz.y);
  //plot lrfdata
  int i;
  CvPoint2D64f LRz;
  for(i=0;i<lrf_data_num;i++){
      LRz.x=cos((dir-90)*p)*lrf_pt[i].pt.x
	-sin((dir-90)*p)*lrf_pt[i].pt.y;
      LRz.y=sin((dir-90)*p)*lrf_pt[i].pt.x
	+cos((dir-90)*p)*lrf_pt[i].pt.y;
      if((LRz.x!=0)&&(LRz.y!=0)){
	pt2.x=(int)(pt1.x+(LRz.x)/res);
	pt2.y=(int)(pt1.y+(LRz.y)/res);
	cvLine(img,pt1,pt2,CV_RGB(255,255,255),1,8,0); 
	cvCircle(img,pt2,0,CV_RGB(0,0,0),1,-1,0);
      }
  }
}


////////////////////////////////////////////
//実験結果の表示（＠gnuplot）
////////////////////////////////////////////
void Plotresult2Dline(int num,PARAM *par,int totaltime,int disp_flag){
  //void Plotresult2Dline(int num,char *CL[],int totaltime,int disp_flag){
  //gnuplotへの入力を書き込むファイルをOPEN
  FILE *fp;
  char buff[256],buff2[256];
  sprintf(buff,"%s/FIG/plotcmdfile%d.plt",par->dir,num);
  fp=fopen(buff,"w");
  switch(num){
  case 0: fprintf(fp,"set xlabel \"x[m]\";set ylabel \"y[m]\"\n");
    fprintf(fp,"set size ratio -1\n");
    //    fprintf(fp,"set xrange [-4000:4000]\n");
    //  fprintf(fp,"plot \"%s/log/KINECT.dat\" u ($1):($2) w l ti \"KINECT\" ls 7",par->dir);
    fprintf(fp,"plot \"%s/log/GPS.dat\" u ($1):($2) w l ti \"GPS\" ls 7 lw 2",par->dir);
    fprintf(fp,", \"%s/position_onlyU.dat\" u ($1/1000):($2/1000) w l ti \"Inputonly\" ls 3 lw 2",par->dir);
    fprintf(fp,", \"%s/log/TRUE.dat\" u ($1):($2) w l ti \"TRUE\" ls 2 lw 2",par->dir);
    fprintf(fp,", \"%s/position_usingPF.dat\" u ($1/1000):($2/1000) w l ti \"usingPF\" ls 5 lw 2",par->dir);
    //1
    fprintf(fp,",\"%s/log/KINECT.dat\" u ($1):($2) w l ti \"KINECT\" ls 1 lw 2",par->dir);
    sprintf(buff2,"STATE_ANGLE"); break;
    sprintf(buff2,"POSITION"); break;
  case 1: fprintf(fp,"set xlabel \"time[*%.2f s]\";set ylabel \"orientation[deg]\"\n",DELTA);
    //    fprintf(fp,"set xrange [0:%d]\n",totaltime);
    fprintf(fp,"plot \"%s/position_usingPF.dat\" u 4:3 w l ti \"usingPF\" ls 1",par->dir);
    fprintf(fp,", \"%s/position_onlyU.dat\" u 4:3 w l ti \"Inputonly\" ls 3",par->dir);
    fprintf(fp,", \"%s/log/ORS.dat\" u 2:1 w l ti \"ORS\" ls 4",par->dir);
    //fprintf(fp,",\"%s/log/GPS.dat\" u 4:3 w l ti \"GPS\" ls 2",par->dir);

    //2
    fprintf(fp,",\"%s/log/KINECT.dat\" u 4:3 w l ti \"KINECT\" ls 1",par->dir);
    sprintf(buff2,"STATE_ANGLE"); break;
  case 2: fprintf(fp,"set xlabel \"time[*%.2f s]\";set ylabel \"ESS\"\n",DELTA);
    //    fprintf(fp,"set xrange [0:%d]\n",totaltime);
    fprintf(fp,"plot \"%s/log/ESS.dat\" u 1:2 w l ti \"ESS\" ls 6",par->dir);
    sprintf(buff2,"ESS"); break;
  case 3: fprintf(fp,"set xlabel \"time[*%.2f s]\";set ylabel \"StandardDeviation of x[m]\"\n",DELTA);
    //    fprintf(fp,"set xrange [0:%d]\n",totaltime);
    fprintf(fp,"plot \"%s/log/VARIANCE.dat\" u 1:(sqrt($2)/1000.) w l ti \"STD\" ls 1",par->dir);
    sprintf(buff2,"VAR_X"); break;
  case 4: fprintf(fp,"set xlabel \"time[*%.2f s]\";set ylabel \"StandardDeviation of y[m]\"\n",DELTA);
    //    fprintf(fp,"set xrange [0:%d]\n",totaltime);
    fprintf(fp,"plot \"%s/log/VARIANCE.dat\" u 1:(sqrt($3)/1000.) w l ti \"STD\" ls 1",par->dir);
    sprintf(buff2,"VAR_Y"); break;
  case 5: fprintf(fp,"set xlabel \"time[*%.2f s]\";set ylabel \"StandardDeviation of orientation[deg]\"\n",DELTA);
    //    fprintf(fp,"set xrange [0:%d]\n",totaltime);
    fprintf(fp,"plot \"%s/log/VARIANCE.dat\" u 1:(sqrt($4)) w l ti \"VARIANCE\" ls 1",par->dir);
    sprintf(buff2,"VAR_THETA"); break;
  
   case 6: fprintf(fp,"set xlabel \"time[*%.2f s]\";set ylabel \"alfa\"\n",DELTA);
    fprintf(fp,"plot \"%s/log/ALFA.dat\" u 2:1 w l ti \"alfa\" ls 1",par->dir);
    sprintf(buff2,"alfa"); break;

    case 7: fprintf(fp,"set xlabel \"time[*%.2f s]\";set ylabel \"true_dis[m]\"\n",DELTA);
    fprintf(fp,"plot \"%s/log/TRUE_DIS.dat\" u 3:1 w l ti \"true_dis\" ls 1",par->dir);
    sprintf(buff2,"true_dis"); break;
  }
  fprintf(fp,"\nset terminal tgif");
  fprintf(fp,"\nset output \"%s/FIG/%s.obj\"",par->dir,buff2);
  fprintf(fp,"\nreplot\nset terminal x11");
  //  if(disp_flag==1) fprintf(fp,"\npause -1 \"Press enter.\"\n");
  fprintf(fp,"\npause -1 \"Press enter to quit. See %s to replot %s.\"\n",buff,buff2);
  fclose(fp);
  //gnuplotの実行
  char cmd[256];
  sprintf(cmd,"xterm -geometry 40x5-0-150 -T \"%s\" -e gnuplot -geometry 300x240 %s&",buff2,buff);
  system(cmd);
}
