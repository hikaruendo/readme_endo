////////////////////////////////////////////////
//記録用ファイルを全消去
////////////////////////////////////////////////
void resetALLlogfile(char *dir){//void resetALLlogfile(char *CL[]){
  char buff[256];
  //  int ret;
  sprintf(buff,"mkdir %s/log >/dev/null 2>&1",dir); system(buff);
  sprintf(buff,"mkdir %s/FIG >/dev/null 2>&1",dir); system(buff);
  sprintf(buff,"rm %s/log/* >/dev/null 2>&1",dir); system(buff);
  sprintf(buff,"rm %s/FIG/* >/dev/null 2>&1",dir); system(buff);
}

////////////////////////////////////////////////
//パーティクルをリセット
//x=0[mm],y=0[mm],theta=90[deg from x-axis],w=1
////////////////////////////////////////////////
void resetPFonstart(PARTICLE* pos,double ang){
  int i;
  for(i=0;i<NofParticles;i++){
    pos[i].state.pos.x=0;
    pos[i].state.pos.y=0;
    pos[i].state.theta=ang;//PI_2;//90;
    pos[i].weight=1;
  }  
}

////////////////////////////
//初期姿勢をセット(Map描画用)
////////////////////////////
//STATE setInitdrawpos(char *CL[]){
//  STATE offset;
//  offset.pos.x=atoi(CL[2]);
//  offset.pos.y=atoi(CL[3]);
//  offset.theta=atoi(CL[4]);
//  return offset;
//}

///////////////////////////////
//初期姿勢をセット(推定姿勢用)
///////////////////////////////
STATE setInitrobotpos(double ang){
  STATE robot;
  robot.pos.x=0;
  robot.pos.y=0;
  robot.theta=ang;//90
  return robot;
}

///////////////////////////////
//画像の初期設定をする
///////////////////////////////
void SetImageconfig(IplImage* img,char windowname[],int mvX,int mvY){
  //原点を左下に設定
  img->origin=1; 
  //ウィンドウの名前をつける
  cvNamedWindow (windowname,CV_WINDOW_AUTOSIZE);
  //ウィンドウを移動
  cvMoveWindow(windowname,mvX,mvY);
}

///////////////////////
//rand関数の初期化
///////////////////////
#ifdef ZMTRAND
void initrand(){int seed=0;InitMt((unsigned long)seed);}
#else
void initrand(){srand(0);}
#endif
void initrandorg(){
  int utime;
  long ltime;
  ltime = time(NULL);
  utime = (unsigned int) ltime/2;
  srand(utime);
}
