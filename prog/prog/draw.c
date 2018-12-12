///////////////////////////////////////////////////////
//現時刻でのレンジデータをウィンドウに表示(@"SCAN"window)
///////////////////////////////////////////////////////
void showdataonLRFwindow(IplImage* scan_img,SCAN lrf_k[]){   
  int i,scale=30;
  double cx,cy;
  static CvPoint pt1,ptr1,ptr2;
  CvScalar red,blue;
  red=CV_RGB(255,0,0);
  blue=CV_RGB(0,0,255);
  //スキャン画像(t=k)を描画
  cvZero(scan_img);
  cy=scan_img->height/(int)3;
  cx=scan_img->width/(int)2;
  for(i=0;i<lrf_data_num;i++){
    if((lrf_k[i].pt.x!=0)&&(lrf_k[i].pt.y!=0)){
      pt1.x=(int)(cx+(lrf_k[i].pt.x)/scale);
      pt1.y=(int)(cy+(lrf_k[i].pt.y)/scale); 
      cvCircle(scan_img,pt1,1,red,1,-1,0);
    }
  }
  //ロボットマークを描画
  int robotmark=8;
  ptr1.x=(int)(scan_img->width/2);
  ptr1.y=(int)(scan_img->height/3);
  ptr2.y=(int)(ptr1.y+robotmark);
  ptr2.x=ptr1.x;
  cvCircle(scan_img,ptr1,robotmark,blue,2,8,0);    
  cvLine(scan_img,ptr1,ptr2,blue,2,8,0);
#define kuroDISP
#ifdef kuroDISP
  //表示 
  cvShowImage("SCAN",scan_img);
#endif
}

//////////////////////////////////////////////
//SLAMウィンドウにGPSマークを描画 
//type1:Draw Estimated robot position
//type2:Draw Particle
///////////////////////////////////////////////
void DrawGPSmarkonmap(IplImage* img,GPSDATA gps,STATE offset,CvScalar color,double res,int time){
  img->origin=1; 
  CvPoint pt1;
  int radius,line;
  //  double p=CV_PI/180;
  radius=(int)(1000/res);
  line=2;

  //robot position
  //  CvPoint2D64f RRz;
  double ang=(offset.theta)*D2R;//modified by kuro
  //  double ang=(offset.theta-90)*D2R;//original ??
  double gpsx=gps.x*m2mm;
  double gpsy=gps.y*m2mm;
  pt1.x=(int)((gpsx*cos(ang) -gpsy*sin(ang))/res +offset.pos.x);
  pt1.y=(int)((gpsx*sin(ang) +gpsy*cos(ang))/res +offset.pos.y);
  //robot mark
  cvCircle(img,pt1,radius,color,line,8,0);    
}
//////////////////////////////////////////////
//SLAMウィンドウにロボットマークを描画 
//type1:Draw Estimated robot position
//type2:Draw Particle
///////////////////////////////////////////////
void DrawRobotmarkonmap(int MODE,IplImage* img,STATE robot,STATE offset,CvScalar color,double res,int time){
  img->origin=1; 
  CvPoint pt1,pt2;
  double dir;
  int radius,line;
  //  double p=CV_PI/180;
  dir=robot.theta+offset.theta;//modied by kuro
  //  if(time==0) dir=offset.theta;  else dir=(robot.theta-90)+offset.theta;
  switch(MODE){
  case 1: radius=(int)(250/res); line=2; break;
    //  case 1: radius=(int)(250/res); line=2; break;
  case 2: radius=(int)(100/res); line=1; break;
  default: printf("ロボットマークモードが違います"); exit(1); break;
  }
  //robot position
  //  CvPoint2D64f RRz;
  double ang=(offset.theta)*D2R;//modified by kuro
  //  double ang=(offset.theta-90)*D2R;//original ??
  pt1.x=(int)((robot.pos.x*cos(ang) -robot.pos.y*sin(ang))/res +offset.pos.x);
  pt1.y=(int)((robot.pos.x*sin(ang) +robot.pos.y*cos(ang))/res +offset.pos.y);
  pt2.x=(int)(pt1.x+cos(dir*D2R)*radius);
  pt2.y=(int)(pt1.y+sin(dir*D2R)*radius);
  //robot mark
  cvCircle(img,pt1,radius,color,line,8,0);    
  cvLine(img,pt1,pt2,color,line,8,0); 
}
void DrawORSmarkonmap(int mag,IplImage* img,STATE robot,STATE offset,
			CvScalar color,double res,int time){
  img->origin=1; 
  CvPoint pt1,pt2;
  double dir;
  int radius,line;
  //  double p=CV_PI/180;
  dir=robot.theta+offset.theta;//modied by kuro
  //  if(time==0) dir=offset.theta;  else dir=(robot.theta-90)+offset.theta;
  radius=(int)(mag/res); 
  line=2; 
  //robot position
  //  CvPoint2D64f RRz;
  double ang=(offset.theta)*D2R;//modified by kuro
  //  double ang=(offset.theta-90)*D2R;//original ??
  pt1.x=(int)((robot.pos.x*cos(ang) -robot.pos.y*sin(ang))/res +offset.pos.x);
  pt1.y=(int)((robot.pos.x*sin(ang) +robot.pos.y*cos(ang))/res +offset.pos.y);
  pt2.x=(int)(pt1.x+cos(dir*D2R)*radius);
  pt2.y=(int)(pt1.y+sin(dir*D2R)*radius);
  //robot mark
  cvCircle(img,pt1,radius,color,line,8,0);    
  cvLine(img,pt1,pt2,color,line,8,0); 
}
//void DrawRobotmarkonmap0(int MODE,IplImage* img,STATE robot,STATE offset,
//			CvScalar color,double res,int time){
//  img->origin=1; 
//  CvPoint pt1,pt2;
//  double dir;
//  int radius,line;
//  //  double p=CV_PI/180;
//  if(time==0) dir=offset.theta;
//  else dir=(robot.theta-90)+offset.theta;
//  switch(MODE){
//  case 1: radius=(int)(250/res); line=2; break;
//  case 2: radius=(int)(100/res); line=1; break;
//  default: printf("ロボットマークモードが違います"); exit(1); break;
//  }
//  //robot position
//  CvPoint2D64f RRz;
//  RRz.x=(robot.pos.x*cos((offset.theta-90)*D2R)
//	 -robot.pos.y*sin((offset.theta-90)*D2R))/res
//    +offset.pos.x;
//  RRz.y=(robot.pos.x*sin((offset.theta-90)*D2R)
//	 +robot.pos.y*cos((offset.theta-90)*D2R))/res
//    +offset.pos.y;
//  pt1.x=(int)(RRz.x);
//  pt1.y=(int)(RRz.y);
//  pt2.x=(int)(pt1.x+cos(dir*D2R)*radius);
//  pt2.y=(int)(pt1.y+sin(dir*D2R)*radius);
//  //robot mark
//  cvCircle(img,pt1,radius,color,line,8,0);    
//  cvLine(img,pt1,pt2,color,line,8,0); 
//}

/////////////////////////////////////////////////
//ロボットマークとLRFデータの合成(@SLAMウィンドウ)
/////////////////////////////////////////////////
void superpositionRandL(IplImage* Robotmarkimg,IplImage* LRFimg,IplImage* sppimg){
  IplImage *grayimg=cvCreateImage(cvGetSize(Robotmarkimg),8,1);
  IplImage *maskimg=cvCreateImage(cvGetSize(Robotmarkimg),8,1);
  //ロボットマークの画像をグレースケール変換
  cvCvtColor(Robotmarkimg,grayimg,CV_BGR2GRAY);
  //2値化
  cvThreshold(grayimg,maskimg,0,255,CV_THRESH_BINARY_INV);
  IplImage *inverseMaskimg=cvCreateImage(cvGetSize(Robotmarkimg),8,1);
  IplImage *extractedObjectimg=cvCreateImage(cvGetSize(Robotmarkimg),8,3);
  IplImage *extractedBackgroundimg=cvCreateImage(cvGetSize(Robotmarkimg),8,3);
  cvSetZero(extractedBackgroundimg);
  cvCopy(LRFimg,extractedBackgroundimg,maskimg);
  cvNot(maskimg,inverseMaskimg);
  cvSetZero(extractedObjectimg);
  cvCopy(Robotmarkimg,extractedObjectimg,inverseMaskimg);
  cvAdd(extractedBackgroundimg,extractedObjectimg,sppimg,NULL);
  cvReleaseImage(&grayimg);
  cvReleaseImage(&maskimg);
  cvReleaseImage(&inverseMaskimg);
  cvReleaseImage(&extractedObjectimg);
  cvReleaseImage(&extractedBackgroundimg);
#ifdef kuroDISP
  cvShowImage("SLAM",sppimg);
#endif
}

//////////////////////////////////
//ROIの矩形を描画(ユーザ提示用)
//////////////////////////////////
void DrawROIrectangle(IplImage* img,CvRect rect,char windowname[],int brightness,int spa_img_w){
  IplImage *copyimg=cvCreateImage(cvGetSize(img),8,1);
  copyimg->origin=1;
  CvPoint pt1,pt2;
  cvCopy(img,copyimg,NULL);
  //矩形描画に必要な設定(pt1:左下，pt2:右上)
  pt1.x=rect.x; pt1.y=rect.y; pt2.x=pt1.x+rect.width; pt2.y=pt1.y+rect.height;
  //ROI矩形を描画
  cvRectangle(copyimg,pt1,pt2,CV_RGB(brightness,brightness,brightness),1,8,0);
   //ウィンドウサイズに合わせた画像の拡大縮小 H23,03
  IplImage *resize_img=cvCreateImage(cvSize(spa_img_w,spa_img_w),8,1);
  resize_img->origin=1;
  //cvResize(img,resize_img,CV_INTER_NN);
  cvResize(img,resize_img,CV_INTER_LINEAR);
  //cvResize(copyimg,resize_img,CV_INTER_CUBIC);
#ifdef kuroDISP
  cvShowImage(windowname,resize_img);
#endif
  //cvShowImage(windowname,copyimg);
  cvReleaseImage(&copyimg);
  cvReleaseImage(&resize_img);
 }

///////////////////////////////////////////
//スキャンデータからマッチング用画像を生成
///////////////////////////////////////////
void makeimageforMatching(IplImage* img,char windowname[],SCAN LRFdata2D[],SCAN LRFdata3D[],double res,int spa_img_w){
  int i;
  double cx=img->width/2;
  double cy=img->height/3;
  double sig2f=-pow(LRFH,2)/(2*log(FSETP));
  double sig2m=-pow(YSTD,2)/(2*log(MBSETP));
  double sigee=sqrt(-pow(RSTD/res,2)/(2*log(EESETP)));
  double p=CV_PI/180;
  //  double r;
  CvScalar white=CV_RGB(255,255,255);
  cvZero(img);
  //床面高さを考慮したグレースケールプロット画像を生成
  for(i=0;i<lrf_data_num;i++){
    //スキャンLRFデータ(2D)の座標変換(->3D)
    LRFdata3D[i].pt.x=LRFdata2D[i].pt.x;
    LRFdata3D[i].pt.y=cos(ANG*p)*LRFdata2D[i].pt.y;
    LRFdata3D[i].pt.z=LRFH-sin(ANG*p)*LRFdata2D[i].pt.y;
    //    double r=Norm2D(LRFdata2D[i].pt.x,LRFdata2D[i].pt.y);
    CvPoint pt1;
    if(ANG==0){
      //水平方向を向いているとき
      if(LRFdata3D[i].pt.x!=0||LRFdata3D[i].pt.y!=0){
	pt1.x=(int)(cx+(LRFdata3D[i].pt.x)/res);
	pt1.y=(int)(cy+(LRFdata3D[i].pt.y)/res); 
	cvCircle(img,pt1,0,white,1,-1,0);
      }
    }
    else{
      //床面への投影確率画像の生成
      pt1.x=(int)(cx+LRFdata3D[i].pt.x/res);
      pt1.y=(int)(cy+LRFdata3D[i].pt.y/res);
      double pf,pmb,ppb;
      pf=exp(-pow(LRFdata3D[i].pt.z,2)/sig2f);
      pmb=exp(-pow(LRFdata3D[i].pt.y,2)/sig2m);
      ppb=pf*pmb;
      int g=(int)(255*ppb);
      CvScalar grayscale=CV_RGB(g,g,g);
      //床面からz座標がCUT[mm]以下のデータは除いて描画
      if(LRFdata3D[i].pt.x!=0||LRFdata3D[i].pt.y!=0){
	if(LRFdata3D[i].pt.z>CUT) cvCircle(img,pt1,0,grayscale,-1,8,0);
      }
    }
  }
  //ガウシアンフィルタをかける
  IplImage *img2=cvCloneImage(img);
  cvCopy(img2,img,NULL);
  if(ANG!=0){
    cvSmooth(img,img,CV_GAUSSIAN,0,0,sigee,sigee);
    //画素値の正規化
    int x,y;
    double maxval1=0,maxval2=0;
    for(y=0;y<img->height;y++){
      for(x=0;x<img->width;x++){
	CvScalar value1,value2;
	value1=cvGet2D(img,y,x);
	if(value1.val[0]>maxval1) maxval1=value1.val[0];
	value2=cvGet2D(img2,y,x);
	if(value2.val[0]>maxval2) maxval2=value2.val[0];
      }
    }
    for(y=0;y<img2->height;y++){
      for(x=0;x<img2->width;x++){
	CvScalar value;
	value=cvGet2D(img,y,x);
	value=cvScalarAll(maxval2/maxval1*value.val[0]);
	cvSet2D(img,y,x,value);
      }
    }
  }
   //ウィンドウサイズに合わせた画像の拡大縮小（ユーザ提示用）H23,03
  IplImage *resize_img=cvCreateImage(cvSize(spa_img_w,spa_img_w),8,1);
  resize_img->origin=1;
  //cvResize(img,resize_img,CV_INTER_NN);
  cvResize(img,resize_img,CV_INTER_LINEAR);
  //cvResize(img,resize_img,CV_INTER_CUBIC);
  cvReleaseImage(&img2);
#ifdef kuroDISP
  cvShowImage(windowname,resize_img);
#endif
  cvReleaseImage(&resize_img);
  // cvShowImage(windowname,img);
}

//////////////////////////////////////////////////////////////
//ロボット座標系から見たパーティクルの分布をウィンドウに描画
//////////////////////////////////////////////////////////////
void DrawParticleonRobotWindow(IplImage* img,INPUT input,STATE robot,PARTICLE* pos_predict,
			       STATE varp,int div){
  CvPoint RobotC,RobotT;
  CvPoint robotc,robott;
  CvPoint Yaxis1,Yaxis2,Xaxis1,Xaxis2;
  CvPoint pt1,pt2,pt3,pt4,pt5,pt6;
  STATE nextpos;
  double r,wmax=0;//  double r,p=CV_PI/180,wmax=0;
  //  double p=CV_PI/180;
  int i,max=0,Radius=10,radius=4;
  //動かさない点の定義
  cvZero(img);
  RobotC.x=(img->width)/2;  RobotC.y=(img->height)/3;
  RobotT.x=RobotC.x;  RobotT.y=RobotC.y+Radius;
  Yaxis1.x=RobotC.x; Yaxis1.y=img->height;
  Yaxis2.x=RobotC.x; Yaxis2.y=0;
  Xaxis1.x=0; Xaxis1.y=RobotC.y;
  Xaxis2.x=img->width; Xaxis2.y=RobotC.y;
  //座標軸を描く
  cvLine(img,Xaxis1,Xaxis2,CV_RGB(128,128,128),1,8,0); 
  cvLine(img,Yaxis1,Yaxis2,CV_RGB(128,128,128),1,8,0);
  //ロボットマークを描く
  cvLine(img,RobotC,RobotT,CV_RGB(0,0,255),2,8,0); 
  cvCircle(img,RobotC,Radius,CV_RGB(0,0,255),2,-1,0);
  //雑音のない次時刻位置を描画
  if(input.omega!=0){ 
    r=1000*input.v/input.omega;
    nextpos.pos.x=-r+r*sin(CV_PI/2+input.omega*DELTA);
    nextpos.pos.y=-r*cos(CV_PI/2+input.omega*DELTA);
    nextpos.theta=CV_PI/2+input.omega*DELTA;
  }
  if(input.omega==0){ 
    nextpos.pos.x=0;
    nextpos.pos.y=1000*input.v*DELTA;
    nextpos.theta=CV_PI/2;
  }
  double staticR=(img->height)/2.5;
  double vR=Norm2D(nextpos.pos.x,nextpos.pos.y);
  double interval=staticR/div;
  double scale=staticR/vR; 
  robotc.x=RobotC.x+scale*nextpos.pos.x; 
  robotc.y=RobotC.y+scale*nextpos.pos.y;
  robott.x=robotc.x+Radius*cos(nextpos.theta);
  robott.y=robotc.y+Radius*sin(nextpos.theta); 
  cvLine(img,robotc,robott,CV_RGB(128,128,128),2,8,0); 
  cvCircle(img,robotc,Radius,CV_RGB(128,128,128),2,8,0);
  //等円を描く
  for(i=0;i<div+2;i++){
    int R;
    R=staticR+i*interval;
    cvCircle(img,RobotC,R,CV_RGB(128,128,128),1,-1,0);
    R=staticR-i*interval;
    if(R>0) cvCircle(img,RobotC,R,CV_RGB(128,128,128),1,-1,0);  }
  //参考円を書く
  cvCircle(img,robotc,interval,CV_RGB(0,0,180),1,-1,0);
  //矢印を描く
  int arsize=interval/10;
  pt1.x=RobotC.x-interval; pt2.x=RobotC.x-2*interval; pt1.y=pt2.y=RobotC.y;
  cvLine(img,pt1,pt2,CV_RGB(255,255,0),1,8,0); 
  pt3.x=pt4.x=pt1.x-arsize; pt3.y=pt1.y+arsize; pt4.y=pt1.y-arsize;
  cvLine(img,pt1,pt3,CV_RGB(255,255,0),1,8,0); cvLine(img,pt1,pt4,CV_RGB(255,255,0),1,8,0); 
  pt5.x=pt6.x=pt2.x+arsize; pt5.y=pt2.y+arsize; pt6.y=pt2.y-arsize;
  cvLine(img,pt2,pt5,CV_RGB(255,255,0),1,8,0); cvLine(img,pt2,pt6,CV_RGB(255,255,0),1,8,0); 
  //数値を描画
  CvFont font[10]; char buff[256];
  cvInitFont(font,CV_FONT_HERSHEY_DUPLEX,0.5f,0.5f,0,1,8);
  sprintf(buff,"%.0f[mm]",vR/div);
  cvPutText(img,buff,cvPoint(pt2.x,pt6.y-20),font,CV_RGB(255,255,0));
//  cvPutText(img,buff,cvPoint(pt2.x,pt6.y-20),&font,CV_RGB(255,255,0));
  //最大尤度のパーティクルのインデックスを探索
  for(i=0;i<NofParticles;i++){if(pos_predict[i].weight>wmax){wmax=pos_predict[i].weight; max=i;}}
  //パーティクル描画
  for(i=0;i<NofParticles;i++){
    CvPoint2D64f dS;//    CvPoint2D64f trns,dS;
    dS.x=pos_predict[i].state.pos.x-robot.pos.x;
    dS.y=pos_predict[i].state.pos.y-robot.pos.y;
    double dp=(robot.theta)*D2R;//modified by kuro
    //    double dp=(robot.theta-90)*D2R;
    /*
    trns.x=dS.x;//cos(-dp)*dS.x-sin(-dp)*dS.y;
    trns.y=dS.y;//sin(-dp)*dS.x+cos(-dp)*dS.y;
    CvPoint ptp;
    ptp.x=scale*trns.x+RobotC.x;
    ptp.y=scale*trns.y+RobotC.y;
    */
    //H23.3.24
    CvPoint ptp;
    ptp.x=scale*(cos(-dp)*dS.x-sin(-dp)*dS.y)+RobotC.x;
    ptp.y=scale*(sin(-dp)*dS.x+cos(-dp)*dS.y)+RobotC.y;
   //
    int weightcolor=200/wmax*pos_predict[i].weight;
    if(i==max) cvCircle(img,ptp,radius+2,CV_RGB(255,0,0),2,8,0);
    else cvCircle(img,ptp,radius,CV_RGB(55,weightcolor+55,55),1,8,0);
  }
  /*
  //分散楕円を描画 
  CvBox2D box;
  int color;
  box.center.x=robotc.x; box.center.y=robotc.y; 
  box.angle=90-nextpos.theta;
  box.size.width=2*sqrt(varp.pos.x*scale); box.size.height=2*sqrt(varp.pos.y*scale);
  color=128*varp.theta/3;
  cvEllipseBox(img,box,CV_RGB(color,128,0),1,8,0);
  */
#ifdef kuroDISP
  //表示
  cvShowImage("PARTICLE",img);
#endif
}


//////////////////////////////////////////////
//SLAMウィンドウに分散楕円を描画 
///////////////////////////////////////////////
void DrawVarellipsonmap(IplImage* img,STATE robot,STATE offset,STATE variance,double res,int time){
  img->origin=1; 
  CvBox2D box;
  //  double p=CV_PI/180;
  int color;
  box.angle=robot.theta+offset.theta;
//  if(time==0) box.angle=180-offset.theta;
//  else box.angle=180-(robot.theta-90)+offset.theta;
  //robot position
  //  if(variance.pos.x<1e-10) variance.pos.x=1;
  //  if(variance.pos.y<1e-10) variance.pos.y=1;
  double ang=(offset.theta)*D2R;
  box.center.x=(robot.pos.x*cos(ang)-robot.pos.y*sin(ang))/res +offset.pos.x;
  box.center.y=(robot.pos.x*sin(ang)+robot.pos.y*cos(ang))/res +offset.pos.y;
  box.size.width=2*sqrt(variance.pos.x/res);
  box.size.height=2*sqrt(variance.pos.y/res);
  color=128*variance.theta/3;
  cvEllipseBox(img,box,CV_RGB(color,128,0),1,8,0);
}

void DrawVarellipsonmap0(IplImage* img,STATE robot,STATE offset,STATE variance,double res,int time){
  img->origin=1; 
  CvBox2D box;
  //  double p=CV_PI/180;
  int color;
  if(time==0) box.angle=180-offset.theta;
  else box.angle=180-(robot.theta-90)+offset.theta;
  //robot position
  box.center.x=(robot.pos.x*cos((offset.theta-90)*D2R)
		-robot.pos.y*sin((offset.theta-90)*D2R))/res
    +offset.pos.x;
  box.center.y=(robot.pos.x*sin((offset.theta-90)*D2R)
		+robot.pos.y*cos((offset.theta-90)*D2R))/res
    +offset.pos.y;
  box.size.width=2*sqrt(variance.pos.x/res);
  box.size.height=2*sqrt(variance.pos.y/res);
  color=128*variance.theta/3;
  cvEllipseBox(img,box,CV_RGB(color,128,0),1,8,0);
}


//////////////////////////////////////////////////////////
//img1をimg2に合成し，出力をimg2とする関数      H23,05,26
//////////////////////////////////////////////////////////
void ComposingMap(IplImage* img1,IplImage* img2){
  IplImage *grayimg=cvCreateImage(cvGetSize(img1),8,1);
  IplImage *maskimg=cvCreateImage(cvGetSize(img1),8,1);
  //img1画像をグレースケール変換
  cvCvtColor(img1,grayimg,CV_BGR2GRAY);
  //2値化
  cvThreshold(grayimg,maskimg,0,255,CV_THRESH_BINARY_INV);
  IplImage *inverseMaskimg=cvCreateImage(cvGetSize(img1),8,1);
  IplImage *extractedObjectimg=cvCreateImage(cvGetSize(img1),8,3);
  IplImage *extractedBackgroundimg=cvCreateImage(cvGetSize(img1),8,3);
  cvSetZero(extractedBackgroundimg);
  cvCopy(img2,extractedBackgroundimg,maskimg);
  cvNot(maskimg,inverseMaskimg);
  cvSetZero(extractedObjectimg);
  cvCopy(img1,extractedObjectimg,inverseMaskimg);
  cvAdd(extractedBackgroundimg,extractedObjectimg,img2,NULL);
  cvReleaseImage(&grayimg);
  cvReleaseImage(&maskimg);
  cvReleaseImage(&inverseMaskimg);
  cvReleaseImage(&extractedObjectimg);
  cvReleaseImage(&extractedBackgroundimg);
}

//////////////////////////////////////////////
//SLAMウィンドウにパーティクルをプロット（記録用）H23,5,31
//type1:Draw Estimated robot position
//type2:Draw Particle
///////////////////////////////////////////////
void DrawParticlesonMap(IplImage* img,PARTICLE* pos_predict,STATE offset,double res,int time){
  img->origin=1; 
  CvPoint pt1,pt2;
  int j=0,max=0,line=1,radius=(int)(50/res);
  double dir,wmax=0;//,p=CV_PI/180;
  if(time==0) dir=offset.theta;
  else dir=(pos_predict[j].state.theta-90)+offset.theta;
  //particles position
  CvPoint2D64f RRz;
  //最大尤度のパーティクルのインデックスを探索
  for(j=0;j<NofParticles;j++){if(pos_predict[j].weight>wmax){wmax=pos_predict[j].weight; max=j;}}
  //パーティクルの姿勢をロボットマークで定義
  for(j=0;j<NofParticles;j++){
    RRz.x=(pos_predict[j].state.pos.x*cos((offset.theta-90)*D2R)
	   -pos_predict[j].state.pos.y*sin((offset.theta-90)*D2R))/res
      +offset.pos.x;
    RRz.y=(pos_predict[j].state.pos.x*sin((offset.theta-90)*D2R)
	   +pos_predict[j].state.pos.y*cos((offset.theta-90)*D2R))/res
      +offset.pos.y;
    pt1.x=(int)(RRz.x);
    pt1.y=(int)(RRz.y);
    pt2.x=(int)(pt1.x+cos(dir*D2R)*radius);
    pt2.y=(int)(pt1.y+sin(dir*D2R)*radius);
    //図にプロット
    //    int weightcolor=200/wmax*pos_predict[j].weight;
    int weightcolor=255*(pos_predict[j].weight/wmax);
    if(j==max){
      cvCircle(img,pt1,radius,CV_RGB(255,0,0),line,8,0);
      cvLine(img,pt1,pt2,CV_RGB(255,0,0),line,8,0); 
    }
    else{
      cvCircle(img,pt1,radius,CV_RGB(255-weightcolor,255,255-weightcolor),line,8,0);
      cvLine(img,pt1,pt2,CV_RGB(255-weightcolor,255,255-weightcolor),line,8,0); 
    }
  }
}
