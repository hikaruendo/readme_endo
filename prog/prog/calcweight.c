//////////////////////////////////////////
//尤度計算(単純にANDをとる方法)
//////////////////////////////////////////
double calcWeightonlyAND(IplImage* imgA,IplImage* imgB){
  double result=0;
  IplImage *and_img=cvCloneImage(imgA);
  cvAnd(imgA,imgB,and_img,NULL);
  result=cvCountNonZero(and_img);
  cvReleaseImage(&and_img);
  return result;
}

////////////////////////////////////////////////
//平均値利用の正規化相互相関を用いる尤度計算
////////////////////////////////////////////////
double calcWeightbyMeanNCC(IplImage* imgA, IplImage* imgB){
  double result=0;
  CvMat* mat=cvCreateMat(1,1,CV_32FC1);
  cvMatchTemplate(imgA,imgB,mat,CV_TM_CCOEFF_NORMED);
  result=cvGetReal1D(mat,0);
  result=fabs(result);
  return result;
}

////////////////////////////////////////////////
//相互情報量を用いる尤度計算
////////////////////////////////////////////////
double calcWeightbyMI(IplImage* imgA, IplImage* imgB){
  double histA[256],histB[256],histAB[256][256];
  double result=0;
  int x,y,i,j;
  int xmax,ymax,Gmax=0,Gmin=255;
  CvRect ROI=cvGetImageROI(imgA);
  int K=ROI.height*ROI.width;
  ymax=ROI.height;
  xmax=ROI.width;
  //reset histgram
  for(i=0;i<256;i++){
    histA[i]=0;
    histB[i]=0;
    for(j=0;j<256;j++) histAB[i][j]=0;
  }
  //make histgram A,B,AB
  for(y=0;y<ymax;y++){
    for(x=0;x<xmax;x++){
      CvScalar valueA,valueB;
      valueA=cvGet2D(imgA,y,x);
      valueB=cvGet2D(imgB,y,x);
      if(valueA.val[0]>Gmax) Gmax=valueA.val[0];
      if(valueB.val[0]>Gmax) Gmax=valueB.val[0];
      if(valueA.val[0]<Gmin) Gmin=valueA.val[0];
      if(valueB.val[0]<Gmin) Gmin=valueB.val[0];
      int Aindex=valueA.val[0],Bindex=valueB.val[0];
      histA[Aindex]++;
      histB[Bindex]++;
      histAB[Aindex][Bindex]++;
    }
  }
  //calc I(A,B)
  double pA,pB,pAB;
  for(i=Gmin;i<Gmax+1;i++){
    for(j=Gmin;j<Gmax+1;j++){
      if(histA[i]!=0&&histB[j]!=0&&histAB[i][j]!=0){
	pA=histA[i]/K;
	pB=histB[j]/K;
	pAB=histAB[i][j]/K;
	result=result+(pAB*log2(pAB/(pA*pB)));
      }
    }
  }
  return result;
}
