////////////////////////////////////////////////
//オフラインファイルからLRFデータをとってくる
////////////////////////////////////////////////
void getLRFdatafromfile(char FILEPASS[], SCAN LRFdata[], int t){
  FILE *lrf_file;
  char buff[256];
  int i;

  sprintf(buff,"%s/LRF/LRF_%d.dat",FILEPASS,t);
  if((lrf_file=fopen(buff,"r"))==NULL){
    printf("CAN'T OPEN OFFLINE LRF DATA(%s)\n",buff);
    exit(1);
  }
  int nxy=0;
  for(i=0;i<lrf_data_num;i++){
    double Lx,Ly,Lr;
    nxy=fscanf(lrf_file,"%lf %lf\n",&Lx,&Ly);
    Lr=Norm2D(Lx,Ly);
    //半径500[mm]以内と4000[mm]以上はカット(0,0として記録)
    if((Lr<500)||(Lr>4000)||nxy<2){Lx=Ly=0;}
    //    if((Lr<500)||Lr>4000){Lx=Ly=0;}
    LRFdata[i].pt.x=Lx;
    LRFdata[i].pt.y=Ly;
  }
  //  if(nxy<lrf_data_num*2) fprintf(stderr,"#lack of data(%d,%d)\n",nxy,lrf_data_num*2);
  fclose(lrf_file);
}

///////////////////////////////
//LRFのデータをk+1->kにコピー
///////////////////////////////
void copyLRF(SCAN lrf_k[], SCAN lrf_kplus1[]){
  int i;
  for(i=0;i<lrf_data_num;i++){
    lrf_k[i].pt.x=lrf_kplus1[i].pt.x;
    lrf_k[i].pt.y=lrf_kplus1[i].pt.y;
    lrf_k[i].pt.z=lrf_kplus1[i].pt.z;
  }
}
