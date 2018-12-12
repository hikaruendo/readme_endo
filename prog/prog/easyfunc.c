///////////////////////////////
//2次元ノルムを計算
///////////////////////////////
double Norm2D(double x,double y){
  return sqrt(x*x+y*y);
}

//////////////////////////////////////////////
//平均mean，分散sigの正規分布に従う乱数を生成
//////////////////////////////////////////////
#ifdef ZMTRAND
double randgauss(double mean, double sig, int rngpara)
{
  return(NextNormal()*sig+mean);
}
#else
double randgauss(double mean, double sig, int rngpara){
  double x,y,a,b;
  CvRNG rng = cvRNG(rngpara);
  x = cvRandReal(&rng);
  y = cvRandReal(&rng);
  a = sqrt(-2*log(x))*cos(2*CV_PI*y);
  b = sig*a+mean;
  return b; 
}
#endif

//////////////////////////////////////////////
//区間[0,1]の乱数を生成
//////////////////////////////////////////////
#ifdef ZMTRAND
double rand0to1(int rngpara){return(NextUnif());}
#else
double rand0to1(int rngpara){
  double value;
  CvRNG rng = cvRNG(rngpara);
  value=cvRandReal(&rng);
  return value; 
}
#endif
////////////////////////
//2つの値の最小値を計算
////////////////////////
int min(int a,int b){
  int c;
  if(a<b) c=a;
  else c=b;
  return c;
}

//////////////////////
//ヘルプを表示
//////////////////////
void help(){
  exit(system("cat HELP"));
}
