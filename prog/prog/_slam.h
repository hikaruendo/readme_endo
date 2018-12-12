//////////////////////////////////////////////////////////////
//slamX.cに必要なサブルーチン類の定義
//
//なるべく相互に関数を使わないように書いているが
//書かれている順番を変えると動かなくなることもあるので注意
//////////////////////////////////////////////////////////////

//基礎的な構造体など
//#include "structure.h"
//////////////////////////
//structure definition
//////////////////////////

//x,y,theta
typedef struct{
  //  CvPoint2D64f pos; // [mm]   position
  CvPoint3D64f pos;
  //  CvPoint3D64f pos3;
  CvPoint3D64f ang;
  double theta; //[deg]
} STATE;

typedef struct{
  CvPoint3D64f pt; // [mm] 
}SCAN;

//particle
typedef struct{
  STATE  state;     // state vector
  double weight;    // weight
} PARTICLE;

//v,omega
typedef struct{
  int type;
  double v; // [m/s]
  double omega; //[rad/s]
  double theta;
  double V[3];//Velocity for 3D
  double O[3];//Orientation 3D
  double DELTA;
  //input_case 2
  int nv;//2(x,y) for type=1, 3 for type2
  int nw;//1(a) for type1, 3 for type2
  double t00;//initial time
  //  double t0;//prefious time
} INPUT;

typedef struct{
  double x; //
  double y; //
  double theta; 
  int    nS;//number of Saterite
  double d;
} GPSDATA;


typedef struct{
  double x; //
  double y; //
  double z; //
  double w;//quaternion?
  double ax;
  double ay;
  double az;
  double d; // 
} KINECTDATA;

typedef struct{
  double v; // [m/s]
  double omega; //[rad/s]
  double V[3];
  double W[3];
} ACTION;

typedef struct{
  double v; // [m/s]
  double omega; //[rad/s]
  double V[3];
  double W[3];
} OBSERVATION;

typedef struct{
  int class;
  int type;
  int ndata;
  char fn[126];
  FILE *fp;
  //  double sf; //sampling frequency
  double T;//sampling perod
  double x0,y0,a0;//for U
  double X0[3],A0[3];//for U of type=2 for 3D-kinect input
  double k0;//for K
  int nS0;//for x0,y0,nS0 for GPS
  //  GPSDATA gps0;
  int l;//current line number 
  double lD;
  int timeA;
  int nv;
  int nw;
  double t00;
} FILEDATA;
typedef struct{
  FILEDATA U[1];//action (or input)
  FILEDATA Z[3];//observation
  FILEDATA T[1];
  FILEDATA F[1];//true
  FILEDATA M[1];//map_matrix
  FILEDATA K[1];//kinect
  int nZ;
  char *dir;//argv[1]//in _slam.c

  int MATCHres;//MATCHING: RESOLUTION[mm] argv[5]
  int MAPres;//mapping resoluution[mm] argv[6] 
  STATE offset;
  double robotang0,kinectang0;//initial robot & kinect orientation
  double DELTA;
  double gpsA;
  double orsA;
  double kinectA;
  double gps_theta;
  int ft;
} PARAM;
typedef struct{
  int N;
  double sigma_v;
  double sigma_w;
  double sigma_a;
  double sigma_V[3];
  double sigma_W[3];
  double sigma_A[3];
} PF;


//簡単な数式を用いる関数など
#include "easyfunc.c"
//#include "easyfunc.h"

//各種初期化をする関数など
#include "init.c"

//PFの処理など
#include "pfprocess.c"

//ウィンドウなどへの描画関数など
#include "draw.c"

//ファイル入出力など
#include "fileRorW5.c"

//重みの計算など
#include "calcweight.c"

//LRFのデータ処理など
#include "LRFfunc.c"

//理想軌道からの距離
#include "calcalfa.c"

//GPSとパーティクルの平均との距離
#include "distance.c"
