//////////////////////////
//structure definition
//////////////////////////

//x,y,theta
typedef struct{
  CvPoint2D64f pos; // [mm]   position
  double theta; //[deg]
}STATE;

typedef struct{
  CvPoint3D64f pt; // [mm] 
}SCAN;

//particle
typedef struct{
  STATE  state;     // state vector
  double weight;    // weight
}PARTICLE;

//v,omega
typedef struct{
  double v; // [m/s]
  double omega; //[rad/s]
}INPUT;

typedef struct{
  double v; // [m/s]
  double omega; //[rad/s]
} ACTION;

typedef struct{
  double v; // [m/s]
  double omega; //[rad/s]
} OBSERVATION;
typedef struct{
  double v; // [m/s]
  double omega; //[rad/s]
} PARAM;
