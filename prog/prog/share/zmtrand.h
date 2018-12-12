//#ifndef __ZMTRAND_H__
//#define __ZMTRAND_H__
#ifndef __zmtrand_h
#define __zmtrand_h

/*
from http://www001.upp.so-net.ne.jp/isaku/rand.html

zmtrand.h ����饤�֥�ꡦ�إå��ե�����
coded by isaku@pb4.so-net.ne.jp

���르�ꥺ���
http://www.math.sci.hiroshima-u.ac.jp/~m-mat/MT/mt.html ��Ȥä�
��������������������������������������������������������������������
�̾�ޥ���   ʣ������     ����
��������������������������������������������������������������������
             state_t        ʣ��������ΰ�����Ѥη�
InitMt()     InitMt_r()     �����μ�Ӥˤ������
InitMtEx()   InitMtEx_r()   Ĺ���̤�����ˤˤ������
NextMt()     NextMt_r()     32�ӥå����ʤ����������
NextUnif()   NextUnif_r()   ���ʾ壱̤�������(��ĤȤ�ޥ���)
NextUnifEx() NextUnifEx_r() ���ʾ壱̤�������(53bit����)
NextInt()    NextInt_r()    ���ʾ��̤�����������(��ĤȤ�ޥ���)
NextIntEx()  NextIntEx_r()  �ݤ���Τʤ����ʾ��̤�����������
��������������������������������������������������������������������

 InitMt()��NextUnif() �� _r ���դ��ʤ��ؿ����̾�λȤ����򤹤�ޥ���
 �Ǥ��롣

 �Ф��� InitMt_r() NextUnif_r() ���ϥޥ������åɤʥץ�����ʣ��
 ����������ɬ�פȤ����絬�ϥ��ߥ�졼������Ѥδؿ��Ǥ��롣 �����
 ���������� state_t �ˤ����ݤ��줿�ΰ���׵᤹�롣 �ۤʤ��ΰ��
 �Ȥ��С��ߤ��θƤӽФ�������Ω������������뤳�Ȥ��Ǥ��롣

 ��InitMt()��InitMtEx() ��ƤӽФ����� NextMt() ����ƤӽФ��Ƥ⡢
   ��ưŪ������ξ��֤��������롣

 ��NextMt_r() ���ǻ��ꤹ���ΰ�򡢥����å��ΰ�䣰�˽��������ʤ�
   malloc() �ΰ�ʤɤˤ���Ȥ��ϡ��ǽ��ɬ�� InitMt_r() ��
   InitMtEx_r() ������Ū�˽�������뤳�ȡ�

����ʬ�ۤ˽������
�֣ø���ˤ��ǿ����르�ꥺ���ŵ��1991 ����ɾ���� �򻲹ͤˤ���
��������������������������������������������������������������������
�̾�ޥ���      ʣ������          ����
��������������������������������������������������������������������
NextChisq()     NextChisq_r()     ��ͳ�٦ͤΥ�������ʬ��       p. 27
NextGamma()     NextGamma_r()     �ѥ�᡼����Υ����ʬ��     p. 31
NextGeometric() NextGeometric_r() ��Ψ�Фδ���ʬ��             p. 34
NextTriangle()  NextTriangle_r()  ����ʬ��                     p. 89
NextExp()       NextExp_r()       ʿ�ѣ��λؿ�ʬ��             p.106
NextNormal()    NextNormal_r()    ɸ������ʬ��(����6.660437��) p.133
NextUnitVect()  NextUnitVect_r()  �μ����Υ�����ñ�̥٥��ȥ� p.185
NextBinomial()  NextBinomial_r()  �ѥ�᡼����,�ФΣ���ʬ��    p.203
NextBinormal()  NextBinormal_r()  ��ط����ҤΣ���������ʬ��   p.211
NextBeta()      NextBeta_r()      �ѥ�᡼����,�¤Υ١���ʬ��  p.257
NextPower()     NextPower_r()     �ѥ�᡼���Τ��߾�ʬ��       p.305
NextLogistic()  NextLogistic_r()  �����ƥ��å�ʬ��           p.313
NextCauchy()    NextCauchy_r()    ��������ʬ��                 p.331
NextFDist()     NextFDist_r()     ��ͳ�٣�,�¤Σ�ʬ��          p.344
NextPoisson()   NextPoisson_r()   ʿ�ѦˤΥݥ�����ʬ��         p.412
NextTDist()     NextTDist_r()     ��ͳ�٣ΤΣ�ʬ��             p.430
NextWeibull()   NextWeibull_r()   �ѥ�᡼�����Υ磻�֥�ʬ��   p.431
��������������������������������������������������������������������
*/

typedef struct {
    unsigned long x[624];      /* ���֥ơ��֥� */
    int           index;       /* ����ǥå��� */
    int           initialized; /* ���������Ƥ���У� */
    long          range;       /* NextIntEx ��������ϰ� */
    unsigned long base;        /* NextIntEx ������δ���� */
    int           shift;       /* NextIntEx ������Υ��եȿ� */
    int           normal_sw;   /* NextNormal �ǻĤ����äƤ��� */
    double        normal_save; /* NextNormal �λĤ���� */
} state_t[1];

extern state_t DefaultMt;

extern unsigned long NextMt_r(state_t);
extern void   InitMt_r    (state_t,unsigned long);
extern void   InitMtEx_r  (state_t,unsigned long*,unsigned);
extern double NextUnif_r  (state_t);
extern double NextUnifEx_r(state_t);
extern long   NextIntEx_r (state_t,long);

extern double NextChisq_r    (state_t,double);
extern double NextGamma_r    (state_t,double);
extern int    NextGeometric_r(state_t,double);
extern double NextTriangle_r (state_t);
extern double NextExp_r      (state_t);
extern double NextNormal_r   (state_t);
extern void   NextUnitVect_r (state_t,double*,int);
extern int    NextBinomial_r (state_t,int,double);
extern void   NextBinormal_r (state_t,double,double*,double*);
extern double NextBeta_r     (state_t,double,double);
extern double NextPower_r    (state_t,double);
extern double NextLogistic_r (state_t);
extern double NextCauchy_r   (state_t);
extern double NextFDist_r    (state_t,double,double);
extern int    NextPoisson_r  (state_t,double);
extern double NextTDist_r    (state_t,double);
extern double NextWeibull_r  (state_t,double);

#define InitMt(S)      InitMt_r    (DefaultMt,S)
#define InitMtEx(K,L)  InitMtEx_r  (DefaultMt,K,L)
#define NextMt()       NextMt_r    (DefaultMt)
#define NextUnif_r(M)  (NextMt_r(M)*(1.0/4294967296.0))
#define NextUnif()     NextUnif_r  (DefaultMt)
#define NextUnifEx()   NextUnifEx_r(DefaultMt)
#define NextInt_r(M,N) ((long)((N)*NextUnif_r(M)))
#define NextInt(N)     NextInt_r   (DefaultMt,N)
#define NextIntEx(N)   NextIntEx_r (DefaultMt,N)

#define NextChisq(N)        NextChisq_r    (DefaultMt,N)
#define NextGamma(A)        NextGamma_r    (DefaultMt,A)
#define NextGeometric(P)    NextGeometric_r(DefaultMt,P)
#define NextTriangle()      NextTriangle_r (DefaultMt)
#define NextExp()           NextExp_r      (DefaultMt)
#define NextNormal()        NextNormal_r   (DefaultMt)
#define NextUnitVect(V,N)   NextUnitVect_r (DefaultMt,V,N)
#define NextBinomial(N,P)   NextBinomial_r (DefaultMt,N,P)
#define NextBinormal(R,X,Y) NextBinormal_r (DefaultMt,R,X,Y)
#define NextBeta(A,B)       NextBeta_r     (DefaultMt,A,B)
#define NextPower(N)        NextPower_r    (DefaultMt,N)
#define NextLogistic()      NextLogistic_r (DefaultMt)
#define NextCauchy()        NextCauchy_r   (DefaultMt)
#define NextFDist(A,B)      NextFDist_r    (DefaultMt,A,B)
#define NextPoisson(L)      NextPoisson_r  (DefaultMt,L)
#define NextTDist(N)        NextTDist_r    (DefaultMt,N)
#define NextWeibull(A)      NextWeibull_r  (DefaultMt,A)

#endif /* __zmtrand_h*/

//#endif /* __ZMTRAND_H__ */
