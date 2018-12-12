//#ifndef __ZMTRAND_H__
//#define __ZMTRAND_H__
#ifndef __zmtrand_h
#define __zmtrand_h

/*
from http://www001.upp.so-net.ne.jp/isaku/rand.html

zmtrand.h 乱数ライブラリ・ヘッダファイル
coded by isaku@pb4.so-net.ne.jp

アルゴリズムは
http://www.math.sci.hiroshima-u.ac.jp/~m-mat/MT/mt.html を使った
──────────────────────────────────
通常マクロ   複数系列     説明
──────────────────────────────────
             state_t        複数系列の領域宣言用の型
InitMt()     InitMt_r()     整数の種Ｓによる初期化
InitMtEx()   InitMtEx_r()   長さＬの配列Ｋによる初期化
NextMt()     NextMt_r()     32ビット符号なし整数の乱数
NextUnif()   NextUnif_r()   ０以上１未満の乱数(二つともマクロ)
NextUnifEx() NextUnifEx_r() ０以上１未満の乱数(53bit精度)
NextInt()    NextInt_r()    ０以上Ｎ未満の整数乱数(二つともマクロ)
NextIntEx()  NextIntEx_r()  丸め誤差のない０以上Ｎ未満の整数乱数
──────────────────────────────────

 InitMt()、NextUnif() 等 _r の付かない関数は通常の使い方をするマクロ
 である。

 対して InitMt_r() NextUnif_r() 等はマルチスレッドなプログラムや複数
 系列の乱数を必要とする大規模シミュレーション用の関数である。 これら
 は第一引数で state_t により確保された領域を要求する。 異なる領域を
 使えば、互いの呼び出しから独立した系列を得ることかできる。

 ※InitMt()、InitMtEx() を呼び出さずに NextMt() 等を呼び出しても、
   自動的に乱数の状態を初期化する。

 ※NextMt_r() 等で指定する領域を、スタック領域や０に初期化されない
   malloc() 領域などにするときは、最初に必ず InitMt_r() か
   InitMtEx_r() で明示的に初期化すること。

●各分布に従う乱数
「Ｃ言語による最新アルゴリズム事典」1991 技術評論社 を参考にした
──────────────────────────────────
通常マクロ      複数系列          説明
──────────────────────────────────
NextChisq()     NextChisq_r()     自由度νのカイ２乗分布       p. 27
NextGamma()     NextGamma_r()     パラメータａのガンマ分布     p. 31
NextGeometric() NextGeometric_r() 確率Ｐの幾何分布             p. 34
NextTriangle()  NextTriangle_r()  三角分布                     p. 89
NextExp()       NextExp_r()       平均１の指数分布             p.106
NextNormal()    NextNormal_r()    標準正規分布(最大6.660437σ) p.133
NextUnitVect()  NextUnitVect_r()  Ｎ次元のランダム単位ベクトル p.185
NextBinomial()  NextBinomial_r()  パラメータＮ,Ｐの２項分布    p.203
NextBinormal()  NextBinormal_r()  相関係数Ｒの２変量正規分布   p.211
NextBeta()      NextBeta_r()      パラメータＡ,Ｂのベータ分布  p.257
NextPower()     NextPower_r()     パラメータＮの累乗分布       p.305
NextLogistic()  NextLogistic_r()  ロジスティック分布           p.313
NextCauchy()    NextCauchy_r()    コーシー分布                 p.331
NextFDist()     NextFDist_r()     自由度Ａ,ＢのＦ分布          p.344
NextPoisson()   NextPoisson_r()   平均λのポアソン分布         p.412
NextTDist()     NextTDist_r()     自由度Ｎのｔ分布             p.430
NextWeibull()   NextWeibull_r()   パラメータαのワイブル分布   p.431
──────────────────────────────────
*/

typedef struct {
    unsigned long x[624];      /* 状態テーブル */
    int           index;       /* インデックス */
    int           initialized; /* 初期化されていれば１ */
    long          range;       /* NextIntEx で前回の範囲 */
    unsigned long base;        /* NextIntEx で前回の基準値 */
    int           shift;       /* NextIntEx で前回のシフト数 */
    int           normal_sw;   /* NextNormal で残りを持っている */
    double        normal_save; /* NextNormal の残りの値 */
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
