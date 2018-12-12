#ifndef __zmtrand_c
#define __zmtrand_c
/*http://www001.upp.so-net.ne.jp/isaku/rand.html
zmtrand.c 乱数ライブラリ本体
coded by isaku@pb4.so-net.ne.jp
*/

#include <math.h>
#include "zmtrand.h"

/*┌────────────────┐
　│マクロで渡されるデフォルトの領域│
　└────────────────┘*/
state_t DefaultMt;

/*┌──────┐
　│領域を初期化│
　└──────┘*/
void InitMt_r(state_t mt,unsigned long s)
{
    unsigned p;

    for (mt->x[0]=s&=0xFFFFFFFFUL,p=1;p<624;p++)
        mt->x[p]=s=(1812433253UL*(s^(s>>30))+p)&0xFFFFFFFFUL;
    mt->index=0; mt->initialized=1; mt->range=0; mt->normal_sw=0;
}

/*┌─────────┐
　│配列を使って初期化│
　└─────────┘*/
void InitMtEx_r(state_t mt,unsigned long*key,unsigned len)
{
    unsigned i=1,j=0,k; unsigned long*x=mt->x;
    InitMt_r(mt,19650218UL);
    for (k=624>len?624:len;k;k--) {
        x[i]=(x[i]^((x[i-1]^(x[i-1]>>30))*1664525UL))+key[j]+j;
        x[i]&=0xFFFFFFFFUL;
        if (++i>=624) { x[0]=x[623]; i=1; }
        if (++j>=len) j=0;
    }
    for (k=623;k;k--) {
        x[i]=(x[i]^((x[i-1]^(x[i-1]>>30))*1566083941UL))-i;
        x[i]&=0xFFFFFFFFUL;
        if (++i>=624) { x[0]=x[623]; i=1; }
    }
    x[0]=0x80000000UL;
}

/*┌───────────────────────┐
　│32ビット符号なし長整数乱数、他の乱数の基本関数│
　└───────────────────────┘*/
unsigned long NextMt_r(state_t mt)
{
    static unsigned long seed=5489,mag01[2]={0,0x9908B0DFUL};
    unsigned long y; int k=mt->index;
    if (k==0) {
        if (mt->initialized!=1) {
            if (mt==DefaultMt) InitMt_r(mt,5489);
            else               InitMt_r(mt,++seed);
        }
        for (;k<227;k++) {
            y=(mt->x[k]&(1UL<<31))|(mt->x[k+1]&~(1UL<<31));
            mt->x[k]=mt->x[k+397]^(y>>1)^mag01[(int)y&1];
        }
        for (;k<623;k++) {
            y=(mt->x[k]&(1UL<<31))|(mt->x[k+1]&~(1UL<<31));
            mt->x[k]=mt->x[k-227]^(y>>1)^mag01[(int)y&1];
        }
        y=(mt->x[623]&(1UL<<31))|(mt->x[0]&~(1UL<<31));
        mt->x[623]=mt->x[396]^(y>>1)^mag01[(int)y&1]; k=0;
    }
    y=mt->x[k]; mt->index=k==623?0:k+1; y^=y>>11;
    y^=(y<<7)&0x9D2C5680UL; y^=(y<<15)&0xEFC60000UL;
    y^=y>>18; return y;
}

/*┌──────────────────────┐
　│０以上１未満の浮動小数点一様乱数(53bit精度) │
　└──────────────────────┘*/
double NextUnifEx_r(state_t mt)
{
    unsigned long x=NextMt_r(mt)>>5,y=NextMt_r(mt)>>6;
    return(x*67108864.0+y)*(1.0/9007199254740992.0);
}

/*┌───────────────────────────┐
　│丸め誤差のない０以上range未満の整数乱数　　　　　　　 │
　│　　　　　　　　　　　　　　　　　　　　　　　　　　　│
　│rangeが一定ならNextInt_r()よりも３倍高速　　　　　　　│
　│for (i=0;i<10000;i++)　　　　　　　　　　 　　　　　　│
　│{ x[i]=NextIntEx(100); y[i]=NextIntEx(200); } 　　　　│
　│とするよりも　　　　　　　　　　　　　　　　　　　　　│
　│for (i=0;i<10000;i++) x[i]=NextIntEx(100);　　　　　　│
　│for (i=0;i<10000;i++) y[i]=NextIntEx(200);　　　　　　│
　│としたほうが５倍高速になる。また、複数系列を使って　　│
　│for (i=0;i<10000;i++)　　　　　　　　　　 　　　　　　│
　│{ x[i]=NextIntEx_r(a,100); y[i]=NextIntEx_r(b,200); } │
　│としても５倍高速になる　　　　　　　　　　　　　　　　│
　└───────────────────────────┘*/
long NextIntEx_r(state_t mt,long range)
{
    unsigned long y,base; long remain; int shift;

    if (range<=0) return 0;
    if (range!=mt->range) {
        mt->base=mt->range=range;
        for (mt->shift=0;mt->base<=(1UL<<30);mt->shift++)
            mt->base<<=1;
    }
    for (;;) {
        y=NextMt_r(mt)>>1;
        if (y<mt->base) return y>>mt->shift;
        base=mt->base; shift=mt->shift; y-=base;
        for (remain=(1L<<31)-base;remain>=range;remain-=base) {
            for (;(long)base>remain;base>>=1) shift--;
            if (y<base) return y>>shift; else y-=base;
        }
    }
}

/*┌──────────────┐
　│自由度νのカイ２乗分布 p.27 │
　└──────────────┘*/
double NextChisq_r(state_t mt,double n)
{
    return 2*NextGamma_r(mt,0.5*n);
}

/*┌───────────────┐
　│パラメータａのガンマ分布 p.31 │
　└───────────────┘*/
double NextGamma_r(state_t mt,double a)
{
    double t,u,x,y;
    if (a>1) {
        t=sqrt(2*a-1);
        do {
            do {
                do {
                    x=1-NextUnif_r(mt);
                    y=2*NextUnif_r(mt)-1;
                } while (x*x+y*y>1);
                y/=x; x=t*y+a-1;
            } while (x<=0);
            u=(a-1)*log(x/(a-1))-t*y;
        } while (u<-50||NextUnif_r(mt)>(1+y*y)*exp(u));
    } else {
        t=2.718281828459045235/(a+2.718281828459045235);
        do {
            if (NextUnif_r(mt)<t) {
                x=pow(NextUnif_r(mt),1/a); y=exp(-x);
            } else {
                x=1-log(1-NextUnif_r(mt)); y=pow(x,a-1);
            }
        } while (NextUnif_r(mt)>=y);
    }
    return x;
}

/*┌───────────┐
　│確率Ｐの幾何分布 p.34 │
　└───────────┘*/
int NextGeometric_r(state_t mt,double p)
{ return(int)ceil(log(1.0-NextUnif_r(mt))/log(1-p)); }

/*┌───────┐
　│三角分布 p.89 │
　└───────┘*/
double NextTriangle_r(state_t mt)
{ double a=NextUnif_r(mt),b=NextUnif_r(mt); return a-b; }

/*┌───────────┐
　│平均１の指数分布 p.106│
　└───────────┘*/
double NextExp_r(state_t mt)
{ return-log(1-NextUnif_r(mt)); }

/*┌─────────────────┐
　│標準正規分布(最大6.660437σ) p.133│
　└─────────────────┘*/
double NextNormal_r(state_t mt)
{
    if (mt->normal_sw==0) {
        double t=sqrt(-2*log(1.0-NextUnif_r(mt)));
        double u=3.141592653589793*2*NextUnif_r(mt);
        mt->normal_save=t*sin(u); mt->normal_sw=1; return t*cos(u);
    }else{ mt->normal_sw=0; return mt->normal_save; }
}

/*┌─────────────────┐
　│Ｎ次元のランダム単位ベクトル p.185│
　└─────────────────┘*/
void NextUnitVect_r(state_t mt,double*v,int n)
{
    int i; double r=0;
    for (i=0;i<n;i++) { v[i]=NextNormal_r(mt); r+=v[i]*v[i]; }
    r=sqrt(r);
    for (i=0;i<n;i++) v[i]/=r;
}

/*┌────────────────┐
　│パラメータＮ,Ｐの２項分布 p.203 │
　└────────────────┘*/
int NextBinomial_r(state_t mt,int n,double p)
{
    int i,r=0;
    for (i=0;i<n;i++) if (NextUnif_r(mt)<p) r++;
    return r;
}

/*┌────────────────┐
　│相関係数Ｒの２変量正規分布 p.211│
　└────────────────┘*/
void NextBinormal_r(state_t mt,double r,double*x,double*y)
{
    double r1,r2,s;
    do {
        r1=2*NextUnif_r(mt)-1;
        r2=2*NextUnif_r(mt)-1;
        s=r1*r1+r2*r2;
    } while (s>1||s==0);
    s= -log(s)/s; r1=sqrt((1+r)*s)*r1;
    r2=sqrt((1-r)*s)*r2; *x=r1+r2; *y=r1-r2;
}

/*┌─────────────────┐
　│パラメータＡ,Ｂのベータ分布 p.257 │
　└─────────────────┘*/
double NextBeta_r(state_t mt,double a,double b)
{
    double temp=NextGamma_r(mt,a);
    return temp/(temp+NextGamma_r(mt,b));
}

/*┌──────────────┐
　│パラメータＮの累乗分布 p.305│
　└──────────────┘*/
double NextPower_r(state_t mt,double n)
{ return pow(NextUnif_r(mt),1.0/(n+1)); }

/*┌────────────┐
　│ロジスティック分布 p.313│
　└────────────┘*/
double NextLogistic_r(state_t mt)
{
    double r;
    do r=NextUnif_r(mt); while (r==0);
    return log(r/(1-r));
}

/*┌─────────┐
　│コーシー分布 p.331│
　└─────────┘*/
double NextCauchy_r(state_t mt)
{
    double x,y;
    do { x=1-NextUnif_r(mt); y=2*NextUnif_r(mt)-1; }
    while (x*x+y*y>1);
    return y/x;
}

/*┌─────────────┐
　│自由度Ａ,ＢのＦ分布 p.344 │
　└─────────────┘*/
double NextFDist_r(state_t mt,double n1,double n2)
{
    double nc1=NextChisq_r(mt,n1),nc2=NextChisq_r(mt,n2);
    return (nc1*n2)/(nc2*n1);
}

/*┌─────────────┐
　│平均λのポアソン分布 p.412│
　└─────────────┘*/
int NextPoisson_r(state_t mt,double lambda)
{
    int k; lambda=exp(lambda)*NextUnif_r(mt);
    for (k=0;lambda>1;k++) lambda*=NextUnif_r(mt);
    return k;
}

/*┌───────────┐
　│自由度Ｎのｔ分布 p.428│
　└───────────┘*/
double NextTDist_r(state_t mt,double n)
{
    double a,b,c;
    if (n<=2) {
        do a=NextChisq_r(mt,n); while (a==0);
        return NextNormal_r(mt)/sqrt(a/n);
    }
    do {
        a=NextNormal_r(mt); b=a*a/(n-2);
        c=log(1-NextUnif_r(mt))/(1-0.5*n);
    } while (exp(-b-c)>1-b);
    return a/sqrt((1-2.0/n)*(1-b));
}

/*┌────────────────┐
　│パラメータαのワイブル分布 p.431│
　└────────────────┘*/
double NextWeibull_r(state_t mt,double alpha)
{ return pow(-log(1-NextUnif_r(mt)),1/alpha); }
//extern state_t DefaultMt;

#endif /* __zmtrand_c*/
