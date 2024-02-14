/* This file was automatically generated by CasADi 3.6.3.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) cassie_left_foot_back_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

static const casadi_int casadi_s0[20] = {16, 1, 0, 16, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
static const casadi_int casadi_s1[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s2[67] = {3, 16, 0, 3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2};

/* cassie_left_foot_back:(i0[16],i1[16])->(o0[3],o1[3x16],o2[3]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a7, a8, a9;
  a0=-4.9000000000000002e-02;
  a1=8.9999999999999997e-02;
  a2=-4.4408920985006262e-16;
  a3=arg[0]? arg[0][0] : 0;
  a4=cos(a3);
  a5=(a2*a4);
  a6=(a1*a5);
  a0=(a0+a6);
  a6=1.2000000000000000e-01;
  a7=arg[0]? arg[0][1] : 0;
  a8=cos(a7);
  a9=(a2*a8);
  a10=(a5*a9);
  a3=sin(a3);
  a11=(a2*a3);
  a12=sin(a7);
  a13=(a11*a12);
  a13=(a8-a13);
  a10=(a10+a13);
  a13=4.8966386501092529e-12;
  a14=arg[0]? arg[0][2] : 0;
  a15=cos(a14);
  a16=(a13*a15);
  a17=sin(a14);
  a16=(a16+a17);
  a18=(a10*a16);
  a19=(a2-a5);
  a20=4.8965831389580217e-12;
  a21=(a20*a17);
  a21=(a21-a15);
  a22=(a19*a21);
  a23=(a2*a12);
  a5=(a5*a23);
  a11=(a11*a8);
  a11=(a11+a12);
  a5=(a5+a11);
  a11=5.5511151231257827e-17;
  a24=(a11*a15);
  a25=(a13*a17);
  a24=(a24+a25);
  a25=(a5*a24);
  a22=(a22-a25);
  a18=(a18+a22);
  a22=(a6*a18);
  a25=4.4999999999999997e-03;
  a26=(a20*a10);
  a26=(a26+a5);
  a27=(a25*a26);
  a22=(a22+a27);
  a22=(a0+a22);
  a27=6.0679999999999998e-02;
  a28=arg[0]? arg[0][3] : 0;
  a29=cos(a28);
  a30=(a18*a29);
  a31=(a13*a17);
  a31=(a15-a31);
  a10=(a10*a31);
  a32=(a20*a15);
  a32=(a17+a32);
  a33=(a19*a32);
  a15=(a13*a15);
  a17=(a11*a17);
  a15=(a15-a17);
  a5=(a5*a15);
  a33=(a33-a5);
  a10=(a10+a33);
  a33=sin(a28);
  a5=(a10*a33);
  a30=(a30+a5);
  a5=(a27*a30);
  a17=4.7410000000000001e-02;
  a10=(a10*a29);
  a18=(a18*a33);
  a10=(a10-a18);
  a18=(a17*a10);
  a5=(a5+a18);
  a5=(a22+a5);
  a18=4.3475999999999998e-01;
  a34=arg[0]? arg[0][4] : 0;
  a35=cos(a34);
  a36=(a30*a35);
  a37=sin(a34);
  a38=(a10*a37);
  a36=(a36+a38);
  a38=(a18*a36);
  a39=2.0000000000000000e-02;
  a10=(a10*a35);
  a30=(a30*a37);
  a10=(a10-a30);
  a30=(a39*a10);
  a38=(a38+a30);
  a38=(a5+a38);
  a30=4.0799999999999997e-01;
  a40=arg[0]? arg[0][5] : 0;
  a41=cos(a40);
  a42=(a36*a41);
  a43=sin(a40);
  a44=(a10*a43);
  a42=(a42+a44);
  a44=(a30*a42);
  a45=-4.0000000000000001e-02;
  a10=(a10*a41);
  a36=(a36*a43);
  a10=(a10-a36);
  a36=(a45*a10);
  a44=(a44+a36);
  a44=(a38+a44);
  a36=-8.9999999999999997e-02;
  a46=arg[0]? arg[0][7] : 0;
  a47=cos(a46);
  a48=(a42*a47);
  a49=sin(a46);
  a50=(a10*a49);
  a48=(a48+a50);
  a50=(a36*a48);
  a50=(a44+a50);
  if (res[0]!=0) res[0][0]=a50;
  a51=1.3500000000000001e-01;
  a52=(a1*a3);
  a51=(a51+a52);
  a52=(a3*a9);
  a53=(a4*a12);
  a52=(a52+a53);
  a53=(a52*a16);
  a54=(a4*a8);
  a55=(a3*a23);
  a54=(a54-a55);
  a55=(a54*a24);
  a56=(a3*a21);
  a55=(a55-a56);
  a53=(a53+a55);
  a55=(a6*a53);
  a56=(a20*a52);
  a56=(a56-a54);
  a57=(a25*a56);
  a55=(a55+a57);
  a55=(a51+a55);
  a57=(a53*a29);
  a52=(a52*a31);
  a54=(a54*a15);
  a58=(a3*a32);
  a54=(a54-a58);
  a52=(a52+a54);
  a54=(a52*a33);
  a57=(a57+a54);
  a54=(a27*a57);
  a52=(a52*a29);
  a53=(a53*a33);
  a52=(a52-a53);
  a53=(a17*a52);
  a54=(a54+a53);
  a54=(a55+a54);
  a53=(a57*a35);
  a58=(a52*a37);
  a53=(a53+a58);
  a58=(a18*a53);
  a52=(a52*a35);
  a57=(a57*a37);
  a52=(a52-a57);
  a57=(a39*a52);
  a58=(a58+a57);
  a58=(a54+a58);
  a57=(a53*a41);
  a59=(a52*a43);
  a57=(a57+a59);
  a59=(a30*a57);
  a52=(a52*a41);
  a53=(a53*a43);
  a52=(a52-a53);
  a53=(a45*a52);
  a59=(a59+a53);
  a59=(a58+a59);
  a53=(a57*a47);
  a60=(a52*a49);
  a53=(a53+a60);
  a60=(a36*a53);
  a60=(a59+a60);
  if (res[0]!=0) res[0][1]=a60;
  a61=(a3*a12);
  a62=(a2*a8);
  a61=(a61+a62);
  a9=(a4*a9);
  a61=(a61-a9);
  a16=(a61*a16);
  a23=(a4*a23);
  a8=(a3*a8);
  a12=(a2*a12);
  a8=(a8-a12);
  a23=(a23+a8);
  a24=(a23*a24);
  a8=1.9721522630525295e-31;
  a8=(a4+a8);
  a21=(a8*a21);
  a24=(a24+a21);
  a16=(a16+a24);
  a24=(a6*a16);
  a21=(a20*a61);
  a21=(a21-a23);
  a12=(a25*a21);
  a24=(a24+a12);
  a4=(a1*a4);
  a24=(a24-a4);
  a12=(a16*a29);
  a61=(a61*a31);
  a23=(a23*a15);
  a32=(a8*a32);
  a23=(a23+a32);
  a61=(a61+a23);
  a23=(a61*a33);
  a12=(a12+a23);
  a23=(a27*a12);
  a61=(a61*a29);
  a16=(a16*a33);
  a61=(a61-a16);
  a16=(a17*a61);
  a23=(a23+a16);
  a23=(a24+a23);
  a16=(a12*a35);
  a33=(a61*a37);
  a16=(a16+a33);
  a33=(a18*a16);
  a61=(a61*a35);
  a12=(a12*a37);
  a61=(a61-a12);
  a12=(a39*a61);
  a33=(a33+a12);
  a33=(a23+a33);
  a12=(a16*a41);
  a37=(a61*a43);
  a12=(a12+a37);
  a37=(a30*a12);
  a61=(a61*a41);
  a16=(a16*a43);
  a61=(a61-a16);
  a16=(a45*a61);
  a37=(a37+a16);
  a37=(a33+a37);
  a16=(a12*a47);
  a43=(a61*a49);
  a16=(a16+a43);
  a43=(a36*a16);
  a43=(a37+a43);
  if (res[0]!=0) res[0][2]=a43;
  a41=-5.9952043329758457e-17;
  a35=(a2*a60);
  a41=(a41-a35);
  if (res[1]!=0) res[1][0]=a41;
  a41=-2.1760371282653069e-17;
  a35=(a2*a50);
  a35=(a43-a35);
  a41=(a41-a35);
  if (res[1]!=0) res[1][1]=a41;
  a41=-1.3500000000000001e-01;
  a41=(a41+a60);
  if (res[1]!=0) res[1][2]=a41;
  a41=(a51*a8);
  a35=(a4*a3);
  a41=(a41-a35);
  a35=(a60*a8);
  a29=(a43*a3);
  a35=(a35+a29);
  a41=(a41-a35);
  if (res[1]!=0) res[1][3]=a41;
  a41=(a4*a19);
  a35=(a0*a8);
  a41=(a41+a35);
  a35=(a43*a19);
  a8=(a50*a8);
  a35=(a35-a8);
  a41=(a41+a35);
  a41=(-a41);
  if (res[1]!=0) res[1][4]=a41;
  a41=(a50*a3);
  a35=(a60*a19);
  a41=(a41+a35);
  a3=(a0*a3);
  a19=(a51*a19);
  a3=(a3+a19);
  a41=(a41-a3);
  if (res[1]!=0) res[1][5]=a41;
  a41=(a51*a21);
  a3=(a4*a56);
  a41=(a41+a3);
  a3=(a60*a21);
  a19=(a43*a56);
  a3=(a3-a19);
  a41=(a41-a3);
  if (res[1]!=0) res[1][6]=a41;
  a4=(a4*a26);
  a41=(a0*a21);
  a4=(a4+a41);
  a41=(a43*a26);
  a3=(a50*a21);
  a41=(a41-a3);
  a4=(a4+a41);
  a4=(-a4);
  if (res[1]!=0) res[1][7]=a4;
  a0=(a0*a56);
  a51=(a51*a26);
  a0=(a0-a51);
  a51=(a50*a56);
  a4=(a60*a26);
  a51=(a51-a4);
  a0=(a0-a51);
  if (res[1]!=0) res[1][8]=a0;
  a0=(a55*a21);
  a51=(a24*a56);
  a0=(a0-a51);
  a51=(a60*a21);
  a4=(a43*a56);
  a51=(a51-a4);
  a0=(a0-a51);
  if (res[1]!=0) res[1][9]=a0;
  a24=(a24*a26);
  a0=(a22*a21);
  a24=(a24-a0);
  a0=(a43*a26);
  a51=(a50*a21);
  a0=(a0-a51);
  a24=(a24-a0);
  if (res[1]!=0) res[1][10]=a24;
  a22=(a22*a56);
  a55=(a55*a26);
  a22=(a22-a55);
  a55=(a50*a56);
  a24=(a60*a26);
  a55=(a55-a24);
  a22=(a22-a55);
  if (res[1]!=0) res[1][11]=a22;
  a22=(a54*a21);
  a55=(a23*a56);
  a22=(a22-a55);
  a55=(a60*a21);
  a24=(a43*a56);
  a55=(a55-a24);
  a22=(a22-a55);
  if (res[1]!=0) res[1][12]=a22;
  a23=(a23*a26);
  a22=(a5*a21);
  a23=(a23-a22);
  a22=(a43*a26);
  a55=(a50*a21);
  a22=(a22-a55);
  a23=(a23-a22);
  if (res[1]!=0) res[1][13]=a23;
  a5=(a5*a56);
  a54=(a54*a26);
  a5=(a5-a54);
  a54=(a50*a56);
  a23=(a60*a26);
  a54=(a54-a23);
  a5=(a5-a54);
  if (res[1]!=0) res[1][14]=a5;
  a5=(a58*a21);
  a54=(a33*a56);
  a5=(a5-a54);
  a54=(a60*a21);
  a23=(a43*a56);
  a54=(a54-a23);
  a5=(a5-a54);
  if (res[1]!=0) res[1][15]=a5;
  a33=(a33*a26);
  a5=(a38*a21);
  a33=(a33-a5);
  a5=(a43*a26);
  a54=(a50*a21);
  a5=(a5-a54);
  a33=(a33-a5);
  if (res[1]!=0) res[1][16]=a33;
  a38=(a38*a56);
  a58=(a58*a26);
  a38=(a38-a58);
  a58=(a50*a56);
  a33=(a60*a26);
  a58=(a58-a33);
  a38=(a38-a58);
  if (res[1]!=0) res[1][17]=a38;
  a38=0.;
  if (res[1]!=0) res[1][18]=a38;
  if (res[1]!=0) res[1][19]=a38;
  if (res[1]!=0) res[1][20]=a38;
  a58=(a59*a21);
  a33=(a37*a56);
  a58=(a58-a33);
  a33=(a60*a21);
  a5=(a43*a56);
  a33=(a33-a5);
  a58=(a58-a33);
  if (res[1]!=0) res[1][21]=a58;
  a37=(a37*a26);
  a58=(a44*a21);
  a37=(a37-a58);
  a43=(a43*a26);
  a58=(a50*a21);
  a43=(a43-a58);
  a37=(a37-a43);
  if (res[1]!=0) res[1][22]=a37;
  a44=(a44*a56);
  a59=(a59*a26);
  a44=(a44-a59);
  a50=(a50*a56);
  a60=(a60*a26);
  a50=(a50-a60);
  a44=(a44-a50);
  if (res[1]!=0) res[1][23]=a44;
  if (res[1]!=0) res[1][24]=a38;
  if (res[1]!=0) res[1][25]=a38;
  if (res[1]!=0) res[1][26]=a38;
  if (res[1]!=0) res[1][27]=a38;
  if (res[1]!=0) res[1][28]=a38;
  if (res[1]!=0) res[1][29]=a38;
  if (res[1]!=0) res[1][30]=a38;
  if (res[1]!=0) res[1][31]=a38;
  if (res[1]!=0) res[1][32]=a38;
  if (res[1]!=0) res[1][33]=a38;
  if (res[1]!=0) res[1][34]=a38;
  if (res[1]!=0) res[1][35]=a38;
  if (res[1]!=0) res[1][36]=a38;
  if (res[1]!=0) res[1][37]=a38;
  if (res[1]!=0) res[1][38]=a38;
  if (res[1]!=0) res[1][39]=a38;
  if (res[1]!=0) res[1][40]=a38;
  if (res[1]!=0) res[1][41]=a38;
  if (res[1]!=0) res[1][42]=a38;
  if (res[1]!=0) res[1][43]=a38;
  if (res[1]!=0) res[1][44]=a38;
  if (res[1]!=0) res[1][45]=a38;
  if (res[1]!=0) res[1][46]=a38;
  if (res[1]!=0) res[1][47]=a38;
  a38=arg[1]? arg[1][7] : 0;
  a44=cos(a46);
  a50=cos(a40);
  a60=cos(a34);
  a59=cos(a28);
  a37=cos(a14);
  a14=sin(a14);
  a43=(a13*a14);
  a43=(a37-a43);
  a58=sin(a7);
  a33=arg[1]? arg[1][0] : 0;
  a1=(a1*a33);
  a5=(a58*a1);
  a54=(a43*a5);
  a23=(a13*a37);
  a22=(a11*a14);
  a23=(a23-a22);
  a7=cos(a7);
  a1=(a7*a1);
  a22=(a23*a1);
  a54=(a54+a22);
  a22=(a13*a37);
  a22=(a22+a14);
  a7=(a7*a33);
  a55=(a22*a7);
  a24=(a20*a14);
  a24=(a24-a37);
  a0=arg[1]? arg[1][1] : 0;
  a2=(a2*a33);
  a2=(a0+a2);
  a24=(a24*a2);
  a11=(a11*a37);
  a13=(a13*a14);
  a11=(a11+a13);
  a58=(a58*a33);
  a33=(a11*a58);
  a24=(a24-a33);
  a55=(a55+a24);
  a24=(a25*a55);
  a33=arg[1]? arg[1][2] : 0;
  a13=(a20*a7);
  a13=(a13+a58);
  a13=(a33+a13);
  a51=(a6*a13);
  a24=(a24-a51);
  a24=(a54-a24);
  a51=(a59*a24);
  a28=sin(a28);
  a4=(a22*a5);
  a41=(a11*a1);
  a4=(a4+a41);
  a41=(a43*a7);
  a37=(a20*a37);
  a14=(a14+a37);
  a14=(a14*a2);
  a2=(a23*a58);
  a14=(a14-a2);
  a41=(a41+a14);
  a14=(a25*a41);
  a14=(a4+a14);
  a2=(a28*a14);
  a51=(a51-a2);
  a2=arg[1]? arg[1][3] : 0;
  a13=(a2+a13);
  a37=(a27*a13);
  a37=(a51+a37);
  a3=(a60*a37);
  a34=sin(a34);
  a14=(a59*a14);
  a24=(a28*a24);
  a14=(a14+a24);
  a24=(a17*a13);
  a24=(a14-a24);
  a19=(a34*a24);
  a3=(a3-a19);
  a19=arg[1]? arg[1][4] : 0;
  a13=(a19+a13);
  a35=(a18*a13);
  a35=(a3+a35);
  a8=(a50*a35);
  a40=sin(a40);
  a24=(a60*a24);
  a37=(a34*a37);
  a24=(a24+a37);
  a37=(a39*a13);
  a37=(a24-a37);
  a29=(a40*a37);
  a8=(a8-a29);
  a29=arg[1]? arg[1][5] : 0;
  a13=(a29+a13);
  a32=(a30*a13);
  a32=(a8+a32);
  a15=(a44*a32);
  a46=sin(a46);
  a37=(a50*a37);
  a35=(a40*a35);
  a37=(a37+a35);
  a13=(a45*a13);
  a13=(a37-a13);
  a35=(a46*a13);
  a15=(a15-a35);
  a15=(a38*a15);
  a8=(a29*a8);
  a3=(a19*a3);
  a51=(a2*a51);
  a54=(a33*a54);
  a1=(a0*a1);
  a35=(a22*a1);
  a5=(a0*a5);
  a31=(a11*a5);
  a35=(a35-a31);
  a54=(a54+a35);
  a35=(a33*a55);
  a58=(a0*a58);
  a31=(a43*a58);
  a0=(a0*a7);
  a7=(a23*a0);
  a31=(a31+a7);
  a35=(a35+a31);
  a31=(a25*a35);
  a54=(a54-a31);
  a31=(a59*a54);
  a43=(a43*a1);
  a23=(a23*a5);
  a43=(a43-a23);
  a4=(a33*a4);
  a43=(a43-a4);
  a33=(a33*a41);
  a22=(a22*a58);
  a11=(a11*a0);
  a22=(a22+a11);
  a33=(a33-a22);
  a25=(a25*a33);
  a58=(a20*a58);
  a0=(a0-a58);
  a58=(a6*a0);
  a25=(a25-a58);
  a43=(a43-a25);
  a25=(a28*a43);
  a31=(a31+a25);
  a51=(a51+a31);
  a31=(a17*a0);
  a51=(a51-a31);
  a31=(a60*a51);
  a43=(a59*a43);
  a54=(a28*a54);
  a43=(a43-a54);
  a14=(a2*a14);
  a43=(a43-a14);
  a14=(a27*a0);
  a43=(a43+a14);
  a14=(a34*a43);
  a31=(a31+a14);
  a3=(a3+a31);
  a31=(a39*a0);
  a3=(a3-a31);
  a31=(a50*a3);
  a43=(a60*a43);
  a51=(a34*a51);
  a43=(a43-a51);
  a24=(a19*a24);
  a43=(a43-a24);
  a24=(a18*a0);
  a43=(a43+a24);
  a24=(a40*a43);
  a31=(a31+a24);
  a8=(a8+a31);
  a31=(a45*a0);
  a8=(a8-a31);
  a31=(a44*a8);
  a43=(a50*a43);
  a3=(a40*a3);
  a43=(a43-a3);
  a37=(a29*a37);
  a43=(a43-a37);
  a37=(a30*a0);
  a43=(a43+a37);
  a37=(a46*a43);
  a31=(a31+a37);
  a15=(a15+a31);
  a48=(a48*a15);
  a10=(a10*a47);
  a42=(a42*a49);
  a10=(a10-a42);
  a43=(a44*a43);
  a8=(a46*a8);
  a43=(a43-a8);
  a13=(a44*a13);
  a32=(a46*a32);
  a13=(a13+a32);
  a13=(a38*a13);
  a43=(a43-a13);
  a0=(a36*a0);
  a43=(a43+a0);
  a10=(a10*a43);
  a20=(a20*a1);
  a20=(a20+a5);
  a6=(a6*a35);
  a20=(a20+a6);
  a6=(a59*a55);
  a5=(a28*a41);
  a6=(a6+a5);
  a5=(a2*a6);
  a1=(a28*a33);
  a0=(a59*a35);
  a1=(a1+a0);
  a5=(a5+a1);
  a27=(a27*a5);
  a41=(a59*a41);
  a55=(a28*a55);
  a41=(a41-a55);
  a2=(a2*a41);
  a59=(a59*a33);
  a28=(a28*a35);
  a59=(a59-a28);
  a2=(a2+a59);
  a17=(a17*a2);
  a27=(a27+a17);
  a20=(a20+a27);
  a27=(a60*a6);
  a17=(a34*a41);
  a27=(a27+a17);
  a17=(a19*a27);
  a59=(a34*a2);
  a28=(a60*a5);
  a59=(a59+a28);
  a17=(a17+a59);
  a18=(a18*a17);
  a41=(a60*a41);
  a6=(a34*a6);
  a41=(a41-a6);
  a19=(a19*a41);
  a60=(a60*a2);
  a34=(a34*a5);
  a60=(a60-a34);
  a19=(a19+a60);
  a39=(a39*a19);
  a18=(a18+a39);
  a20=(a20+a18);
  a18=(a50*a27);
  a39=(a40*a41);
  a18=(a18+a39);
  a39=(a29*a18);
  a60=(a40*a19);
  a34=(a50*a17);
  a60=(a60+a34);
  a39=(a39+a60);
  a30=(a30*a39);
  a41=(a50*a41);
  a27=(a40*a27);
  a41=(a41-a27);
  a29=(a29*a41);
  a50=(a50*a19);
  a40=(a40*a17);
  a50=(a50-a40);
  a29=(a29+a50);
  a45=(a45*a29);
  a30=(a30+a45);
  a20=(a20+a30);
  a18=(a44*a18);
  a41=(a46*a41);
  a18=(a18+a41);
  a38=(a38*a18);
  a46=(a46*a29);
  a44=(a44*a39);
  a46=(a46+a44);
  a38=(a38+a46);
  a36=(a36*a38);
  a20=(a20+a36);
  a26=(a26*a20);
  a10=(a10+a26);
  a48=(a48+a10);
  if (res[2]!=0) res[2][0]=a48;
  a53=(a53*a15);
  a52=(a52*a47);
  a57=(a57*a49);
  a52=(a52-a57);
  a52=(a52*a43);
  a56=(a56*a20);
  a52=(a52+a56);
  a53=(a53+a52);
  if (res[2]!=0) res[2][1]=a53;
  a16=(a16*a15);
  a61=(a61*a47);
  a12=(a12*a49);
  a61=(a61-a12);
  a61=(a61*a43);
  a21=(a21*a20);
  a61=(a61+a21);
  a16=(a16+a61);
  if (res[2]!=0) res[2][2]=a16;
  return 0;
}

CASADI_SYMBOL_EXPORT int cassie_left_foot_back(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int cassie_left_foot_back_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int cassie_left_foot_back_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void cassie_left_foot_back_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int cassie_left_foot_back_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void cassie_left_foot_back_release(int mem) {
}

CASADI_SYMBOL_EXPORT void cassie_left_foot_back_incref(void) {
}

CASADI_SYMBOL_EXPORT void cassie_left_foot_back_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int cassie_left_foot_back_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int cassie_left_foot_back_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real cassie_left_foot_back_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* cassie_left_foot_back_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* cassie_left_foot_back_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* cassie_left_foot_back_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* cassie_left_foot_back_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    case 1: return casadi_s2;
    case 2: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int cassie_left_foot_back_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif