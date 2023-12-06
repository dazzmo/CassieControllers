/* This file was automatically generated by CasADi 3.6.4.
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
  #define CASADI_PREFIX(ID) cassie_left_ankle_ ## ID
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

/* cassie_left_ankle:(i0[16],i1[16])->(o0[3],o1[3x16],o2[3]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a6, a7, a8, a9;
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
  a11=1.0000000000000002e+00;
  a12=(a11*a8);
  a13=(a11*a12);
  a3=sin(a3);
  a14=(a2*a3);
  a15=sin(a7);
  a16=(a14*a15);
  a13=(a13-a16);
  a10=(a10+a13);
  a13=4.8966386501092529e-12;
  a16=arg[0]? arg[0][2] : 0;
  a17=cos(a16);
  a18=(a13*a17);
  a19=sin(a16);
  a18=(a18+a19);
  a20=(a10*a18);
  a21=-1.0000000000000002e+00;
  a22=(a21*a5);
  a23=-4.4408920985006271e-16;
  a22=(a22+a23);
  a23=4.8965831389580217e-12;
  a24=(a23*a19);
  a24=(a24-a17);
  a25=(a22*a24);
  a26=(a2*a15);
  a5=(a5*a26);
  a14=(a14*a8);
  a27=(a11*a15);
  a28=(a11*a27);
  a14=(a14+a28);
  a5=(a5+a14);
  a14=5.5511151231257827e-17;
  a28=(a14*a17);
  a29=(a13*a19);
  a28=(a28+a29);
  a29=(a5*a28);
  a25=(a25-a29);
  a20=(a20+a25);
  a25=(a6*a20);
  a29=4.4999999999999997e-03;
  a30=(a23*a10);
  a30=(a30+a5);
  a31=(a29*a30);
  a25=(a25+a31);
  a25=(a0+a25);
  a31=6.0679999999999998e-02;
  a32=arg[0]? arg[0][3] : 0;
  a33=cos(a32);
  a34=(a20*a33);
  a35=(a13*a19);
  a35=(a17-a35);
  a10=(a10*a35);
  a36=(a23*a17);
  a36=(a19+a36);
  a37=(a22*a36);
  a17=(a13*a17);
  a19=(a14*a19);
  a17=(a17-a19);
  a5=(a5*a17);
  a37=(a37-a5);
  a10=(a10+a37);
  a37=sin(a32);
  a5=(a10*a37);
  a34=(a34+a5);
  a5=(a31*a34);
  a19=4.7410000000000001e-02;
  a10=(a10*a33);
  a20=(a20*a37);
  a10=(a10-a20);
  a20=(a19*a10);
  a5=(a5+a20);
  a5=(a25+a5);
  a20=4.3475999999999998e-01;
  a38=arg[0]? arg[0][4] : 0;
  a39=cos(a38);
  a40=(a34*a39);
  a41=sin(a38);
  a42=(a10*a41);
  a40=(a40+a42);
  a42=(a20*a40);
  a43=2.0000000000000000e-02;
  a10=(a10*a39);
  a34=(a34*a41);
  a10=(a10-a34);
  a34=(a43*a10);
  a42=(a42+a34);
  a42=(a5+a42);
  a34=4.0799999999999997e-01;
  a44=arg[0]? arg[0][5] : 0;
  a45=cos(a44);
  a46=(a40*a45);
  a47=sin(a44);
  a48=(a10*a47);
  a46=(a46+a48);
  a48=(a34*a46);
  a49=-4.0000000000000001e-02;
  a10=(a10*a45);
  a40=(a40*a47);
  a10=(a10-a40);
  a40=(a49*a10);
  a48=(a48+a40);
  a48=(a42+a48);
  if (res[0]!=0) res[0][0]=a48;
  a40=1.3500000000000001e-01;
  a50=(a1*a3);
  a40=(a40+a50);
  a50=(a3*a9);
  a51=(a4*a15);
  a50=(a50+a51);
  a51=(a50*a18);
  a52=(a4*a8);
  a53=(a3*a26);
  a52=(a52-a53);
  a53=(a52*a28);
  a54=(a21*a3);
  a55=(a54*a24);
  a53=(a53+a55);
  a51=(a51+a53);
  a53=(a6*a51);
  a55=(a23*a50);
  a55=(a55-a52);
  a56=(a29*a55);
  a53=(a53+a56);
  a53=(a40+a53);
  a56=(a51*a33);
  a50=(a50*a35);
  a52=(a52*a17);
  a57=(a54*a36);
  a52=(a52+a57);
  a50=(a50+a52);
  a52=(a50*a37);
  a56=(a56+a52);
  a52=(a31*a56);
  a50=(a50*a33);
  a51=(a51*a37);
  a50=(a50-a51);
  a51=(a19*a50);
  a52=(a52+a51);
  a52=(a53+a52);
  a51=(a56*a39);
  a57=(a50*a41);
  a51=(a51+a57);
  a57=(a20*a51);
  a50=(a50*a39);
  a56=(a56*a41);
  a50=(a50-a56);
  a56=(a43*a50);
  a57=(a57+a56);
  a57=(a52+a57);
  a56=(a51*a45);
  a58=(a50*a47);
  a56=(a56+a58);
  a58=(a34*a56);
  a50=(a50*a45);
  a51=(a51*a47);
  a50=(a50-a51);
  a51=(a49*a50);
  a58=(a58+a51);
  a58=(a57+a58);
  if (res[0]!=0) res[0][1]=a58;
  a4=(a21*a4);
  a51=(a1*a4);
  a9=(a4*a9);
  a12=(a2*a12);
  a3=(a21*a3);
  a15=(a3*a15);
  a12=(a12-a15);
  a9=(a9+a12);
  a18=(a9*a18);
  a21=(a21*a4);
  a12=1.9721522630525295e-31;
  a21=(a21+a12);
  a24=(a21*a24);
  a4=(a4*a26);
  a3=(a3*a8);
  a27=(a2*a27);
  a3=(a3+a27);
  a4=(a4+a3);
  a28=(a4*a28);
  a24=(a24-a28);
  a18=(a18+a24);
  a24=(a6*a18);
  a28=(a23*a9);
  a28=(a28+a4);
  a3=(a29*a28);
  a24=(a24+a3);
  a24=(a51+a24);
  a3=(a18*a33);
  a9=(a9*a35);
  a36=(a21*a36);
  a4=(a4*a17);
  a36=(a36-a4);
  a9=(a9+a36);
  a36=(a9*a37);
  a3=(a3+a36);
  a36=(a31*a3);
  a9=(a9*a33);
  a18=(a18*a37);
  a9=(a9-a18);
  a18=(a19*a9);
  a36=(a36+a18);
  a36=(a24+a36);
  a18=(a3*a39);
  a37=(a9*a41);
  a18=(a18+a37);
  a37=(a20*a18);
  a9=(a9*a39);
  a3=(a3*a41);
  a9=(a9-a3);
  a3=(a43*a9);
  a37=(a37+a3);
  a37=(a36+a37);
  a3=(a18*a45);
  a41=(a9*a47);
  a3=(a3+a41);
  a41=(a34*a3);
  a9=(a9*a45);
  a18=(a18*a47);
  a9=(a9-a18);
  a18=(a49*a9);
  a41=(a41+a18);
  a41=(a37+a41);
  if (res[0]!=0) res[0][2]=a41;
  a18=-5.9952043329758457e-17;
  a47=(a2*a58);
  a18=(a18-a47);
  if (res[1]!=0) res[1][0]=a18;
  a18=-2.1760371282653069e-17;
  a47=(a11*a41);
  a45=(a2*a48);
  a47=(a47-a45);
  a18=(a18-a47);
  if (res[1]!=0) res[1][1]=a18;
  a18=-1.3500000000000004e-01;
  a47=(a11*a58);
  a18=(a18+a47);
  if (res[1]!=0) res[1][2]=a18;
  a18=(a40*a21);
  a47=(a51*a54);
  a18=(a18-a47);
  a47=(a58*a21);
  a45=(a41*a54);
  a47=(a47-a45);
  a18=(a18-a47);
  if (res[1]!=0) res[1][3]=a18;
  a18=(a51*a22);
  a47=(a0*a21);
  a18=(a18-a47);
  a47=(a41*a22);
  a21=(a48*a21);
  a47=(a47-a21);
  a18=(a18-a47);
  if (res[1]!=0) res[1][4]=a18;
  a18=(a0*a54);
  a47=(a40*a22);
  a18=(a18-a47);
  a54=(a48*a54);
  a22=(a58*a22);
  a54=(a54-a22);
  a18=(a18-a54);
  if (res[1]!=0) res[1][5]=a18;
  a18=(a40*a28);
  a54=(a51*a55);
  a18=(a18-a54);
  a54=(a58*a28);
  a22=(a41*a55);
  a54=(a54-a22);
  a18=(a18-a54);
  if (res[1]!=0) res[1][6]=a18;
  a51=(a51*a30);
  a18=(a0*a28);
  a51=(a51-a18);
  a18=(a41*a30);
  a54=(a48*a28);
  a18=(a18-a54);
  a51=(a51-a18);
  if (res[1]!=0) res[1][7]=a51;
  a0=(a0*a55);
  a40=(a40*a30);
  a0=(a0-a40);
  a40=(a48*a55);
  a51=(a58*a30);
  a40=(a40-a51);
  a0=(a0-a40);
  if (res[1]!=0) res[1][8]=a0;
  a0=(a53*a28);
  a40=(a24*a55);
  a0=(a0-a40);
  a40=(a58*a28);
  a51=(a41*a55);
  a40=(a40-a51);
  a0=(a0-a40);
  if (res[1]!=0) res[1][9]=a0;
  a24=(a24*a30);
  a0=(a25*a28);
  a24=(a24-a0);
  a0=(a41*a30);
  a40=(a48*a28);
  a0=(a0-a40);
  a24=(a24-a0);
  if (res[1]!=0) res[1][10]=a24;
  a25=(a25*a55);
  a53=(a53*a30);
  a25=(a25-a53);
  a53=(a48*a55);
  a24=(a58*a30);
  a53=(a53-a24);
  a25=(a25-a53);
  if (res[1]!=0) res[1][11]=a25;
  a25=(a52*a28);
  a53=(a36*a55);
  a25=(a25-a53);
  a53=(a58*a28);
  a24=(a41*a55);
  a53=(a53-a24);
  a25=(a25-a53);
  if (res[1]!=0) res[1][12]=a25;
  a36=(a36*a30);
  a25=(a5*a28);
  a36=(a36-a25);
  a25=(a41*a30);
  a53=(a48*a28);
  a25=(a25-a53);
  a36=(a36-a25);
  if (res[1]!=0) res[1][13]=a36;
  a5=(a5*a55);
  a52=(a52*a30);
  a5=(a5-a52);
  a52=(a48*a55);
  a36=(a58*a30);
  a52=(a52-a36);
  a5=(a5-a52);
  if (res[1]!=0) res[1][14]=a5;
  a5=(a57*a28);
  a52=(a37*a55);
  a5=(a5-a52);
  a52=(a58*a28);
  a36=(a41*a55);
  a52=(a52-a36);
  a5=(a5-a52);
  if (res[1]!=0) res[1][15]=a5;
  a37=(a37*a30);
  a5=(a42*a28);
  a37=(a37-a5);
  a5=(a41*a30);
  a52=(a48*a28);
  a5=(a5-a52);
  a37=(a37-a5);
  if (res[1]!=0) res[1][16]=a37;
  a42=(a42*a55);
  a57=(a57*a30);
  a42=(a42-a57);
  a57=(a48*a55);
  a37=(a58*a30);
  a57=(a57-a37);
  a42=(a42-a57);
  if (res[1]!=0) res[1][17]=a42;
  a42=0.;
  if (res[1]!=0) res[1][18]=a42;
  if (res[1]!=0) res[1][19]=a42;
  if (res[1]!=0) res[1][20]=a42;
  a57=(a58*a28);
  a37=(a41*a55);
  a57=(a57-a37);
  a37=(a58*a28);
  a5=(a41*a55);
  a37=(a37-a5);
  a57=(a57-a37);
  if (res[1]!=0) res[1][21]=a57;
  a57=(a41*a30);
  a37=(a48*a28);
  a57=(a57-a37);
  a41=(a41*a30);
  a37=(a48*a28);
  a41=(a41-a37);
  a57=(a57-a41);
  if (res[1]!=0) res[1][22]=a57;
  a57=(a48*a55);
  a41=(a58*a30);
  a57=(a57-a41);
  a48=(a48*a55);
  a58=(a58*a30);
  a48=(a48-a58);
  a57=(a57-a48);
  if (res[1]!=0) res[1][23]=a57;
  if (res[1]!=0) res[1][24]=a42;
  if (res[1]!=0) res[1][25]=a42;
  if (res[1]!=0) res[1][26]=a42;
  if (res[1]!=0) res[1][27]=a42;
  if (res[1]!=0) res[1][28]=a42;
  if (res[1]!=0) res[1][29]=a42;
  if (res[1]!=0) res[1][30]=a42;
  if (res[1]!=0) res[1][31]=a42;
  if (res[1]!=0) res[1][32]=a42;
  if (res[1]!=0) res[1][33]=a42;
  if (res[1]!=0) res[1][34]=a42;
  if (res[1]!=0) res[1][35]=a42;
  if (res[1]!=0) res[1][36]=a42;
  if (res[1]!=0) res[1][37]=a42;
  if (res[1]!=0) res[1][38]=a42;
  if (res[1]!=0) res[1][39]=a42;
  if (res[1]!=0) res[1][40]=a42;
  if (res[1]!=0) res[1][41]=a42;
  if (res[1]!=0) res[1][42]=a42;
  if (res[1]!=0) res[1][43]=a42;
  if (res[1]!=0) res[1][44]=a42;
  if (res[1]!=0) res[1][45]=a42;
  if (res[1]!=0) res[1][46]=a42;
  if (res[1]!=0) res[1][47]=a42;
  a42=arg[0]? arg[0][7] : 0;
  a57=cos(a42);
  a48=(a46*a57);
  a58=sin(a42);
  a41=(a10*a58);
  a48=(a48+a41);
  a41=arg[1]? arg[1][7] : 0;
  a37=cos(a42);
  a5=cos(a44);
  a52=cos(a38);
  a36=cos(a32);
  a25=cos(a16);
  a16=sin(a16);
  a53=(a13*a16);
  a53=(a25-a53);
  a24=sin(a7);
  a0=arg[1]? arg[1][0] : 0;
  a1=(a1*a0);
  a40=(a24*a1);
  a51=(a53*a40);
  a18=(a13*a25);
  a54=(a14*a16);
  a18=(a18-a54);
  a7=cos(a7);
  a1=(a7*a1);
  a54=(a18*a1);
  a51=(a51+a54);
  a54=(a13*a25);
  a54=(a54+a16);
  a7=(a11*a7);
  a7=(a7*a0);
  a22=(a54*a7);
  a47=(a23*a16);
  a47=(a47-a25);
  a21=arg[1]? arg[1][1] : 0;
  a2=(a2*a0);
  a2=(a21+a2);
  a47=(a47*a2);
  a14=(a14*a25);
  a13=(a13*a16);
  a14=(a14+a13);
  a11=(a11*a24);
  a11=(a11*a0);
  a0=(a14*a11);
  a47=(a47-a0);
  a22=(a22+a47);
  a47=(a29*a22);
  a0=arg[1]? arg[1][2] : 0;
  a24=(a23*a7);
  a24=(a24+a11);
  a24=(a0+a24);
  a13=(a6*a24);
  a47=(a47-a13);
  a47=(a51-a47);
  a13=(a36*a47);
  a32=sin(a32);
  a45=(a54*a40);
  a39=(a14*a1);
  a45=(a45+a39);
  a39=(a53*a7);
  a25=(a23*a25);
  a16=(a16+a25);
  a16=(a16*a2);
  a2=(a18*a11);
  a16=(a16-a2);
  a39=(a39+a16);
  a16=(a29*a39);
  a16=(a45+a16);
  a2=(a32*a16);
  a13=(a13-a2);
  a2=arg[1]? arg[1][3] : 0;
  a24=(a2+a24);
  a25=(a31*a24);
  a25=(a13+a25);
  a33=(a52*a25);
  a38=sin(a38);
  a16=(a36*a16);
  a47=(a32*a47);
  a16=(a16+a47);
  a47=(a19*a24);
  a47=(a16-a47);
  a4=(a38*a47);
  a33=(a33-a4);
  a4=arg[1]? arg[1][4] : 0;
  a24=(a4+a24);
  a17=(a20*a24);
  a17=(a33+a17);
  a35=(a5*a17);
  a44=sin(a44);
  a47=(a52*a47);
  a25=(a38*a25);
  a47=(a47+a25);
  a25=(a43*a24);
  a25=(a47-a25);
  a27=(a44*a25);
  a35=(a35-a27);
  a27=arg[1]? arg[1][5] : 0;
  a24=(a27+a24);
  a8=(a34*a24);
  a8=(a35+a8);
  a26=(a37*a8);
  a42=sin(a42);
  a25=(a5*a25);
  a17=(a44*a17);
  a25=(a25+a17);
  a24=(a49*a24);
  a24=(a25-a24);
  a17=(a42*a24);
  a26=(a26-a17);
  a26=(a41*a26);
  a35=(a27*a35);
  a33=(a4*a33);
  a13=(a2*a13);
  a51=(a0*a51);
  a1=(a21*a1);
  a17=(a54*a1);
  a40=(a21*a40);
  a12=(a14*a40);
  a17=(a17-a12);
  a51=(a51+a17);
  a17=(a0*a22);
  a11=(a21*a11);
  a12=(a53*a11);
  a21=(a21*a7);
  a7=(a18*a21);
  a12=(a12+a7);
  a17=(a17+a12);
  a12=(a29*a17);
  a51=(a51-a12);
  a12=(a36*a51);
  a53=(a53*a1);
  a18=(a18*a40);
  a53=(a53-a18);
  a45=(a0*a45);
  a53=(a53-a45);
  a0=(a0*a39);
  a54=(a54*a11);
  a14=(a14*a21);
  a54=(a54+a14);
  a0=(a0-a54);
  a29=(a29*a0);
  a11=(a23*a11);
  a21=(a21-a11);
  a11=(a6*a21);
  a29=(a29-a11);
  a53=(a53-a29);
  a29=(a32*a53);
  a12=(a12+a29);
  a13=(a13+a12);
  a12=(a19*a21);
  a13=(a13-a12);
  a12=(a52*a13);
  a53=(a36*a53);
  a51=(a32*a51);
  a53=(a53-a51);
  a16=(a2*a16);
  a53=(a53-a16);
  a16=(a31*a21);
  a53=(a53+a16);
  a16=(a38*a53);
  a12=(a12+a16);
  a33=(a33+a12);
  a12=(a43*a21);
  a33=(a33-a12);
  a12=(a5*a33);
  a53=(a52*a53);
  a13=(a38*a13);
  a53=(a53-a13);
  a47=(a4*a47);
  a53=(a53-a47);
  a47=(a20*a21);
  a53=(a53+a47);
  a47=(a44*a53);
  a12=(a12+a47);
  a35=(a35+a12);
  a12=(a49*a21);
  a35=(a35-a12);
  a12=(a37*a35);
  a53=(a5*a53);
  a33=(a44*a33);
  a53=(a53-a33);
  a25=(a27*a25);
  a53=(a53-a25);
  a21=(a34*a21);
  a53=(a53+a21);
  a21=(a42*a53);
  a12=(a12+a21);
  a26=(a26+a12);
  a48=(a48*a26);
  a10=(a10*a57);
  a46=(a46*a58);
  a10=(a10-a46);
  a53=(a37*a53);
  a35=(a42*a35);
  a53=(a53-a35);
  a37=(a37*a24);
  a42=(a42*a8);
  a37=(a37+a42);
  a41=(a41*a37);
  a53=(a53-a41);
  a10=(a10*a53);
  a23=(a23*a1);
  a23=(a23+a40);
  a6=(a6*a17);
  a23=(a23+a6);
  a6=(a36*a22);
  a40=(a32*a39);
  a6=(a6+a40);
  a40=(a2*a6);
  a1=(a32*a0);
  a41=(a36*a17);
  a1=(a1+a41);
  a40=(a40+a1);
  a31=(a31*a40);
  a39=(a36*a39);
  a22=(a32*a22);
  a39=(a39-a22);
  a2=(a2*a39);
  a36=(a36*a0);
  a32=(a32*a17);
  a36=(a36-a32);
  a2=(a2+a36);
  a19=(a19*a2);
  a31=(a31+a19);
  a23=(a23+a31);
  a31=(a52*a6);
  a19=(a38*a39);
  a31=(a31+a19);
  a19=(a4*a31);
  a36=(a38*a2);
  a32=(a52*a40);
  a36=(a36+a32);
  a19=(a19+a36);
  a20=(a20*a19);
  a39=(a52*a39);
  a6=(a38*a6);
  a39=(a39-a6);
  a4=(a4*a39);
  a52=(a52*a2);
  a38=(a38*a40);
  a52=(a52-a38);
  a4=(a4+a52);
  a43=(a43*a4);
  a20=(a20+a43);
  a23=(a23+a20);
  a20=(a5*a31);
  a43=(a44*a39);
  a20=(a20+a43);
  a20=(a27*a20);
  a43=(a44*a4);
  a52=(a5*a19);
  a43=(a43+a52);
  a20=(a20+a43);
  a34=(a34*a20);
  a39=(a5*a39);
  a31=(a44*a31);
  a39=(a39-a31);
  a27=(a27*a39);
  a5=(a5*a4);
  a44=(a44*a19);
  a5=(a5-a44);
  a27=(a27+a5);
  a49=(a49*a27);
  a34=(a34+a49);
  a23=(a23+a34);
  a30=(a30*a23);
  a10=(a10+a30);
  a48=(a48+a10);
  if (res[2]!=0) res[2][0]=a48;
  a48=(a56*a57);
  a10=(a50*a58);
  a48=(a48+a10);
  a48=(a48*a26);
  a50=(a50*a57);
  a56=(a56*a58);
  a50=(a50-a56);
  a50=(a50*a53);
  a55=(a55*a23);
  a50=(a50+a55);
  a48=(a48+a50);
  if (res[2]!=0) res[2][1]=a48;
  a48=(a3*a57);
  a50=(a9*a58);
  a48=(a48+a50);
  a48=(a48*a26);
  a9=(a9*a57);
  a3=(a3*a58);
  a9=(a9-a3);
  a9=(a9*a53);
  a28=(a28*a23);
  a9=(a9+a28);
  a48=(a48+a9);
  if (res[2]!=0) res[2][2]=a48;
  return 0;
}

CASADI_SYMBOL_EXPORT int cassie_left_ankle(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int cassie_left_ankle_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int cassie_left_ankle_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void cassie_left_ankle_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int cassie_left_ankle_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void cassie_left_ankle_release(int mem) {
}

CASADI_SYMBOL_EXPORT void cassie_left_ankle_incref(void) {
}

CASADI_SYMBOL_EXPORT void cassie_left_ankle_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int cassie_left_ankle_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int cassie_left_ankle_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real cassie_left_ankle_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* cassie_left_ankle_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* cassie_left_ankle_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* cassie_left_ankle_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* cassie_left_ankle_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    case 1: return casadi_s2;
    case 2: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int cassie_left_ankle_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
