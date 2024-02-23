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
  #define CASADI_PREFIX(ID) cassie_LeftFootFront_ ## ID
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
#define casadi_s3 CASADI_PREFIX(s3)

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

static const casadi_int casadi_s0[27] = {23, 1, 0, 23, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
static const casadi_int casadi_s1[26] = {22, 1, 0, 22, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21};
static const casadi_int casadi_s2[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s3[91] = {3, 22, 0, 3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48, 51, 54, 57, 60, 63, 66, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2};

/* cassie_LeftFootFront:(i0[23],i1[22])->(o0[3],o1[3x22],o2[3]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a8, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a9, a90, a91;
  a0=arg[0]? arg[0][0] : 0;
  a1=-4.9000000000000002e-02;
  a2=1.;
  a3=2.;
  a4=arg[0]? arg[0][4] : 0;
  a5=(a3*a4);
  a6=(a5*a4);
  a7=arg[0]? arg[0][5] : 0;
  a8=(a3*a7);
  a7=(a8*a7);
  a9=(a6+a7);
  a9=(a2-a9);
  a10=(a1*a9);
  a11=1.3500000000000001e-01;
  a12=arg[0]? arg[0][3] : 0;
  a13=(a5*a12);
  a14=arg[0]? arg[0][6] : 0;
  a15=(a8*a14);
  a16=(a13-a15);
  a17=(a11*a16);
  a10=(a10+a17);
  a10=(a0+a10);
  a17=8.9999999999999997e-02;
  a18=-4.4408920985006262e-16;
  a19=arg[0]? arg[0][7] : 0;
  a20=cos(a19);
  a21=(a18*a20);
  a22=(a9*a21);
  a23=sin(a19);
  a24=(a16*a23);
  a25=(a8*a12);
  a5=(a5*a14);
  a26=(a25+a5);
  a27=-1.0000000000000002e+00;
  a28=(a27*a20);
  a29=(a26*a28);
  a24=(a24+a29);
  a22=(a22+a24);
  a24=(a17*a22);
  a24=(a10+a24);
  a29=1.2000000000000000e-01;
  a30=arg[0]? arg[0][8] : 0;
  a31=cos(a30);
  a32=(a18*a31);
  a33=(a22*a32);
  a34=(a16*a20);
  a35=(a27*a23);
  a36=(a26*a35);
  a34=(a34-a36);
  a36=(a18*a23);
  a37=(a9*a36);
  a34=(a34-a37);
  a37=sin(a30);
  a38=(a34*a37);
  a39=1.0000000000000002e+00;
  a40=(a39*a9);
  a41=(a18*a26);
  a40=(a40+a41);
  a41=(a39*a31);
  a42=(a40*a41);
  a38=(a38+a42);
  a33=(a33+a38);
  a38=4.8966386501092529e-12;
  a42=arg[0]? arg[0][9] : 0;
  a43=cos(a42);
  a44=(a38*a43);
  a45=sin(a42);
  a44=(a44+a45);
  a46=(a33*a44);
  a34=(a34*a31);
  a47=(a39*a37);
  a48=(a40*a47);
  a34=(a34-a48);
  a48=(a18*a37);
  a49=(a22*a48);
  a34=(a34-a49);
  a49=5.5511151231257827e-17;
  a50=(a49*a43);
  a51=(a38*a45);
  a50=(a50+a51);
  a51=(a34*a50);
  a22=(a27*a22);
  a52=(a18*a40);
  a22=(a22+a52);
  a52=4.8965831389580217e-12;
  a53=(a52*a45);
  a53=(a53-a43);
  a54=(a22*a53);
  a51=(a51+a54);
  a46=(a46+a51);
  a51=(a29*a46);
  a54=4.4999999999999997e-03;
  a55=(a52*a33);
  a55=(a55-a34);
  a56=(a54*a55);
  a51=(a51+a56);
  a51=(a24+a51);
  a56=6.0679999999999998e-02;
  a57=arg[0]? arg[0][10] : 0;
  a58=cos(a57);
  a59=(a46*a58);
  a60=(a38*a45);
  a60=(a43-a60);
  a33=(a33*a60);
  a61=(a38*a43);
  a62=(a49*a45);
  a61=(a61-a62);
  a34=(a34*a61);
  a43=(a52*a43);
  a45=(a45+a43);
  a43=(a22*a45);
  a34=(a34+a43);
  a33=(a33+a34);
  a34=sin(a57);
  a43=(a33*a34);
  a59=(a59+a43);
  a43=(a56*a59);
  a62=4.7410000000000001e-02;
  a33=(a33*a58);
  a46=(a46*a34);
  a33=(a33-a46);
  a46=(a62*a33);
  a43=(a43+a46);
  a43=(a51+a43);
  a46=4.3475999999999998e-01;
  a63=arg[0]? arg[0][11] : 0;
  a64=cos(a63);
  a65=(a59*a64);
  a66=sin(a63);
  a67=(a33*a66);
  a65=(a65+a67);
  a67=(a46*a65);
  a68=2.0000000000000000e-02;
  a33=(a33*a64);
  a59=(a59*a66);
  a33=(a33-a59);
  a59=(a68*a33);
  a67=(a67+a59);
  a67=(a43+a67);
  a59=4.0799999999999997e-01;
  a69=arg[0]? arg[0][12] : 0;
  a70=cos(a69);
  a71=(a65*a70);
  a72=sin(a69);
  a73=(a33*a72);
  a71=(a71+a73);
  a73=(a59*a71);
  a74=-4.0000000000000001e-02;
  a33=(a33*a70);
  a65=(a65*a72);
  a33=(a33-a65);
  a65=(a74*a33);
  a73=(a73+a65);
  a73=(a67+a73);
  a65=-4.0700000000000000e-02;
  a75=arg[0]? arg[0][14] : 0;
  a76=cos(a75);
  a77=(a71*a76);
  a78=sin(a75);
  a79=(a33*a78);
  a77=(a77+a79);
  a79=(a65*a77);
  a80=1.0700000000000000e-01;
  a33=(a33*a76);
  a71=(a71*a78);
  a33=(a33-a71);
  a71=(a80*a33);
  a79=(a79+a71);
  a79=(a73+a79);
  if (res[0]!=0) res[0][0]=a79;
  a71=arg[0]? arg[0][1] : 0;
  a13=(a13+a15);
  a15=(a1*a13);
  a3=(a3*a12);
  a12=(a3*a12);
  a7=(a12+a7);
  a7=(a2-a7);
  a81=(a11*a7);
  a15=(a15+a81);
  a15=(a71+a15);
  a81=(a13*a21);
  a82=(a7*a23);
  a8=(a8*a4);
  a3=(a3*a14);
  a14=(a8-a3);
  a4=(a14*a28);
  a82=(a82+a4);
  a81=(a81+a82);
  a82=(a17*a81);
  a82=(a15+a82);
  a4=(a81*a32);
  a83=(a7*a20);
  a84=(a14*a35);
  a83=(a83-a84);
  a84=(a13*a36);
  a83=(a83-a84);
  a84=(a83*a37);
  a85=(a39*a13);
  a86=(a18*a14);
  a85=(a85+a86);
  a86=(a85*a41);
  a84=(a84+a86);
  a4=(a4+a84);
  a84=(a4*a44);
  a83=(a83*a31);
  a86=(a85*a47);
  a83=(a83-a86);
  a86=(a81*a48);
  a83=(a83-a86);
  a86=(a83*a50);
  a81=(a27*a81);
  a87=(a18*a85);
  a81=(a81+a87);
  a87=(a81*a53);
  a86=(a86+a87);
  a84=(a84+a86);
  a86=(a29*a84);
  a87=(a52*a4);
  a87=(a87-a83);
  a88=(a54*a87);
  a86=(a86+a88);
  a86=(a82+a86);
  a88=(a84*a58);
  a4=(a4*a60);
  a83=(a83*a61);
  a89=(a81*a45);
  a83=(a83+a89);
  a4=(a4+a83);
  a83=(a4*a34);
  a88=(a88+a83);
  a83=(a56*a88);
  a4=(a4*a58);
  a84=(a84*a34);
  a4=(a4-a84);
  a84=(a62*a4);
  a83=(a83+a84);
  a83=(a86+a83);
  a84=(a88*a64);
  a89=(a4*a66);
  a84=(a84+a89);
  a89=(a46*a84);
  a4=(a4*a64);
  a88=(a88*a66);
  a4=(a4-a88);
  a88=(a68*a4);
  a89=(a89+a88);
  a89=(a83+a89);
  a88=(a84*a70);
  a90=(a4*a72);
  a88=(a88+a90);
  a90=(a59*a88);
  a4=(a4*a70);
  a84=(a84*a72);
  a4=(a4-a84);
  a84=(a74*a4);
  a90=(a90+a84);
  a90=(a89+a90);
  a84=(a88*a76);
  a91=(a4*a78);
  a84=(a84+a91);
  a91=(a65*a84);
  a4=(a4*a76);
  a88=(a88*a78);
  a4=(a4-a88);
  a88=(a80*a4);
  a91=(a91+a88);
  a91=(a90+a91);
  if (res[0]!=0) res[0][1]=a91;
  a88=arg[0]? arg[0][2] : 0;
  a25=(a25-a5);
  a5=(a1*a25);
  a8=(a8+a3);
  a3=(a11*a8);
  a5=(a5+a3);
  a5=(a88+a5);
  a21=(a25*a21);
  a23=(a8*a23);
  a12=(a12+a6);
  a2=(a2-a12);
  a28=(a2*a28);
  a23=(a23+a28);
  a21=(a21+a23);
  a23=(a17*a21);
  a23=(a5+a23);
  a32=(a21*a32);
  a20=(a8*a20);
  a35=(a2*a35);
  a20=(a20-a35);
  a36=(a25*a36);
  a20=(a20-a36);
  a37=(a20*a37);
  a36=(a39*a25);
  a35=(a18*a2);
  a36=(a36+a35);
  a41=(a36*a41);
  a37=(a37+a41);
  a32=(a32+a37);
  a44=(a32*a44);
  a20=(a20*a31);
  a47=(a36*a47);
  a20=(a20-a47);
  a48=(a21*a48);
  a20=(a20-a48);
  a50=(a20*a50);
  a21=(a27*a21);
  a48=(a18*a36);
  a21=(a21+a48);
  a53=(a21*a53);
  a50=(a50+a53);
  a44=(a44+a50);
  a50=(a29*a44);
  a53=(a52*a32);
  a53=(a53-a20);
  a48=(a54*a53);
  a50=(a50+a48);
  a50=(a23+a50);
  a48=(a44*a58);
  a32=(a32*a60);
  a20=(a20*a61);
  a45=(a21*a45);
  a20=(a20+a45);
  a32=(a32+a20);
  a20=(a32*a34);
  a48=(a48+a20);
  a20=(a56*a48);
  a32=(a32*a58);
  a44=(a44*a34);
  a32=(a32-a44);
  a44=(a62*a32);
  a20=(a20+a44);
  a20=(a50+a20);
  a44=(a48*a64);
  a34=(a32*a66);
  a44=(a44+a34);
  a34=(a46*a44);
  a32=(a32*a64);
  a48=(a48*a66);
  a32=(a32-a48);
  a48=(a68*a32);
  a34=(a34+a48);
  a34=(a20+a34);
  a48=(a44*a70);
  a66=(a32*a72);
  a48=(a48+a66);
  a66=(a59*a48);
  a32=(a32*a70);
  a44=(a44*a72);
  a32=(a32-a44);
  a44=(a74*a32);
  a66=(a66+a44);
  a66=(a34+a66);
  a44=(a48*a76);
  a72=(a32*a78);
  a44=(a44+a72);
  a72=(a65*a44);
  a32=(a32*a76);
  a48=(a48*a78);
  a32=(a32-a48);
  a48=(a80*a32);
  a72=(a72+a48);
  a72=(a66+a72);
  if (res[0]!=0) res[0][2]=a72;
  if (res[1]!=0) res[1][0]=a9;
  if (res[1]!=0) res[1][1]=a13;
  if (res[1]!=0) res[1][2]=a25;
  if (res[1]!=0) res[1][3]=a16;
  if (res[1]!=0) res[1][4]=a7;
  if (res[1]!=0) res[1][5]=a8;
  if (res[1]!=0) res[1][6]=a26;
  if (res[1]!=0) res[1][7]=a14;
  if (res[1]!=0) res[1][8]=a2;
  a48=(a71*a25);
  a78=(a88*a13);
  a48=(a48-a78);
  a78=(a91*a25);
  a76=(a72*a13);
  a78=(a78-a76);
  a48=(a48-a78);
  if (res[1]!=0) res[1][9]=a48;
  a48=(a88*a9);
  a78=(a0*a25);
  a48=(a48-a78);
  a78=(a72*a9);
  a25=(a79*a25);
  a78=(a78-a25);
  a48=(a48-a78);
  if (res[1]!=0) res[1][10]=a48;
  a48=(a0*a13);
  a78=(a71*a9);
  a48=(a48-a78);
  a13=(a79*a13);
  a9=(a91*a9);
  a13=(a13-a9);
  a48=(a48-a13);
  if (res[1]!=0) res[1][11]=a48;
  a48=(a71*a8);
  a13=(a88*a7);
  a48=(a48-a13);
  a13=(a91*a8);
  a9=(a72*a7);
  a13=(a13-a9);
  a48=(a48-a13);
  if (res[1]!=0) res[1][12]=a48;
  a48=(a88*a16);
  a13=(a0*a8);
  a48=(a48-a13);
  a13=(a72*a16);
  a8=(a79*a8);
  a13=(a13-a8);
  a48=(a48-a13);
  if (res[1]!=0) res[1][13]=a48;
  a48=(a0*a7);
  a13=(a71*a16);
  a48=(a48-a13);
  a7=(a79*a7);
  a16=(a91*a16);
  a7=(a7-a16);
  a48=(a48-a7);
  if (res[1]!=0) res[1][14]=a48;
  a48=(a71*a2);
  a7=(a88*a14);
  a48=(a48-a7);
  a7=(a91*a2);
  a16=(a72*a14);
  a7=(a7-a16);
  a48=(a48-a7);
  if (res[1]!=0) res[1][15]=a48;
  a88=(a88*a26);
  a48=(a0*a2);
  a88=(a88-a48);
  a48=(a72*a26);
  a2=(a79*a2);
  a48=(a48-a2);
  a88=(a88-a48);
  if (res[1]!=0) res[1][16]=a88;
  a0=(a0*a14);
  a71=(a71*a26);
  a0=(a0-a71);
  a14=(a79*a14);
  a26=(a91*a26);
  a14=(a14-a26);
  a0=(a0-a14);
  if (res[1]!=0) res[1][17]=a0;
  a0=(a15*a36);
  a14=(a5*a85);
  a0=(a0-a14);
  a14=(a91*a36);
  a26=(a72*a85);
  a14=(a14-a26);
  a0=(a0-a14);
  if (res[1]!=0) res[1][18]=a0;
  a5=(a5*a40);
  a0=(a10*a36);
  a5=(a5-a0);
  a0=(a72*a40);
  a36=(a79*a36);
  a0=(a0-a36);
  a5=(a5-a0);
  if (res[1]!=0) res[1][19]=a5;
  a10=(a10*a85);
  a15=(a15*a40);
  a10=(a10-a15);
  a85=(a79*a85);
  a40=(a91*a40);
  a85=(a85-a40);
  a10=(a10-a85);
  if (res[1]!=0) res[1][20]=a10;
  a10=(a82*a21);
  a85=(a23*a81);
  a10=(a10-a85);
  a85=(a91*a21);
  a40=(a72*a81);
  a85=(a85-a40);
  a10=(a10-a85);
  if (res[1]!=0) res[1][21]=a10;
  a10=(a23*a22);
  a85=(a24*a21);
  a10=(a10-a85);
  a85=(a72*a22);
  a21=(a79*a21);
  a85=(a85-a21);
  a10=(a10-a85);
  if (res[1]!=0) res[1][22]=a10;
  a10=(a24*a81);
  a85=(a82*a22);
  a10=(a10-a85);
  a81=(a79*a81);
  a22=(a91*a22);
  a81=(a81-a22);
  a10=(a10-a81);
  if (res[1]!=0) res[1][23]=a10;
  a10=(a82*a53);
  a81=(a23*a87);
  a10=(a10-a81);
  a81=(a91*a53);
  a22=(a72*a87);
  a81=(a81-a22);
  a10=(a10-a81);
  if (res[1]!=0) res[1][24]=a10;
  a23=(a23*a55);
  a10=(a24*a53);
  a23=(a23-a10);
  a10=(a72*a55);
  a81=(a79*a53);
  a10=(a10-a81);
  a23=(a23-a10);
  if (res[1]!=0) res[1][25]=a23;
  a24=(a24*a87);
  a82=(a82*a55);
  a24=(a24-a82);
  a82=(a79*a87);
  a23=(a91*a55);
  a82=(a82-a23);
  a24=(a24-a82);
  if (res[1]!=0) res[1][26]=a24;
  a24=(a86*a53);
  a82=(a50*a87);
  a24=(a24-a82);
  a82=(a91*a53);
  a23=(a72*a87);
  a82=(a82-a23);
  a24=(a24-a82);
  if (res[1]!=0) res[1][27]=a24;
  a50=(a50*a55);
  a24=(a51*a53);
  a50=(a50-a24);
  a24=(a72*a55);
  a82=(a79*a53);
  a24=(a24-a82);
  a50=(a50-a24);
  if (res[1]!=0) res[1][28]=a50;
  a51=(a51*a87);
  a86=(a86*a55);
  a51=(a51-a86);
  a86=(a79*a87);
  a50=(a91*a55);
  a86=(a86-a50);
  a51=(a51-a86);
  if (res[1]!=0) res[1][29]=a51;
  a51=(a83*a53);
  a86=(a20*a87);
  a51=(a51-a86);
  a86=(a91*a53);
  a50=(a72*a87);
  a86=(a86-a50);
  a51=(a51-a86);
  if (res[1]!=0) res[1][30]=a51;
  a20=(a20*a55);
  a51=(a43*a53);
  a20=(a20-a51);
  a51=(a72*a55);
  a86=(a79*a53);
  a51=(a51-a86);
  a20=(a20-a51);
  if (res[1]!=0) res[1][31]=a20;
  a43=(a43*a87);
  a83=(a83*a55);
  a43=(a43-a83);
  a83=(a79*a87);
  a20=(a91*a55);
  a83=(a83-a20);
  a43=(a43-a83);
  if (res[1]!=0) res[1][32]=a43;
  a43=(a89*a53);
  a83=(a34*a87);
  a43=(a43-a83);
  a83=(a91*a53);
  a20=(a72*a87);
  a83=(a83-a20);
  a43=(a43-a83);
  if (res[1]!=0) res[1][33]=a43;
  a34=(a34*a55);
  a43=(a67*a53);
  a34=(a34-a43);
  a43=(a72*a55);
  a83=(a79*a53);
  a43=(a43-a83);
  a34=(a34-a43);
  if (res[1]!=0) res[1][34]=a34;
  a67=(a67*a87);
  a89=(a89*a55);
  a67=(a67-a89);
  a89=(a79*a87);
  a34=(a91*a55);
  a89=(a89-a34);
  a67=(a67-a89);
  if (res[1]!=0) res[1][35]=a67;
  a67=0.;
  if (res[1]!=0) res[1][36]=a67;
  if (res[1]!=0) res[1][37]=a67;
  if (res[1]!=0) res[1][38]=a67;
  a89=(a90*a53);
  a34=(a66*a87);
  a89=(a89-a34);
  a34=(a91*a53);
  a43=(a72*a87);
  a34=(a34-a43);
  a89=(a89-a34);
  if (res[1]!=0) res[1][39]=a89;
  a66=(a66*a55);
  a89=(a73*a53);
  a66=(a66-a89);
  a72=(a72*a55);
  a89=(a79*a53);
  a72=(a72-a89);
  a66=(a66-a72);
  if (res[1]!=0) res[1][40]=a66;
  a73=(a73*a87);
  a90=(a90*a55);
  a73=(a73-a90);
  a79=(a79*a87);
  a91=(a91*a55);
  a79=(a79-a91);
  a73=(a73-a79);
  if (res[1]!=0) res[1][41]=a73;
  if (res[1]!=0) res[1][42]=a67;
  if (res[1]!=0) res[1][43]=a67;
  if (res[1]!=0) res[1][44]=a67;
  if (res[1]!=0) res[1][45]=a67;
  if (res[1]!=0) res[1][46]=a67;
  if (res[1]!=0) res[1][47]=a67;
  if (res[1]!=0) res[1][48]=a67;
  if (res[1]!=0) res[1][49]=a67;
  if (res[1]!=0) res[1][50]=a67;
  if (res[1]!=0) res[1][51]=a67;
  if (res[1]!=0) res[1][52]=a67;
  if (res[1]!=0) res[1][53]=a67;
  if (res[1]!=0) res[1][54]=a67;
  if (res[1]!=0) res[1][55]=a67;
  if (res[1]!=0) res[1][56]=a67;
  if (res[1]!=0) res[1][57]=a67;
  if (res[1]!=0) res[1][58]=a67;
  if (res[1]!=0) res[1][59]=a67;
  if (res[1]!=0) res[1][60]=a67;
  if (res[1]!=0) res[1][61]=a67;
  if (res[1]!=0) res[1][62]=a67;
  if (res[1]!=0) res[1][63]=a67;
  if (res[1]!=0) res[1][64]=a67;
  if (res[1]!=0) res[1][65]=a67;
  a67=arg[1]? arg[1][13] : 0;
  a73=cos(a75);
  a79=cos(a69);
  a91=cos(a63);
  a90=cos(a57);
  a66=cos(a42);
  a42=sin(a42);
  a72=(a38*a42);
  a72=(a66-a72);
  a89=cos(a30);
  a34=(a18*a89);
  a43=cos(a19);
  a83=(a18*a43);
  a20=arg[1]? arg[1][0] : 0;
  a51=arg[1]? arg[1][5] : 0;
  a86=(a11*a51);
  a86=(a20-a86);
  a50=(a83*a86);
  a19=sin(a19);
  a24=arg[1]? arg[1][1] : 0;
  a82=(a1*a51);
  a82=(a24+a82);
  a23=(a19*a82);
  a10=(a27*a43);
  a81=arg[1]? arg[1][2] : 0;
  a22=arg[1]? arg[1][4] : 0;
  a1=(a1*a22);
  a85=arg[1]? arg[1][3] : 0;
  a11=(a11*a85);
  a1=(a1-a11);
  a1=(a81-a1);
  a11=(a10*a1);
  a23=(a23+a11);
  a50=(a50+a23);
  a23=(a34*a50);
  a30=sin(a30);
  a82=(a43*a82);
  a11=(a27*a19);
  a21=(a11*a1);
  a82=(a82-a21);
  a21=(a18*a19);
  a40=(a21*a86);
  a82=(a82-a40);
  a40=arg[1]? arg[1][6] : 0;
  a15=(a39*a85);
  a5=(a18*a51);
  a15=(a15+a5);
  a15=(a40+a15);
  a5=(a17*a15);
  a5=(a82+a5);
  a0=(a30*a5);
  a36=(a39*a89);
  a86=(a39*a86);
  a1=(a18*a1);
  a86=(a86+a1);
  a1=(a43*a22);
  a14=(a11*a51);
  a1=(a1-a14);
  a14=(a21*a85);
  a1=(a1-a14);
  a14=(a17*a1);
  a86=(a86-a14);
  a14=(a36*a86);
  a0=(a0+a14);
  a23=(a23+a0);
  a0=(a72*a23);
  a14=(a38*a66);
  a26=(a49*a42);
  a14=(a14-a26);
  a5=(a89*a5);
  a26=(a39*a30);
  a71=(a26*a86);
  a5=(a5-a71);
  a71=(a18*a30);
  a88=(a71*a50);
  a5=(a5-a88);
  a88=(a14*a5);
  a48=(a52*a66);
  a48=(a42+a48);
  a2=(a27*a50);
  a86=(a18*a86);
  a2=(a2+a86);
  a86=(a48*a2);
  a88=(a88+a86);
  a0=(a0+a88);
  a88=(a38*a66);
  a88=(a88+a42);
  a86=(a83*a85);
  a7=(a19*a22);
  a16=(a10*a51);
  a7=(a7+a16);
  a86=(a86+a7);
  a7=(a34*a86);
  a16=(a30*a1);
  a13=(a36*a15);
  a16=(a16+a13);
  a7=(a7+a16);
  a16=(a88*a7);
  a49=(a49*a66);
  a38=(a38*a42);
  a49=(a49+a38);
  a38=(a89*a1);
  a13=(a26*a15);
  a38=(a38-a13);
  a13=(a71*a86);
  a38=(a38-a13);
  a13=(a49*a38);
  a42=(a52*a42);
  a42=(a42-a66);
  a66=arg[1]? arg[1][7] : 0;
  a8=(a27*a86);
  a15=(a18*a15);
  a8=(a8+a15);
  a8=(a66+a8);
  a15=(a42*a8);
  a13=(a13+a15);
  a16=(a16+a13);
  a13=(a54*a16);
  a15=arg[1]? arg[1][8] : 0;
  a9=(a52*a7);
  a9=(a9-a38);
  a9=(a15+a9);
  a78=(a29*a9);
  a13=(a13-a78);
  a13=(a0-a13);
  a78=(a90*a13);
  a57=sin(a57);
  a25=(a88*a23);
  a76=(a49*a5);
  a2=(a42*a2);
  a76=(a76+a2);
  a25=(a25+a76);
  a76=(a72*a7);
  a2=(a14*a38);
  a8=(a48*a8);
  a2=(a2+a8);
  a76=(a76+a2);
  a2=(a54*a76);
  a2=(a25+a2);
  a8=(a57*a2);
  a78=(a78-a8);
  a8=arg[1]? arg[1][9] : 0;
  a9=(a8+a9);
  a70=(a56*a9);
  a70=(a78+a70);
  a64=(a91*a70);
  a63=sin(a63);
  a2=(a90*a2);
  a13=(a57*a13);
  a2=(a2+a13);
  a13=(a62*a9);
  a13=(a2-a13);
  a58=(a63*a13);
  a64=(a64-a58);
  a58=arg[1]? arg[1][10] : 0;
  a9=(a58+a9);
  a45=(a46*a9);
  a45=(a64+a45);
  a61=(a79*a45);
  a69=sin(a69);
  a13=(a91*a13);
  a70=(a63*a70);
  a13=(a13+a70);
  a70=(a68*a9);
  a70=(a13-a70);
  a60=(a69*a70);
  a61=(a61-a60);
  a60=arg[1]? arg[1][11] : 0;
  a9=(a60+a9);
  a47=(a59*a9);
  a47=(a61+a47);
  a31=(a73*a47);
  a75=sin(a75);
  a70=(a79*a70);
  a45=(a69*a45);
  a70=(a70+a45);
  a9=(a74*a9);
  a9=(a70-a9);
  a45=(a75*a9);
  a31=(a31-a45);
  a31=(a67*a31);
  a61=(a60*a61);
  a64=(a58*a64);
  a78=(a8*a78);
  a0=(a15*a0);
  a5=(a66*a5);
  a82=(a40*a82);
  a45=(a24*a51);
  a37=(a81*a22);
  a45=(a45-a37);
  a37=(a22*a81);
  a41=(a51*a24);
  a37=(a37-a41);
  a45=(a45+a37);
  a83=(a83*a45);
  a37=(a81*a85);
  a41=(a20*a51);
  a37=(a37-a41);
  a51=(a51*a20);
  a81=(a85*a81);
  a51=(a51-a81);
  a37=(a37+a51);
  a19=(a19*a37);
  a51=(a20*a22);
  a81=(a24*a85);
  a51=(a51-a81);
  a85=(a85*a24);
  a22=(a22*a20);
  a85=(a85-a22);
  a51=(a51+a85);
  a10=(a10*a51);
  a19=(a19+a10);
  a83=(a83+a19);
  a82=(a82+a83);
  a83=(a34*a82);
  a43=(a43*a37);
  a11=(a11*a51);
  a43=(a43-a11);
  a21=(a21*a45);
  a43=(a43-a21);
  a50=(a40*a50);
  a43=(a43-a50);
  a50=(a30*a43);
  a39=(a39*a45);
  a51=(a18*a51);
  a39=(a39+a51);
  a86=(a40*a86);
  a17=(a17*a86);
  a39=(a39+a17);
  a36=(a36*a39);
  a50=(a50+a36);
  a83=(a83+a50);
  a5=(a5+a83);
  a83=(a88*a5);
  a43=(a89*a43);
  a26=(a26*a39);
  a43=(a43-a26);
  a26=(a71*a82);
  a43=(a43-a26);
  a23=(a66*a23);
  a43=(a43-a23);
  a23=(a49*a43);
  a82=(a27*a82);
  a18=(a18*a39);
  a82=(a82+a18);
  a18=(a42*a82);
  a23=(a23+a18);
  a83=(a83+a23);
  a0=(a0+a83);
  a38=(a66*a38);
  a40=(a40*a1);
  a34=(a34*a40);
  a30=(a30*a86);
  a34=(a34-a30);
  a38=(a38+a34);
  a34=(a72*a38);
  a27=(a27*a40);
  a30=(a48*a27);
  a66=(a66*a7);
  a71=(a71*a40);
  a89=(a89*a86);
  a71=(a71+a89);
  a66=(a66+a71);
  a71=(a14*a66);
  a30=(a30-a71);
  a34=(a34+a30);
  a30=(a15*a16);
  a34=(a34-a30);
  a30=(a54*a34);
  a0=(a0+a30);
  a30=(a90*a0);
  a72=(a72*a5);
  a14=(a14*a43);
  a48=(a48*a82);
  a14=(a14+a48);
  a72=(a72+a14);
  a25=(a15*a25);
  a72=(a72-a25);
  a15=(a15*a76);
  a88=(a88*a38);
  a42=(a42*a27);
  a49=(a49*a66);
  a42=(a42-a49);
  a88=(a88+a42);
  a15=(a15+a88);
  a54=(a54*a15);
  a38=(a52*a38);
  a38=(a38+a66);
  a66=(a29*a38);
  a54=(a54-a66);
  a72=(a72-a54);
  a54=(a57*a72);
  a30=(a30+a54);
  a78=(a78+a30);
  a30=(a62*a38);
  a78=(a78-a30);
  a30=(a91*a78);
  a72=(a90*a72);
  a0=(a57*a0);
  a72=(a72-a0);
  a2=(a8*a2);
  a72=(a72-a2);
  a2=(a56*a38);
  a72=(a72+a2);
  a2=(a63*a72);
  a30=(a30+a2);
  a64=(a64+a30);
  a30=(a68*a38);
  a64=(a64-a30);
  a30=(a79*a64);
  a72=(a91*a72);
  a78=(a63*a78);
  a72=(a72-a78);
  a13=(a58*a13);
  a72=(a72-a13);
  a13=(a46*a38);
  a72=(a72+a13);
  a13=(a69*a72);
  a30=(a30+a13);
  a61=(a61+a30);
  a30=(a74*a38);
  a61=(a61-a30);
  a30=(a73*a61);
  a72=(a79*a72);
  a64=(a69*a64);
  a72=(a72-a64);
  a70=(a60*a70);
  a72=(a72-a70);
  a70=(a59*a38);
  a72=(a72+a70);
  a70=(a75*a72);
  a30=(a30+a70);
  a31=(a31+a30);
  a30=(a80*a38);
  a31=(a31-a30);
  a77=(a77*a31);
  a72=(a73*a72);
  a61=(a75*a61);
  a72=(a72-a61);
  a9=(a73*a9);
  a47=(a75*a47);
  a9=(a9+a47);
  a9=(a67*a9);
  a72=(a72-a9);
  a38=(a65*a38);
  a72=(a72+a38);
  a33=(a33*a72);
  a52=(a52*a5);
  a52=(a52-a43);
  a29=(a29*a34);
  a52=(a52-a29);
  a29=(a90*a34);
  a43=(a57*a15);
  a29=(a29-a43);
  a43=(a90*a16);
  a5=(a57*a76);
  a43=(a43+a5);
  a5=(a8*a43);
  a29=(a29-a5);
  a56=(a56*a29);
  a76=(a90*a76);
  a16=(a57*a16);
  a76=(a76-a16);
  a8=(a8*a76);
  a90=(a90*a15);
  a57=(a57*a34);
  a90=(a90+a57);
  a8=(a8+a90);
  a62=(a62*a8);
  a56=(a56-a62);
  a52=(a52-a56);
  a56=(a91*a29);
  a62=(a63*a8);
  a56=(a56-a62);
  a62=(a91*a43);
  a90=(a63*a76);
  a62=(a62+a90);
  a90=(a58*a62);
  a56=(a56-a90);
  a46=(a46*a56);
  a76=(a91*a76);
  a43=(a63*a43);
  a76=(a76-a43);
  a58=(a58*a76);
  a91=(a91*a8);
  a63=(a63*a29);
  a91=(a91+a63);
  a58=(a58+a91);
  a68=(a68*a58);
  a46=(a46-a68);
  a52=(a52-a46);
  a46=(a79*a56);
  a68=(a69*a58);
  a46=(a46-a68);
  a68=(a79*a62);
  a91=(a69*a76);
  a68=(a68+a91);
  a91=(a60*a68);
  a46=(a46-a91);
  a59=(a59*a46);
  a76=(a79*a76);
  a62=(a69*a62);
  a76=(a76-a62);
  a60=(a60*a76);
  a79=(a79*a58);
  a69=(a69*a56);
  a79=(a79+a69);
  a60=(a60+a79);
  a74=(a74*a60);
  a59=(a59-a74);
  a52=(a52-a59);
  a59=(a73*a76);
  a74=(a75*a68);
  a59=(a59-a74);
  a59=(a67*a59);
  a74=(a73*a60);
  a79=(a75*a46);
  a74=(a74+a79);
  a59=(a59+a74);
  a80=(a80*a59);
  a46=(a73*a46);
  a60=(a75*a60);
  a46=(a46-a60);
  a73=(a73*a68);
  a75=(a75*a76);
  a73=(a73+a75);
  a67=(a67*a73);
  a46=(a46-a67);
  a65=(a65*a46);
  a80=(a80-a65);
  a52=(a52+a80);
  a55=(a55*a52);
  a33=(a33+a55);
  a77=(a77+a33);
  if (res[2]!=0) res[2][0]=a77;
  a84=(a84*a31);
  a4=(a4*a72);
  a87=(a87*a52);
  a4=(a4+a87);
  a84=(a84+a4);
  if (res[2]!=0) res[2][1]=a84;
  a44=(a44*a31);
  a32=(a32*a72);
  a53=(a53*a52);
  a32=(a32+a53);
  a44=(a44+a32);
  if (res[2]!=0) res[2][2]=a44;
  return 0;
}

CASADI_SYMBOL_EXPORT int cassie_LeftFootFront(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int cassie_LeftFootFront_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int cassie_LeftFootFront_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void cassie_LeftFootFront_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int cassie_LeftFootFront_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void cassie_LeftFootFront_release(int mem) {
}

CASADI_SYMBOL_EXPORT void cassie_LeftFootFront_incref(void) {
}

CASADI_SYMBOL_EXPORT void cassie_LeftFootFront_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int cassie_LeftFootFront_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int cassie_LeftFootFront_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real cassie_LeftFootFront_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* cassie_LeftFootFront_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* cassie_LeftFootFront_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* cassie_LeftFootFront_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* cassie_LeftFootFront_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    case 1: return casadi_s3;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int cassie_LeftFootFront_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
