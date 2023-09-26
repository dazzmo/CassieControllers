/* This file was automatically generated by CasADi 3.6.3+.
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
  #define CASADI_PREFIX(ID) cassie_foot_front_ ## ID
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

static const casadi_int casadi_s0[12] = {8, 1, 0, 8, 0, 1, 2, 3, 4, 5, 6, 7};
static const casadi_int casadi_s1[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s2[35] = {3, 8, 0, 3, 6, 9, 12, 15, 18, 21, 24, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2};

/* cassie_foot_front:(i0[8],i1[8])->(o0[3],o1[3x8],o2[3x8]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a11, a110, a111, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a8, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a9, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99;
  a0=-4.9000000000000002e-02;
  a1=8.9999999999999997e-02;
  a2=-4.4408920985006262e-16;
  a3=arg[0]? arg[0][0] : 0;
  a4=cos(a3);
  a5=(a2*a4);
  a6=(a1*a5);
  a6=(a0+a6);
  a7=1.2000000000000000e-01;
  a8=arg[0]? arg[0][1] : 0;
  a9=cos(a8);
  a10=(a2*a9);
  a11=(a5*a10);
  a3=sin(a3);
  a12=(a2*a3);
  a8=sin(a8);
  a13=(a12*a8);
  a13=(a9-a13);
  a11=(a11+a13);
  a13=4.8966386501092529e-12;
  a14=arg[0]? arg[0][2] : 0;
  a15=cos(a14);
  a16=(a13*a15);
  a14=sin(a14);
  a16=(a16+a14);
  a17=(a11*a16);
  a18=(a2-a5);
  a19=4.8965831389580217e-12;
  a20=(a19*a14);
  a20=(a20-a15);
  a21=(a18*a20);
  a22=(a2*a8);
  a5=(a5*a22);
  a12=(a12*a9);
  a12=(a12+a8);
  a5=(a5+a12);
  a12=5.5511151231257827e-17;
  a23=(a12*a15);
  a24=(a13*a14);
  a23=(a23+a24);
  a24=(a5*a23);
  a21=(a21-a24);
  a17=(a17+a21);
  a21=(a7*a17);
  a24=4.4999999999999997e-03;
  a25=(a19*a11);
  a25=(a25+a5);
  a26=(a24*a25);
  a21=(a21+a26);
  a21=(a6+a21);
  a26=6.0679999999999998e-02;
  a27=arg[0]? arg[0][3] : 0;
  a28=cos(a27);
  a29=(a17*a28);
  a30=(a13*a14);
  a30=(a15-a30);
  a31=(a11*a30);
  a32=(a19*a15);
  a32=(a14+a32);
  a33=(a18*a32);
  a13=(a13*a15);
  a12=(a12*a14);
  a13=(a13-a12);
  a12=(a5*a13);
  a33=(a33-a12);
  a31=(a31+a33);
  a27=sin(a27);
  a33=(a31*a27);
  a29=(a29+a33);
  a33=(a26*a29);
  a12=4.7410000000000001e-02;
  a14=(a31*a28);
  a15=(a17*a27);
  a14=(a14-a15);
  a15=(a12*a14);
  a33=(a33+a15);
  a33=(a21+a33);
  a15=4.3475999999999998e-01;
  a34=arg[0]? arg[0][4] : 0;
  a35=cos(a34);
  a36=(a29*a35);
  a34=sin(a34);
  a37=(a14*a34);
  a36=(a36+a37);
  a37=(a15*a36);
  a38=2.0000000000000000e-02;
  a39=(a14*a35);
  a40=(a29*a34);
  a39=(a39-a40);
  a40=(a38*a39);
  a37=(a37+a40);
  a37=(a33+a37);
  a40=4.0799999999999997e-01;
  a41=arg[0]? arg[0][5] : 0;
  a42=cos(a41);
  a43=(a36*a42);
  a41=sin(a41);
  a44=(a39*a41);
  a43=(a43+a44);
  a44=(a40*a43);
  a45=-4.0000000000000001e-02;
  a46=(a39*a42);
  a47=(a36*a41);
  a46=(a46-a47);
  a47=(a45*a46);
  a44=(a44+a47);
  a44=(a37+a44);
  a47=arg[0]? arg[0][7] : 0;
  a48=cos(a47);
  a49=(a43*a48);
  a47=sin(a47);
  a50=(a46*a47);
  a49=(a49+a50);
  a50=(a1*a49);
  a50=(a44+a50);
  if (res[0]!=0) res[0][0]=a50;
  a51=1.3500000000000001e-01;
  a52=(a1*a3);
  a52=(a51+a52);
  a53=(a3*a10);
  a54=(a4*a8);
  a53=(a53+a54);
  a54=(a53*a16);
  a55=(a4*a9);
  a56=(a3*a22);
  a55=(a55-a56);
  a56=(a55*a23);
  a57=(a3*a20);
  a56=(a56-a57);
  a54=(a54+a56);
  a56=(a7*a54);
  a57=(a19*a53);
  a57=(a57-a55);
  a58=(a24*a57);
  a56=(a56+a58);
  a56=(a52+a56);
  a58=(a54*a28);
  a59=(a53*a30);
  a60=(a55*a13);
  a61=(a3*a32);
  a60=(a60-a61);
  a59=(a59+a60);
  a60=(a59*a27);
  a58=(a58+a60);
  a60=(a26*a58);
  a61=(a59*a28);
  a62=(a54*a27);
  a61=(a61-a62);
  a62=(a12*a61);
  a60=(a60+a62);
  a60=(a56+a60);
  a62=(a58*a35);
  a63=(a61*a34);
  a62=(a62+a63);
  a63=(a15*a62);
  a64=(a61*a35);
  a65=(a58*a34);
  a64=(a64-a65);
  a65=(a38*a64);
  a63=(a63+a65);
  a63=(a60+a63);
  a65=(a62*a42);
  a66=(a64*a41);
  a65=(a65+a66);
  a66=(a40*a65);
  a67=(a64*a42);
  a68=(a62*a41);
  a67=(a67-a68);
  a68=(a45*a67);
  a66=(a66+a68);
  a66=(a63+a66);
  a68=(a65*a48);
  a69=(a67*a47);
  a68=(a68+a69);
  a69=(a1*a68);
  a69=(a66+a69);
  if (res[0]!=0) res[0][1]=a69;
  a70=(a3*a8);
  a71=(a2*a9);
  a70=(a70+a71);
  a10=(a4*a10);
  a70=(a70-a10);
  a10=(a70*a16);
  a22=(a4*a22);
  a71=(a3*a9);
  a72=(a2*a8);
  a71=(a71-a72);
  a22=(a22+a71);
  a71=(a22*a23);
  a72=1.9721522630525295e-31;
  a72=(a4+a72);
  a73=(a72*a20);
  a71=(a71+a73);
  a10=(a10+a71);
  a71=(a7*a10);
  a73=(a19*a70);
  a73=(a73-a22);
  a74=(a24*a73);
  a71=(a71+a74);
  a4=(a1*a4);
  a71=(a71-a4);
  a74=(a10*a28);
  a75=(a70*a30);
  a76=(a22*a13);
  a77=(a72*a32);
  a76=(a76+a77);
  a75=(a75+a76);
  a76=(a75*a27);
  a74=(a74+a76);
  a76=(a26*a74);
  a77=(a75*a28);
  a78=(a10*a27);
  a77=(a77-a78);
  a78=(a12*a77);
  a76=(a76+a78);
  a76=(a71+a76);
  a78=(a74*a35);
  a79=(a77*a34);
  a78=(a78+a79);
  a79=(a15*a78);
  a80=(a77*a35);
  a81=(a74*a34);
  a80=(a80-a81);
  a81=(a38*a80);
  a79=(a79+a81);
  a79=(a76+a79);
  a81=(a78*a42);
  a82=(a80*a41);
  a81=(a81+a82);
  a82=(a40*a81);
  a83=(a80*a42);
  a84=(a78*a41);
  a83=(a83-a84);
  a84=(a45*a83);
  a82=(a82+a84);
  a82=(a79+a82);
  a84=(a81*a48);
  a85=(a83*a47);
  a84=(a84+a85);
  a85=(a1*a84);
  a85=(a82+a85);
  if (res[0]!=0) res[0][2]=a85;
  a86=-5.9952043329758457e-17;
  a87=(a1*a68);
  a87=(a66+a87);
  a88=(a2*a87);
  a88=(a86-a88);
  if (res[1]!=0) res[1][0]=a88;
  a88=-2.1760371282653069e-17;
  a89=(a1*a84);
  a89=(a82+a89);
  a90=(a1*a49);
  a90=(a44+a90);
  a91=(a2*a90);
  a91=(a89-a91);
  a91=(a88-a91);
  if (res[1]!=0) res[1][1]=a91;
  a91=-1.3500000000000001e-01;
  a92=(a91+a87);
  if (res[1]!=0) res[1][2]=a92;
  a92=(a52*a72);
  a93=(a4*a3);
  a92=(a92-a93);
  a93=(a87*a72);
  a94=(a89*a3);
  a93=(a93+a94);
  a93=(a92-a93);
  if (res[1]!=0) res[1][3]=a93;
  a93=(a4*a18);
  a94=(a6*a72);
  a93=(a93+a94);
  a94=(a89*a18);
  a95=(a90*a72);
  a94=(a94-a95);
  a94=(a93+a94);
  a94=(-a94);
  if (res[1]!=0) res[1][4]=a94;
  a94=(a90*a3);
  a95=(a87*a18);
  a94=(a94+a95);
  a95=(a6*a3);
  a96=(a52*a18);
  a95=(a95+a96);
  a94=(a94-a95);
  if (res[1]!=0) res[1][5]=a94;
  a94=(a52*a73);
  a96=(a4*a57);
  a94=(a94+a96);
  a96=(a87*a73);
  a97=(a89*a57);
  a96=(a96-a97);
  a96=(a94-a96);
  if (res[1]!=0) res[1][6]=a96;
  a96=(a4*a25);
  a97=(a6*a73);
  a96=(a96+a97);
  a97=(a89*a25);
  a98=(a90*a73);
  a97=(a97-a98);
  a97=(a96+a97);
  a97=(-a97);
  if (res[1]!=0) res[1][7]=a97;
  a97=(a6*a57);
  a98=(a52*a25);
  a97=(a97-a98);
  a98=(a90*a57);
  a99=(a87*a25);
  a98=(a98-a99);
  a98=(a97-a98);
  if (res[1]!=0) res[1][8]=a98;
  a98=(a56*a73);
  a99=(a71*a57);
  a98=(a98-a99);
  a99=(a87*a73);
  a100=(a89*a57);
  a99=(a99-a100);
  a99=(a98-a99);
  if (res[1]!=0) res[1][9]=a99;
  a99=(a71*a25);
  a100=(a21*a73);
  a99=(a99-a100);
  a100=(a89*a25);
  a101=(a90*a73);
  a100=(a100-a101);
  a100=(a99-a100);
  if (res[1]!=0) res[1][10]=a100;
  a100=(a21*a57);
  a101=(a56*a25);
  a100=(a100-a101);
  a101=(a90*a57);
  a102=(a87*a25);
  a101=(a101-a102);
  a101=(a100-a101);
  if (res[1]!=0) res[1][11]=a101;
  a101=(a60*a73);
  a102=(a76*a57);
  a101=(a101-a102);
  a102=(a87*a73);
  a103=(a89*a57);
  a102=(a102-a103);
  a102=(a101-a102);
  if (res[1]!=0) res[1][12]=a102;
  a102=(a76*a25);
  a103=(a33*a73);
  a102=(a102-a103);
  a103=(a89*a25);
  a104=(a90*a73);
  a103=(a103-a104);
  a103=(a102-a103);
  if (res[1]!=0) res[1][13]=a103;
  a103=(a33*a57);
  a104=(a60*a25);
  a103=(a103-a104);
  a104=(a90*a57);
  a105=(a87*a25);
  a104=(a104-a105);
  a104=(a103-a104);
  if (res[1]!=0) res[1][14]=a104;
  a104=(a63*a73);
  a105=(a79*a57);
  a104=(a104-a105);
  a105=(a87*a73);
  a106=(a89*a57);
  a105=(a105-a106);
  a105=(a104-a105);
  if (res[1]!=0) res[1][15]=a105;
  a105=(a79*a25);
  a106=(a37*a73);
  a105=(a105-a106);
  a106=(a89*a25);
  a107=(a90*a73);
  a106=(a106-a107);
  a106=(a105-a106);
  if (res[1]!=0) res[1][16]=a106;
  a106=(a37*a57);
  a107=(a63*a25);
  a106=(a106-a107);
  a107=(a90*a57);
  a108=(a87*a25);
  a107=(a107-a108);
  a107=(a106-a107);
  if (res[1]!=0) res[1][17]=a107;
  a107=0.;
  if (res[1]!=0) res[1][18]=a107;
  if (res[1]!=0) res[1][19]=a107;
  if (res[1]!=0) res[1][20]=a107;
  a108=(a66*a73);
  a109=(a82*a57);
  a108=(a108-a109);
  a109=(a87*a73);
  a110=(a89*a57);
  a109=(a109-a110);
  a109=(a108-a109);
  if (res[1]!=0) res[1][21]=a109;
  a109=(a82*a25);
  a110=(a44*a73);
  a109=(a109-a110);
  a89=(a89*a25);
  a110=(a90*a73);
  a89=(a89-a110);
  a89=(a109-a89);
  if (res[1]!=0) res[1][22]=a89;
  a89=(a44*a57);
  a110=(a66*a25);
  a89=(a89-a110);
  a90=(a90*a57);
  a87=(a87*a25);
  a90=(a90-a87);
  a90=(a89-a90);
  if (res[1]!=0) res[1][23]=a90;
  a90=arg[1]? arg[1][0] : 0;
  a87=(a2*a90);
  a0=(a0*a87);
  a110=(a2*a0);
  a111=(a88*a87);
  a110=(a110+a111);
  a110=(-a110);
  if (res[2]!=0) res[2][0]=a110;
  a86=(a86*a87);
  a91=(a91*a90);
  a86=(a86-a91);
  a91=(a51*a90);
  a51=(a51*a87);
  a51=(a2*a51);
  a91=(a91+a51);
  a86=(a86-a91);
  if (res[2]!=0) res[2][1]=a86;
  a88=(a88*a90);
  a0=(a0+a88);
  if (res[2]!=0) res[2][2]=a0;
  a1=(a1*a90);
  a0=(a8*a1);
  a88=(a53*a0);
  a1=(a9*a1);
  a86=(a55*a1);
  a88=(a88+a86);
  a9=(a9*a90);
  a86=(a11*a9);
  a8=(a8*a90);
  a91=(a5*a8);
  a51=arg[1]? arg[1][1] : 0;
  a2=(a2*a90);
  a51=(a51+a2);
  a2=(a18*a51);
  a91=(a91+a2);
  a86=(a86+a91);
  a91=(a4*a86);
  a2=(a70*a9);
  a90=(a72*a51);
  a87=(a22*a8);
  a90=(a90-a87);
  a2=(a2+a90);
  a90=(a6*a2);
  a91=(a91+a90);
  a88=(a88-a91);
  a91=(a88*a72);
  a70=(a70*a0);
  a22=(a22*a1);
  a70=(a70+a22);
  a53=(a53*a9);
  a55=(a55*a8);
  a22=(a3*a51);
  a55=(a55+a22);
  a53=(a53-a55);
  a55=(a6*a53);
  a22=(a52*a86);
  a55=(a55-a22);
  a70=(a70+a55);
  a55=(a70*a3);
  a91=(a91+a55);
  a55=(a2*a93);
  a22=(a53*a95);
  a55=(a55-a22);
  a91=(a91+a55);
  a55=(a86*a3);
  a22=(a53*a18);
  a55=(a55+a22);
  a22=(a69*a55);
  a90=(a2*a18);
  a87=(a86*a72);
  a90=(a90-a87);
  a87=(a85*a90);
  a22=(a22+a87);
  a91=(a91+a22);
  if (res[2]!=0) res[2][3]=a91;
  a70=(a70*a18);
  a11=(a11*a0);
  a5=(a5*a1);
  a11=(a11-a5);
  a5=(a52*a2);
  a91=(a4*a53);
  a5=(a5+a91);
  a11=(a11+a5);
  a5=(a11*a72);
  a70=(a70-a5);
  a5=(a2*a92);
  a95=(a86*a95);
  a5=(a5+a95);
  a70=(a70+a5);
  a72=(a53*a72);
  a2=(a2*a3);
  a72=(a72+a2);
  a2=(a85*a72);
  a55=(a50*a55);
  a2=(a2+a55);
  a70=(a70-a2);
  if (res[2]!=0) res[2][4]=a70;
  a11=(a11*a3);
  a88=(a88*a18);
  a11=(a11+a88);
  a86=(a86*a93);
  a53=(a53*a92);
  a86=(a86+a53);
  a11=(a11+a86);
  a90=(a50*a90);
  a72=(a69*a72);
  a90=(a90-a72);
  a11=(a11+a90);
  a11=(-a11);
  if (res[2]!=0) res[2][5]=a11;
  a11=(a16*a0);
  a90=(a23*a1);
  a11=(a11+a90);
  a90=(a54*a11);
  a72=(a30*a0);
  a86=(a13*a1);
  a72=(a72+a86);
  a86=(a59*a72);
  a0=(a19*a0);
  a0=(a0-a1);
  a1=(a57*a0);
  a86=(a86+a1);
  a90=(a90+a86);
  a16=(a16*a9);
  a20=(a20*a51);
  a23=(a23*a8);
  a20=(a20-a23);
  a16=(a16+a20);
  a20=(a17*a16);
  a30=(a30*a9);
  a32=(a32*a51);
  a13=(a13*a8);
  a32=(a32-a13);
  a30=(a30+a32);
  a32=(a31*a30);
  a13=arg[1]? arg[1][2] : 0;
  a19=(a19*a9);
  a19=(a19+a8);
  a13=(a13+a19);
  a19=(a25*a13);
  a32=(a32+a19);
  a20=(a20+a32);
  a32=(a4*a20);
  a19=(a10*a16);
  a8=(a75*a30);
  a9=(a73*a13);
  a8=(a8+a9);
  a19=(a19+a8);
  a8=(a6*a19);
  a32=(a32+a8);
  a90=(a90-a32);
  a32=(a90*a73);
  a10=(a10*a11);
  a75=(a75*a72);
  a8=(a73*a0);
  a75=(a75+a8);
  a10=(a10+a75);
  a54=(a54*a16);
  a59=(a59*a30);
  a75=(a57*a13);
  a59=(a59+a75);
  a54=(a54+a59);
  a6=(a6*a54);
  a59=(a52*a20);
  a6=(a6-a59);
  a10=(a10+a6);
  a6=(a10*a57);
  a32=(a32-a6);
  a6=(a54*a97);
  a59=(a19*a96);
  a6=(a6+a59);
  a32=(a32+a6);
  a6=(a20*a57);
  a59=(a54*a25);
  a6=(a6-a59);
  a59=(a69*a6);
  a75=(a19*a25);
  a8=(a20*a73);
  a75=(a75-a8);
  a8=(a85*a75);
  a59=(a59-a8);
  a32=(a32-a59);
  if (res[2]!=0) res[2][6]=a32;
  a10=(a10*a25);
  a17=(a17*a11);
  a31=(a31*a72);
  a32=(a25*a0);
  a31=(a31+a32);
  a17=(a17+a31);
  a52=(a52*a19);
  a4=(a4*a54);
  a52=(a52+a4);
  a17=(a17+a52);
  a52=(a17*a73);
  a10=(a10-a52);
  a52=(a19*a94);
  a97=(a20*a97);
  a52=(a52-a97);
  a10=(a10+a52);
  a52=(a54*a73);
  a19=(a19*a57);
  a52=(a52-a19);
  a19=(a85*a52);
  a6=(a50*a6);
  a19=(a19-a6);
  a10=(a10-a19);
  if (res[2]!=0) res[2][7]=a10;
  a17=(a17*a57);
  a90=(a90*a25);
  a17=(a17-a90);
  a20=(a20*a96);
  a54=(a54*a94);
  a20=(a20+a54);
  a17=(a17-a20);
  a75=(a50*a75);
  a52=(a69*a52);
  a75=(a75-a52);
  a17=(a17-a75);
  if (res[2]!=0) res[2][8]=a17;
  a17=(a24*a30);
  a11=(a11+a17);
  a17=(a28*a11);
  a24=(a24*a16);
  a75=(a7*a13);
  a24=(a24-a75);
  a72=(a72-a24);
  a24=(a27*a72);
  a17=(a17+a24);
  a24=(a58*a17);
  a72=(a28*a72);
  a11=(a27*a11);
  a72=(a72-a11);
  a11=(a61*a72);
  a7=(a7*a30);
  a0=(a0-a7);
  a7=(a57*a0);
  a11=(a11+a7);
  a24=(a24+a11);
  a11=(a28*a16);
  a7=(a27*a30);
  a11=(a11+a7);
  a7=(a29*a11);
  a28=(a28*a30);
  a27=(a27*a16);
  a28=(a28-a27);
  a27=(a14*a28);
  a16=arg[1]? arg[1][3] : 0;
  a16=(a16+a13);
  a13=(a25*a16);
  a27=(a27+a13);
  a7=(a7+a27);
  a27=(a71*a7);
  a13=(a74*a11);
  a30=(a77*a28);
  a75=(a73*a16);
  a30=(a30+a75);
  a13=(a13+a30);
  a30=(a21*a13);
  a27=(a27-a30);
  a24=(a24+a27);
  a27=(a24*a73);
  a74=(a74*a17);
  a77=(a77*a72);
  a30=(a73*a0);
  a77=(a77+a30);
  a74=(a74+a77);
  a58=(a58*a11);
  a61=(a61*a28);
  a77=(a57*a16);
  a61=(a61+a77);
  a58=(a58+a61);
  a21=(a21*a58);
  a61=(a56*a7);
  a21=(a21-a61);
  a74=(a74+a21);
  a21=(a74*a57);
  a27=(a27-a21);
  a21=(a58*a100);
  a61=(a13*a99);
  a21=(a21-a61);
  a27=(a27+a21);
  a21=(a7*a57);
  a61=(a58*a25);
  a21=(a21-a61);
  a61=(a69*a21);
  a77=(a13*a25);
  a30=(a7*a73);
  a77=(a77-a30);
  a30=(a85*a77);
  a61=(a61-a30);
  a27=(a27-a61);
  if (res[2]!=0) res[2][9]=a27;
  a74=(a74*a25);
  a29=(a29*a17);
  a14=(a14*a72);
  a27=(a25*a0);
  a14=(a14+a27);
  a29=(a29+a14);
  a56=(a56*a13);
  a71=(a71*a58);
  a56=(a56-a71);
  a29=(a29+a56);
  a56=(a29*a73);
  a74=(a74-a56);
  a56=(a13*a98);
  a100=(a7*a100);
  a56=(a56-a100);
  a74=(a74+a56);
  a56=(a58*a73);
  a13=(a13*a57);
  a56=(a56-a13);
  a13=(a85*a56);
  a21=(a50*a21);
  a13=(a13-a21);
  a74=(a74-a13);
  if (res[2]!=0) res[2][10]=a74;
  a29=(a29*a57);
  a24=(a24*a25);
  a29=(a29-a24);
  a7=(a7*a99);
  a58=(a58*a98);
  a7=(a7-a58);
  a29=(a29+a7);
  a77=(a50*a77);
  a56=(a69*a56);
  a77=(a77-a56);
  a29=(a29-a77);
  if (res[2]!=0) res[2][11]=a29;
  a29=(a12*a16);
  a17=(a17-a29);
  a29=(a35*a17);
  a77=(a26*a16);
  a72=(a72+a77);
  a77=(a34*a72);
  a29=(a29+a77);
  a77=(a62*a29);
  a72=(a35*a72);
  a17=(a34*a17);
  a72=(a72-a17);
  a17=(a64*a72);
  a26=(a26*a28);
  a12=(a12*a11);
  a26=(a26-a12);
  a0=(a0-a26);
  a26=(a57*a0);
  a17=(a17+a26);
  a77=(a77+a17);
  a17=(a35*a11);
  a26=(a34*a28);
  a17=(a17+a26);
  a26=(a36*a17);
  a35=(a35*a28);
  a34=(a34*a11);
  a35=(a35-a34);
  a34=(a39*a35);
  a11=arg[1]? arg[1][4] : 0;
  a11=(a11+a16);
  a16=(a25*a11);
  a34=(a34+a16);
  a26=(a26+a34);
  a34=(a76*a26);
  a16=(a78*a17);
  a28=(a80*a35);
  a12=(a73*a11);
  a28=(a28+a12);
  a16=(a16+a28);
  a28=(a33*a16);
  a34=(a34-a28);
  a77=(a77+a34);
  a34=(a77*a73);
  a78=(a78*a29);
  a80=(a80*a72);
  a28=(a73*a0);
  a80=(a80+a28);
  a78=(a78+a80);
  a62=(a62*a17);
  a64=(a64*a35);
  a80=(a57*a11);
  a64=(a64+a80);
  a62=(a62+a64);
  a33=(a33*a62);
  a64=(a60*a26);
  a33=(a33-a64);
  a78=(a78+a33);
  a33=(a78*a57);
  a34=(a34-a33);
  a33=(a62*a103);
  a64=(a16*a102);
  a33=(a33-a64);
  a34=(a34+a33);
  a33=(a26*a57);
  a64=(a62*a25);
  a33=(a33-a64);
  a64=(a69*a33);
  a80=(a16*a25);
  a28=(a26*a73);
  a80=(a80-a28);
  a28=(a85*a80);
  a64=(a64-a28);
  a34=(a34-a64);
  if (res[2]!=0) res[2][12]=a34;
  a78=(a78*a25);
  a36=(a36*a29);
  a39=(a39*a72);
  a34=(a25*a0);
  a39=(a39+a34);
  a36=(a36+a39);
  a60=(a60*a16);
  a76=(a76*a62);
  a60=(a60-a76);
  a36=(a36+a60);
  a60=(a36*a73);
  a78=(a78-a60);
  a60=(a16*a101);
  a103=(a26*a103);
  a60=(a60-a103);
  a78=(a78+a60);
  a60=(a62*a73);
  a16=(a16*a57);
  a60=(a60-a16);
  a16=(a85*a60);
  a33=(a50*a33);
  a16=(a16-a33);
  a78=(a78-a16);
  if (res[2]!=0) res[2][13]=a78;
  a36=(a36*a57);
  a77=(a77*a25);
  a36=(a36-a77);
  a26=(a26*a102);
  a62=(a62*a101);
  a26=(a26-a62);
  a36=(a36+a26);
  a80=(a50*a80);
  a60=(a69*a60);
  a80=(a80-a60);
  a36=(a36-a80);
  if (res[2]!=0) res[2][14]=a36;
  a36=(a38*a11);
  a29=(a29-a36);
  a36=(a42*a29);
  a80=(a15*a11);
  a72=(a72+a80);
  a80=(a41*a72);
  a36=(a36+a80);
  a80=(a65*a36);
  a72=(a42*a72);
  a29=(a41*a29);
  a72=(a72-a29);
  a29=(a67*a72);
  a15=(a15*a35);
  a38=(a38*a17);
  a15=(a15-a38);
  a0=(a0-a15);
  a15=(a57*a0);
  a29=(a29+a15);
  a80=(a80+a29);
  a29=(a42*a17);
  a15=(a41*a35);
  a29=(a29+a15);
  a15=(a43*a29);
  a42=(a42*a35);
  a41=(a41*a17);
  a42=(a42-a41);
  a41=(a46*a42);
  a17=arg[1]? arg[1][5] : 0;
  a17=(a17+a11);
  a11=(a25*a17);
  a41=(a41+a11);
  a15=(a15+a41);
  a41=(a79*a15);
  a11=(a81*a29);
  a35=(a83*a42);
  a38=(a73*a17);
  a35=(a35+a38);
  a11=(a11+a35);
  a35=(a37*a11);
  a41=(a41-a35);
  a80=(a80+a41);
  a41=(a80*a73);
  a35=(a81*a36);
  a38=(a83*a72);
  a60=(a73*a0);
  a38=(a38+a60);
  a35=(a35+a38);
  a38=(a65*a29);
  a60=(a67*a42);
  a26=(a57*a17);
  a60=(a60+a26);
  a38=(a38+a60);
  a37=(a37*a38);
  a60=(a63*a15);
  a37=(a37-a60);
  a35=(a35+a37);
  a37=(a35*a57);
  a41=(a41-a37);
  a37=(a38*a106);
  a60=(a11*a105);
  a37=(a37-a60);
  a41=(a41+a37);
  a37=(a15*a57);
  a60=(a38*a25);
  a37=(a37-a60);
  a60=(a69*a37);
  a26=(a11*a25);
  a62=(a15*a73);
  a26=(a26-a62);
  a62=(a85*a26);
  a60=(a60-a62);
  a41=(a41-a60);
  if (res[2]!=0) res[2][15]=a41;
  a35=(a35*a25);
  a41=(a43*a36);
  a60=(a46*a72);
  a62=(a25*a0);
  a60=(a60+a62);
  a41=(a41+a60);
  a63=(a63*a11);
  a79=(a79*a38);
  a63=(a63-a79);
  a41=(a41+a63);
  a63=(a41*a73);
  a35=(a35-a63);
  a63=(a11*a104);
  a106=(a15*a106);
  a63=(a63-a106);
  a35=(a35+a63);
  a63=(a38*a73);
  a11=(a11*a57);
  a63=(a63-a11);
  a11=(a85*a63);
  a37=(a50*a37);
  a11=(a11-a37);
  a35=(a35-a11);
  if (res[2]!=0) res[2][16]=a35;
  a41=(a41*a57);
  a80=(a80*a25);
  a41=(a41-a80);
  a15=(a15*a105);
  a38=(a38*a104);
  a15=(a15-a38);
  a41=(a41+a15);
  a26=(a50*a26);
  a63=(a69*a63);
  a26=(a26-a63);
  a41=(a41-a26);
  if (res[2]!=0) res[2][17]=a41;
  if (res[2]!=0) res[2][18]=a107;
  if (res[2]!=0) res[2][19]=a107;
  if (res[2]!=0) res[2][20]=a107;
  a107=(a45*a17);
  a36=(a36-a107);
  a107=(a48*a36);
  a41=(a40*a17);
  a72=(a72+a41);
  a41=(a47*a72);
  a107=(a107+a41);
  a41=(a68*a107);
  a67=(a67*a48);
  a65=(a65*a47);
  a67=(a67-a65);
  a72=(a48*a72);
  a36=(a47*a36);
  a72=(a72-a36);
  a36=(a67*a72);
  a40=(a40*a42);
  a45=(a45*a29);
  a40=(a40-a45);
  a0=(a0-a40);
  a40=(a57*a0);
  a36=(a36+a40);
  a41=(a41+a36);
  a36=(a48*a29);
  a40=(a47*a42);
  a36=(a36+a40);
  a40=(a49*a36);
  a46=(a46*a48);
  a43=(a43*a47);
  a46=(a46-a43);
  a42=(a48*a42);
  a29=(a47*a29);
  a42=(a42-a29);
  a29=(a46*a42);
  a43=arg[1]? arg[1][7] : 0;
  a43=(a43+a17);
  a17=(a25*a43);
  a29=(a29+a17);
  a40=(a40+a29);
  a29=(a82*a40);
  a17=(a84*a36);
  a83=(a83*a48);
  a81=(a81*a47);
  a83=(a83-a81);
  a81=(a83*a42);
  a47=(a73*a43);
  a81=(a81+a47);
  a17=(a17+a81);
  a81=(a44*a17);
  a29=(a29-a81);
  a41=(a41+a29);
  a29=(a41*a73);
  a84=(a84*a107);
  a83=(a83*a72);
  a81=(a73*a0);
  a83=(a83+a81);
  a84=(a84+a83);
  a68=(a68*a36);
  a67=(a67*a42);
  a43=(a57*a43);
  a67=(a67+a43);
  a68=(a68+a67);
  a44=(a44*a68);
  a67=(a66*a40);
  a44=(a44-a67);
  a84=(a84+a44);
  a44=(a84*a57);
  a29=(a29-a44);
  a44=(a68*a89);
  a67=(a17*a109);
  a44=(a44-a67);
  a29=(a29+a44);
  a44=(a40*a57);
  a67=(a68*a25);
  a44=(a44-a67);
  a67=(a69*a44);
  a43=(a17*a25);
  a42=(a40*a73);
  a43=(a43-a42);
  a42=(a85*a43);
  a67=(a67-a42);
  a29=(a29-a67);
  if (res[2]!=0) res[2][21]=a29;
  a84=(a84*a25);
  a49=(a49*a107);
  a46=(a46*a72);
  a0=(a25*a0);
  a46=(a46+a0);
  a49=(a49+a46);
  a66=(a66*a17);
  a82=(a82*a68);
  a66=(a66-a82);
  a49=(a49+a66);
  a66=(a49*a73);
  a84=(a84-a66);
  a66=(a17*a108);
  a89=(a40*a89);
  a66=(a66-a89);
  a84=(a84+a66);
  a73=(a68*a73);
  a17=(a17*a57);
  a73=(a73-a17);
  a85=(a85*a73);
  a44=(a50*a44);
  a85=(a85-a44);
  a84=(a84-a85);
  if (res[2]!=0) res[2][22]=a84;
  a49=(a49*a57);
  a41=(a41*a25);
  a49=(a49-a41);
  a40=(a40*a109);
  a68=(a68*a108);
  a40=(a40-a68);
  a49=(a49+a40);
  a50=(a50*a43);
  a69=(a69*a73);
  a50=(a50-a69);
  a49=(a49-a50);
  if (res[2]!=0) res[2][23]=a49;
  return 0;
}

CASADI_SYMBOL_EXPORT int cassie_foot_front(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int cassie_foot_front_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int cassie_foot_front_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void cassie_foot_front_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int cassie_foot_front_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void cassie_foot_front_release(int mem) {
}

CASADI_SYMBOL_EXPORT void cassie_foot_front_incref(void) {
}

CASADI_SYMBOL_EXPORT void cassie_foot_front_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int cassie_foot_front_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int cassie_foot_front_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real cassie_foot_front_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* cassie_foot_front_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* cassie_foot_front_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* cassie_foot_front_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* cassie_foot_front_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    case 1: return casadi_s2;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int cassie_foot_front_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
