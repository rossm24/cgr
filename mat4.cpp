#include "mat4.h"
#include <algorithm>

static inline Mat4 Mzero(){ Mat4 M{}; return M; }

Mat4 Mat4::identity(){
    Mat4 I{}; I.m[0]=I.m[5]=I.m[10]=I.m[15]=1.0; return I;
}
Mat4 Mat4::translate(const Vec3& t){
    Mat4 A = identity(); A.m[12]=t.x; A.m[13]=t.y; A.m[14]=t.z; return A;
}
Mat4 Mat4::scale(const Vec3& s){
    Mat4 A{}; A.m[0]=s.x; A.m[5]=s.y; A.m[10]=s.z; A.m[15]=1.0; return A;
}
Mat4 Mat4::rotateX(double r){
    Mat4 A = identity();
    double c=std::cos(r), s=std::sin(r);
    A.m[5]=c; A.m[6]=s; A.m[9]=-s; A.m[10]=c; return A;
}
Mat4 Mat4::rotateY(double r){
    Mat4 A = identity();
    double c=std::cos(r), s=std::sin(r);
    A.m[0]=c; A.m[2]=-s; A.m[8]=s; A.m[10]=c; return A;
}
Mat4 Mat4::rotateZ(double r){
    Mat4 A = identity();
    double c=std::cos(r), s=std::sin(r);
    A.m[0]=c; A.m[1]=s; A.m[4]=-s; A.m[5]=c; return A;
}
Mat4 Mat4::multiply(const Mat4& A, const Mat4& B){
    Mat4 C = Mzero();
    for(int r=0;r<4;++r)
        for(int c=0;c<4;++c){
            double v=0;
            for(int k=0;k<4;++k) v += A.m[r*4+k]*B.m[k*4+c];
            C.m[r*4+c]=v;
        }
    return C;
}
Mat4 Mat4::transpose(const Mat4& A){
    Mat4 T = Mzero();
    for(int r=0;r<4;++r) for(int c=0;c<4;++c) T.m[c*4+r]=A.m[r*4+c];
    return T;
}

// Gauss-Jordan inverse
bool Mat4::invert(const Mat4& A, Mat4& out){
    double a[4][8]{};
    for(int r=0;r<4;++r){
        for(int c=0;c<4;++c) a[r][c]=A.m[r*4+c];
        for(int c=0;c<4;++c) a[r][4+c]=(r==c)?1.0:0.0;
    }
    for(int c=0;c<4;++c){
        int pr=c; double best=std::abs(a[c][c]);
        for(int r=c+1;r<4;++r){ double v=std::abs(a[r][c]); if(v>best){best=v; pr=r;} }
        if(best<1e-15) return false;
        if(pr!=c) for(int k=0;k<8;++k) std::swap(a[pr][k], a[c][k]);
        double piv=a[c][c];
        for(int k=0;k<8;++k) a[c][k]/=piv;
        for(int r=0;r<4;++r) if(r!=c){
            double f=a[r][c];
            for(int k=0;k<8;++k) a[r][k]-=f*a[c][k];
        }
    }
    for(int r=0;r<4;++r) for(int c=0;c<4;++c) out.m[r*4+c]=a[r][4+c];
    return true;
}

Vec3 Mat4::mul_point(const Mat4& M, const Vec3& p){
    double x = M.m[0]*p.x + M.m[4]*p.y + M.m[8]*p.z + M.m[12];
    double y = M.m[1]*p.x + M.m[5]*p.y + M.m[9]*p.z + M.m[13];
    double z = M.m[2]*p.x + M.m[6]*p.y + M.m[10]*p.z + M.m[14];
    double w = M.m[3]*p.x + M.m[7]*p.y + M.m[11]*p.z + M.m[15];
    if(std::abs(w)>1e-12){ x/=w; y/=w; z/=w; }
    return {x,y,z};
}
Vec3 Mat4::mul_dir(const Mat4& M, const Vec3& v){
    return {
        M.m[0]*v.x + M.m[4]*v.y + M.m[8]*v.z,
        M.m[1]*v.x + M.m[5]*v.y + M.m[9]*v.z,
        M.m[2]*v.x + M.m[6]*v.y + M.m[10]*v.z
    };
}
