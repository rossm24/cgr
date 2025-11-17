/*

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
*/

#include "mat4.h"

// helper for indexing
static inline double& E(Mat4& M, int row, int col) {
    return M.m[row*4 + col];
}
static inline double  E(const Mat4& M, int row, int col) {
    return M.m[row*4 + col];
}

Mat4 Mat4::identity() {
    Mat4 M;
    for (int i = 0; i < 16; ++i) M.m[i] = 0.0;
    E(M,0,0) = 1.0; E(M,1,1) = 1.0; E(M,2,2) = 1.0; E(M,3,3) = 1.0;
    return M;
}

Mat4 Mat4::translate(const Vec3& t) {
    Mat4 M = identity();
    E(M,0,3) = t.x;
    E(M,1,3) = t.y;
    E(M,2,3) = t.z;
    return M;
}

Mat4 Mat4::scale(const Vec3& s) {
    Mat4 M;
    for (int i = 0; i < 16; ++i) M.m[i] = 0.0;
    E(M,0,0) = s.x;
    E(M,1,1) = s.y;
    E(M,2,2) = s.z;
    E(M,3,3) = 1.0;
    return M;
}

Mat4 Mat4::rotateX(double r) {
    Mat4 M = identity();
    double c = std::cos(r);
    double s = std::sin(r);
    E(M,1,1) =  c; E(M,1,2) = -s;
    E(M,2,1) =  s; E(M,2,2) =  c;
    return M;
}

Mat4 Mat4::rotateY(double r) {
    Mat4 M = identity();
    double c = std::cos(r);
    double s = std::sin(r);
    E(M,0,0) =  c; E(M,0,2) =  s;
    E(M,2,0) = -s; E(M,2,2) =  c;
    return M;
}

Mat4 Mat4::rotateZ(double r) {
    Mat4 M = identity();
    double c = std::cos(r);
    double s = std::sin(r);
    E(M,0,0) =  c; E(M,0,1) = -s;
    E(M,1,0) =  s; E(M,1,1) =  c;
    return M;
}

// C = A * B   (column-vector convention: v' = C * v)
Mat4 Mat4::multiply(const Mat4& A, const Mat4& B) {
    Mat4 C;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            double sum = 0.0;
            for (int k = 0; k < 4; ++k) {
                sum += E(A,i,k) * E(B,k,j);
            }
            E(C,i,j) = sum;
        }
    }
    return C;
}

Mat4 Mat4::transpose(const Mat4& A) {
    Mat4 T;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            E(T,j,i) = E(A,i,j);
    return T;
}

// If you already have a working invert(), keep that; otherwise you can fill this in later.
bool Mat4::invert(const Mat4& A, Mat4& out) {
    // Gauss–Jordan elimination on [A | I]
    double aug[4][8];

    // Build augmented matrix
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            aug[i][j]     = E(A, i, j);
            aug[i][4 + j] = (i == j) ? 1.0 : 0.0;
        }
    }

    // Forward elimination
    for (int col = 0; col < 4; ++col) {
        // Find pivot row
        int pivot = col;
        double maxAbs = std::fabs(aug[pivot][col]);
        for (int row = col + 1; row < 4; ++row) {
            double v = std::fabs(aug[row][col]);
            if (v > maxAbs) {
                maxAbs = v;
                pivot = row;
            }
        }

        // Singular?
        if (maxAbs < 1e-12)
            return false;

        // Swap pivot row
        if (pivot != col) {
            for (int j = 0; j < 8; ++j)
                std::swap(aug[col][j], aug[pivot][j]);
        }

        // Normalize pivot row
        double diag = aug[col][col];
        for (int j = 0; j < 8; ++j)
            aug[col][j] /= diag;

        // Eliminate this column from other rows
        for (int row = 0; row < 4; ++row) {
            if (row == col) continue;
            double factor = aug[row][col];
            if (std::fabs(factor) < 1e-18) continue;
            for (int j = 0; j < 8; ++j)
                aug[row][j] -= factor * aug[col][j];
        }
    }

    // Extract inverse from right block
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            E(out, i, j) = aug[i][4 + j];

    return true;
}

// Apply to point (x,y,z,1)
Vec3 Mat4::mul_point(const Mat4& M, const Vec3& p) {
    double x = p.x, y = p.y, z = p.z;

    double xp = E(M,0,0)*x + E(M,0,1)*y + E(M,0,2)*z + E(M,0,3)*1.0;
    double yp = E(M,1,0)*x + E(M,1,1)*y + E(M,1,2)*z + E(M,1,3)*1.0;
    double zp = E(M,2,0)*x + E(M,2,1)*y + E(M,2,2)*z + E(M,2,3)*1.0;
    double wp = E(M,3,0)*x + E(M,3,1)*y + E(M,3,2)*z + E(M,3,3)*1.0;

    if (std::fabs(wp) > 1e-8) {
        xp /= wp; yp /= wp; zp /= wp;
    }
    return Vec3{xp, yp, zp};
}

// Apply to direction (x,y,z,0)  – ignores translation
Vec3 Mat4::mul_dir(const Mat4& M, const Vec3& v) {
    double x = v.x, y = v.y, z = v.z;

    double xp = E(M,0,0)*x + E(M,0,1)*y + E(M,0,2)*z;
    double yp = E(M,1,0)*x + E(M,1,1)*y + E(M,1,2)*z;
    double zp = E(M,2,0)*x + E(M,2,1)*y + E(M,2,2)*z;

    return Vec3{xp, yp, zp};
}
