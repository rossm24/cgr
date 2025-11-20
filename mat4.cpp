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
