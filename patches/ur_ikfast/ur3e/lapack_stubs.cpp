// LAPACK forwarder for Windows builds.
//
// ur_ikfast's generated C++ calls the standard Fortran LAPACK symbols
// (dgetrf_, dgeev_, ...). On Windows we link against scipy-openblas32,
// which renames all exports with a "scipy_" prefix. This file provides
// the unprefixed symbols by forwarding to the scipy_* ones.
//
// The scipy_* variants of char-taking routines (dgetrs_, dgeev_) expect
// trailing hidden strlen arguments (gfortran ABI). We pass 1 because
// LAPACK only inspects the first character. Extra args on Windows x64
// are ignored if the callee doesn't use them, so this is ABI-safe.

#include <cstddef>

extern "C" {
    extern void scipy_dgetrf_(const int* m, const int* n, double* a, const int* lda, int* ipiv, int* info);
    extern void scipy_zgetrf_(const int* m, const int* n, void* a, const int* lda, int* ipiv, int* info);
    extern void scipy_dgetri_(const int* n, double* a, const int* lda, const int* ipiv, double* work, const int* lwork, int* info);
    extern void scipy_dgesv_(const int* n, const int* nrhs, double* a, const int* lda, int* ipiv, double* b, const int* ldb, int* info);
    extern void scipy_dgetrs_(const char* trans, const int* n, const int* nrhs, const double* a, const int* lda, const int* ipiv, double* b, const int* ldb, int* info, std::size_t trans_len);
    extern void scipy_dgeev_(const char* jobvl, const char* jobvr, const int* n, double* a, const int* lda, double* wr, double* wi, double* vl, const int* ldvl, double* vr, const int* ldvr, double* work, const int* lwork, int* info, std::size_t jobvl_len, std::size_t jobvr_len);

    void dgetrf_(const int* m, const int* n, double* a, const int* lda, int* ipiv, int* info) {
        scipy_dgetrf_(m, n, a, lda, ipiv, info);
    }
    void zgetrf_(const int* m, const int* n, void* a, const int* lda, int* ipiv, int* info) {
        scipy_zgetrf_(m, n, a, lda, ipiv, info);
    }
    void dgetri_(const int* n, const double* a, const int* lda, int* ipiv, double* work, const int* lwork, int* info) {
        scipy_dgetri_(n, const_cast<double*>(a), lda, ipiv, work, lwork, info);
    }
    void dgesv_(const int* n, const int* nrhs, double* a, const int* lda, int* ipiv, double* b, const int* ldb, int* info) {
        scipy_dgesv_(n, nrhs, a, lda, ipiv, b, ldb, info);
    }
    void dgetrs_(const char* trans, const int* n, const int* nrhs, double* a, const int* lda, int* ipiv, double* b, const int* ldb, int* info) {
        scipy_dgetrs_(trans, n, nrhs, a, lda, ipiv, b, ldb, info, 1);
    }
    void dgeev_(const char* jobvl, const char* jobvr, const int* n, double* a, const int* lda, double* wr, double* wi, double* vl, const int* ldvl, double* vr, const int* ldvr, double* work, const int* lwork, int* info) {
        scipy_dgeev_(jobvl, jobvr, n, a, lda, wr, wi, vl, ldvl, vr, ldvr, work, lwork, info, 1, 1);
    }
}
