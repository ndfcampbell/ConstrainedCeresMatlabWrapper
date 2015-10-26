//
// A class to wrap around a ceres problem and use it to evaluate cost functions attributed
// to either objectives, inequality (residuals <= 0), or equality (residuals = 0) constraints.
//
// Neill D.F. Campbell, 2015
//

#include "mexDefs.h"

#include "ConstrainedCeresProblem.hpp"

mxArray* ConvertToMatlab(const double& d)
{
    mxArray* mx = mxCreateDoubleScalar(d);
    return mx;
}

mxArray* ConvertToMatlab(const std::vector<double>& v)
{
    mxArray* mx = mxCreateDoubleMatrix(v.size(), 1, mxREAL);
    memcpy(mxGetPr(mx), v.data(), sizeof(double) * v.size());
    return mx;
}

mxArray* ConvertToMatlab(const ceres::CRSMatrix& mat)
{
    // ceres::CRSMatrix Documentation:
    //
    //          -row0-  ---row1---  -row2-
    //rows   = [ 0,      2,          5,     7]
    //cols   = [ 1,  3,  1,  2,  3,  0,  1]
    //values = [10,  4,  2, -3,  2,  1,  2]

    // NOTE MATLAB TAKES THE TRANSPOSE (RCS Matrix..)
    //
    // BOO - STUPID MATLAB..
    //

    const size_t nnz = mat.values.size(); // num non-zeros
    const size_t M = mat.num_cols; // num rows (transpose)
    const size_t N = mat.num_rows; // num cols (transpose)

    mxArray* mx = mxCreateSparse(M, N, nnz, mxREAL);

    if (nnz == 0)
    {
        // Return empty matrix..
        vdbg(mat.rows.size());
        vdbg(mat.cols.size());

        return mx;
    }

    // Pointer to values..
    double* Pr = mxGetPr(mx);

    // Pointer to column indices..
    mwIndex* Jc = mxGetJc(mx);

    // Pointer to row indices.. this is the same length as values..
    mwIndex* Ir = mxGetIr(mx);

    mxAssert (nnz > 0);

    memcpy(Pr, &mat.values[0], sizeof(double) * nnz);

    for (int i = 0; i < (N+1); ++i)
    {
        Jc[i] = mat.rows[i];
    }
    for (int i = 0; i < nnz; ++i)
    {
        Ir[i] = mat.cols[i];
    }

    return mx;
}

CeresConstrainedProblem* GetProblemFromMatlab(const mxArray* mx)
{
    CeresConstrainedProblem* prob = NULL;
    MatlabToPointer(mx, &prob);

    mexInfoPrintf("\nINFO: Checking Pointer (%llu)\n\n", static_cast<void*>(prob));
    mxAssert (CeresConstrainedProblem::IsAnAllocatedPointer(prob));

    return prob;
}
