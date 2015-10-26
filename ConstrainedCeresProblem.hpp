//
// A class to wrap around a ceres problem and use it to evaluate cost functions attributed
// to either objectives, inequality (residuals <= 0), or equality (residuals = 0) constraints.
//
// Neill D.F. Campbell, 2015
//
#ifndef CONSTRAINEDCERESPROBLEM_HPP
#define CONSTRAINEDCERESPROBLEM_HPP

#include "mexDefs.h"

#include <mex.h>
#include <math.h>
#include <matrix.h>

#include <ceres/ceres.h>

#include <tuple>
#include <set>


// Forward declaration
class CeresConstrainedProblem;


/// -- Functions to convert to matlab arrays.. --

mxArray* ConvertToMatlab(const double& d);

mxArray* ConvertToMatlab(const std::vector<double>& v);

mxArray* ConvertToMatlab(const ceres::CRSMatrix& mat);

template <typename T> mxArray* PointerToMatlab(T* pointer)
{
    mxArray* mx = mxCreateNumericMatrix_730(1, 1, mxUINT64_CLASS, mxREAL);

    uint64_t* d = static_cast<uint64_t*>(mxGetData(mx));

    void* p = static_cast<void*>(pointer);

    d[0] = reinterpret_cast<uint64_t>(p);

    return mx;
}

template <typename T> void MatlabToPointer(const mxArray* mx, T** pointer)
{
    mxAssert (mxIsUint64(mx));
    mxAssert (mxGetM(mx) == 1);
    mxAssert (mxGetN(mx) == 1);

    uint64_t* d = static_cast<uint64_t*>(mxGetData(mx));

    void* p = reinterpret_cast<void*>(d[0]);

    (*pointer) = static_cast<T*>(p);

    mxAssert (pointer != NULL);
}

CeresConstrainedProblem* GetProblemFromMatlab(const mxArray* mx);


///
/// NOTE: We assume all params allocated with new[]..
///
class CeresConstrainedProblem
{
public:
    static inline bool IsAnAllocatedPointer(CeresConstrainedProblem* pointer)
    {
        return InternalIsAnAllocatedPointer(pointer);
    }

public:
    enum ConstraintType {
        Equality,
        Inequality
    };

    CeresConstrainedProblem(std::shared_ptr<ceres::Problem> problem,
                            const std::vector<ceres::ResidualBlockId>& objectiveIDs,
                            const std::vector<ceres::ResidualBlockId>& equalityConstraintIDs,
                            const std::vector<ceres::ResidualBlockId>& inequalityConstraintIDs,
                            const std::vector<double*>& parameterBlocks = std::vector<double*>(),
                            const bool deleteParameters = true) :
        _problem(problem), _objectiveIDs(objectiveIDs),
        _equalityConstraintIDs(equalityConstraintIDs),
        _inequalityConstraintIDs(inequalityConstraintIDs),
        _parameterBlocks(parameterBlocks),
        _deleteParameters(deleteParameters)
    {
        mxAssert (_problem.get() != NULL);

        mxAssert (_objectiveIDs.size() > 0);
        mxAssert ((_equalityConstraintIDs.size() > 0) || (_inequalityConstraintIDs.size() > 0));

        if (_parameterBlocks.empty())
        {
            _problem->GetParameterBlocks(&_parameterBlocks);
        }
        else
        {
            std::vector<double*> blocks;
            _problem->GetParameterBlocks(&blocks);

            std::set<double*> blockSet(blocks.begin(), blocks.end());

            for (double* p : _parameterBlocks)
            {
                mxAssert (blockSet.count(p) == 1);
            }
        }

        CeresConstrainedProblem::InternalIsAnAllocatedPointer(this, true);
    }

    ~CeresConstrainedProblem()
    {
        // Delete the param memory as well..
        //
        // NOTE: We assume all params allocated with new[]..
        //

        CeresConstrainedProblem::InternalIsAnAllocatedPointer(this, false, true);

        if (_deleteParameters)
        {
            std::vector<double*> pBlocks;
            _problem->GetParameterBlocks(&pBlocks);

            mexInfoPrintf("\nINFO: Deleting parameters..\n\n");

            for (double* p : pBlocks)
            {
                delete[] p;
            }
        }
    }

    int NumParameters() const
    {
        return _problem->NumParameters();
    }

    int NumEqualityConstraints() const
    {
        return CountConstraints(_equalityConstraintIDs);
    }

    int NumInequalityConstraints() const
    {
        return CountConstraints(_inequalityConstraintIDs);
    }

    void UpdateAllParameters(const mxArray* matlabParameters)
    {
        size_t N = mxGetNumberOfElements(matlabParameters);

        mxAssert (N == NumParameters());

        UpdateAllParameters(mxGetPr(matlabParameters));
    }

    void UpdateAllParameters(const double* parameters)
    {
        std::vector<double*> pBlocks;

        if (_parameterBlocks.empty())
        {
            _problem->GetParameterBlocks(&pBlocks);
        }
        else
        {
            pBlocks = _parameterBlocks;
        }

        int total = 0;
        for (double* p : pBlocks)
        {
            const int S = _problem->ParameterBlockLocalSize(p);
            total += S;

            for (int i = 0; i < S; ++i)
            {
                p[i] = *(parameters);
                ++parameters;
            }
        }

        mxAssert (total == NumParameters());
    }

    void GetParams(std::vector<double>& params) const
    {
        params.clear();
        params.reserve(NumParameters());

        std::vector<double*> pBlocks;

        if (_parameterBlocks.empty())
        {
            _problem->GetParameterBlocks(&pBlocks);
        }
        else
        {
            pBlocks = _parameterBlocks;
        }

        int total = 0;
        for (double* p : pBlocks)
        {
            const int S = _problem->ParameterBlockLocalSize(p);
            total += S;

            for (int i = 0; i < S; ++i)
            {
                params.push_back(p[i]);
            }
        }

        mxAssert (total == NumParameters());
        mxAssert (params.size() == NumParameters());
    }

    void GetObjectiveAndGradient(double& objective, std::vector<double>& gradient) const
    {
        objective = 0;
        gradient.clear();

        ceres::Problem::EvaluateOptions options;

        options.apply_loss_function = true;
        options.residual_blocks = _objectiveIDs;
        options.parameter_blocks = _parameterBlocks;

        _problem->Evaluate(options, &objective, NULL, &gradient, NULL);

        mxAssert (gradient.size() == NumParameters());
    }

    std::tuple<mxArray*, mxArray*> GetObjectiveAndGradient() const
    {
        double objective = 0;
        std::vector<double> gradient;

        GetObjectiveAndGradient(objective, gradient);

        mxArray* matObjective = ConvertToMatlab(objective);
        mxArray* matGrad = ConvertToMatlab(gradient);

        return std::tuple<mxArray*, mxArray*>(matObjective, matGrad);
    }

    inline void GetConstraintsAndJacobian(const ConstraintType constraintType,
                                          std::vector<double>& residuals,
                                          ceres::CRSMatrix& jacobian) const
    {
        switch (constraintType)
        {
        case Equality:
            GetEqualityConstraintsAndJacobian(residuals, jacobian);
            break;
        case Inequality:
            GetInequalityConstraintsAndJacobian(residuals, jacobian);
            break;
        default:
            mexErrMsgTxt("Incorrect Constraint Type Specified.");
            break;
        }
    }

    inline void GetEqualityConstraintsAndJacobian(std::vector<double>& residuals,
                                                  ceres::CRSMatrix& jacobian) const
    {
        GetConstraintsAndJacobian(residuals, jacobian, _equalityConstraintIDs);
    }

    inline std::tuple<mxArray*, mxArray*> GetEqualityConstraintsAndJacobian() const
    {
        return GetConstraintsAndJacobian(_equalityConstraintIDs);
    }

    inline void GetInequalityConstraintsAndJacobian(std::vector<double>& residuals,
                                                    ceres::CRSMatrix& jacobian) const
    {
        GetConstraintsAndJacobian(residuals, jacobian, _inequalityConstraintIDs);
    }

    inline std::tuple<mxArray*, mxArray*> GetInequalityConstraintsAndJacobian() const
    {
        return GetConstraintsAndJacobian(_inequalityConstraintIDs);
    }

    void PrintParameters() const
    {
        std::vector<double> params;
        GetParams(params);

        mexPrintf("Params = \n");
        for (const double p : params)
        {
            mexPrintf("         %e\n", p);
        }
        mexPrintf("\n");
    }

private:

    void GetConstraintsAndJacobian(std::vector<double>& residuals, ceres::CRSMatrix& jacobian,
                                   const std::vector<ceres::ResidualBlockId>& constraintIDs) const
    {
        residuals.clear();
        jacobian.cols.clear();
        jacobian.rows.clear();
        jacobian.values.clear();

        if (constraintIDs.empty())
        {
            // Return empty matrices..
            return;
        }

        ceres::Problem::EvaluateOptions options;

        options.apply_loss_function = false;
        options.residual_blocks = constraintIDs;
        options.parameter_blocks = _parameterBlocks;

        _problem->Evaluate(options, NULL, &residuals, NULL, &jacobian);

        mxAssert (residuals.size() > 0);
    }

    std::tuple<mxArray*, mxArray*> GetConstraintsAndJacobian(
            const std::vector<ceres::ResidualBlockId>& constraintIDs) const
    {
        if (constraintIDs.empty())
        {
            // Return empty matrices..
            return std::tuple<mxArray*, mxArray*>(mxCreateDoubleMatrix(0, 0, mxREAL),
                                                  mxCreateDoubleMatrix(0, 0, mxREAL));
        }

        std::vector<double> residuals;
        ceres::CRSMatrix jacobian;

        GetConstraintsAndJacobian(residuals, jacobian, constraintIDs);

        mxArray* matConstriants = ConvertToMatlab(residuals);
        mxArray* matJacobian = ConvertToMatlab(jacobian);

        return std::tuple<mxArray*, mxArray*>(matConstriants, matJacobian);
    }

    static bool InternalIsAnAllocatedPointer(CeresConstrainedProblem* pointer,
                                             bool addToDatabase = false,
                                             bool removeFromDatabase = false)
    {
        mexInfoPrintf("\nINFO: InternalIsAnAllocatedPointer (%llu, %d, %d)\n\n",
                  static_cast<void*>(pointer), addToDatabase, removeFromDatabase);

        static std::set<CeresConstrainedProblem*> globalAllocatedHandles;


        if (addToDatabase)
        {
            globalAllocatedHandles.insert(pointer);

            // This is necessary to ensure that all the delete calls can be made before
            // the mex file is removed from memory. Each lock has a corresponding unlock.
            mexLock();
        }
        else if (removeFromDatabase)
        {
            globalAllocatedHandles.erase(pointer);

            // This is necessary to ensure that all the delete calls can be made before
            // the mex file is removed from memory. Each lock has a corresponding unlock.
            mexUnlock();
        }

        return (globalAllocatedHandles.count(pointer) > 0);
    }

    int CountConstraints(const std::vector<ceres::ResidualBlockId>& constraintIDs) const
    {
        int numConstraints = 0;

        for (const ceres::ResidualBlockId id : constraintIDs)
        {
            const ceres::CostFunction* cf = _problem->GetCostFunctionForResidualBlock(id);

            numConstraints += cf->num_residuals();
        }

        return numConstraints;
    }

    std::shared_ptr<ceres::Problem> _problem;
    std::vector<ceres::ResidualBlockId> _objectiveIDs;
    std::vector<ceres::ResidualBlockId> _equalityConstraintIDs;
    std::vector<ceres::ResidualBlockId> _inequalityConstraintIDs;
    std::vector<double*> _parameterBlocks;
    const bool _deleteParameters;

};

#endif // CONSTRAINEDCERESPROBLEM_HPP
