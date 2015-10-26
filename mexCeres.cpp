#include "mexDefs.h"

#include <mex.h>
#include <math.h>
#include <matrix.h>

#include <ceres/ceres.h>

#include <tuple>
#include <set>
#include <string>
using namespace std;

#include "ConstrainedCeresProblem.hpp"

// CHANGE THIS FILE
#include "problemToSolve.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    mxAssert (nrhs > 0);

    std::string funcType = mxArrayToString(prhs[0]);

    mexInfoPrintf("\nINFO: Calling Mex Function (%s)\n\n", funcType.c_str());

    if (funcType.compare("Init") == 0)
    {
		mxAssert (nlhs == 1);
		mxAssert (nrhs == 1);

		CeresConstrainedProblem* prob = CreateProblem();

        mexInfoPrintf("\nINFO: Created Problem (%llu)\n\n", static_cast<void*>(prob));

        plhs[0] = PointerToMatlab(prob);
	}
    else
    {
        // All these functions require the second argument to be the pointer..
        mxAssert (nrhs >= 2);

        CeresConstrainedProblem* prob = GetProblemFromMatlab(prhs[1]);

        if (funcType.compare("Delete") == 0)
        {
            mxAssert (nrhs == 2);

            mexInfoPrintf("\nINFO: Deleting Problem (%llu)\n\n", static_cast<void*>(prob));

            delete prob;

            // Reset saved pointer in an effort for some form of safety..
            *static_cast<uint64_t*>(mxGetData(prhs[1])) = 0;
        }
        else if (funcType.compare("Display") == 0)
        {
            mxAssert (nrhs == 2);

            mexPrintf("Num Params = %d\nNum Equality Constraints = %d\nNum Inequality Constraints = %d\n",
                      prob->NumParameters(), prob->NumEqualityConstraints(), prob->NumInequalityConstraints());
        }
        else if (funcType.compare("Objective") == 0)
        {
            mxAssert (nrhs == 3); // 3 = Parameter Values
            mxAssert (nlhs == 2);

            prob->UpdateAllParameters(prhs[2]);

            mxArray* objective;
            mxArray* gradient;
            std::tie(objective, gradient) = prob->GetObjectiveAndGradient();

            plhs[0] = objective;
            plhs[1] = gradient;
        }
        else if (funcType.compare("Constraints") == 0)
        {
            mxAssert (nrhs == 3); // 3 = Parameter Values
            mxAssert (nlhs == 4); // Cineq, Ceq, GradIneq, GradEq

            prob->UpdateAllParameters(prhs[2]);

            mxArray* inequalityConstraints;
            mxArray* inequalityJacobian;
            std::tie(inequalityConstraints, inequalityJacobian) =
                    prob->GetInequalityConstraintsAndJacobian();

            mxArray* equalityConstraints;
            mxArray* equalityJacobian;
            std::tie(equalityConstraints, equalityJacobian) =
                    prob->GetEqualityConstraintsAndJacobian();

            plhs[0] = inequalityConstraints;
            plhs[1] = equalityConstraints;
            plhs[2] = inequalityJacobian;
            plhs[3] = equalityJacobian;
        }
        else if (funcType.compare("NumParameters") == 0)
        {
            mxAssert (nrhs == 2);
            mxAssert (nlhs == 1);

            double numParams = static_cast<double>(prob->NumParameters());
            plhs[0] = mxCreateDoubleScalar(numParams);
        }
        else if (funcType.compare("GetParameters") == 0)
        {
            mxAssert (nrhs == 2);
            mxAssert (nlhs == 1);

            std::vector<double> x;
            prob->GetParams(x);
            mxArray* optimParams = ConvertToMatlab(x);

            plhs[0] = optimParams;
		}
        else
        {
            mexErrMsgTxt("Invalid Function!\n");
        }
    }
}
