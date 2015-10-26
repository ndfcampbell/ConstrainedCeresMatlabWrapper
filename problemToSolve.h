//
// Specify the problem functions here and allocate them to objectives or constraints (inequality <= 0,
// equality = 0) in the CreateProblem function using the appropriate vectors.
//
// Neill D.F. Campbell, 2015
//
#ifndef PROBLEM_TO_SOLVE_H
#define PROBLEM_TO_SOLVE_H

#define USE_GOOGLE_LOG

#include <ceres/ceres.h>

#include <cmath>
using namespace std;

#include "ConstrainedCeresProblem.hpp"

struct constraint_1
{
	template <typename T> bool operator()(const T* const x, T* residual) const
	{
		T x0 = x[0];
		T x1 = x[1];
		T x2 = x[2];

		residual[0] = ((x0 - x1*T(2.0))) * T(10.0) ;
		residual[1] = ((x0 + x1) - x2) * T(10.0) ;
		return true;
	}
};

struct constraint_2
{
	template <typename T> bool operator()(const T* const x, T* residual) const
	{
		T x0 = x[0];
		T x1 = x[1];
		T x2 = x[2];

		residual[0] = (T(10.0) - x0) * T(10.0)  ;
		residual[1] = (T(5) - x1)  * T(10.0) ;
		residual[2] = (T(20) - x2) * T(10.0) ;

		return true;
	}
};

struct equality_constraint
{
	template <typename T> bool operator()(const T* const x, T* residual) const
	{
		T x0 = x[0];
		T x1 = x[1];
		T x2 = x[2];

		residual[0] = (T(6.0) - x1);

		return true;
	}
};

struct objective
{
	template <typename T> bool operator()(const T* const x, T* residual) const
	{
		T x0 = x[0];
		T x1 = x[1];
		T x2 = x[2];

		residual[0] = x0 + x1 + x2;

		return true;
	}
};

CeresConstrainedProblem* CreateProblem()
{
#ifdef USE_GOOGLE_LOG
	// This should really be called from a main function therefore not a library..
	static bool doneGoogleInit = false;
	if (!doneGoogleInit)
	{
		mexInfoPrintf("\nINFO: InitGoogleLogging\n\n");

		google::InitGoogleLogging("mexCeres");
		doneGoogleInit = true;
	}
#endif // USE_GOOGLE_LOG

	//
	// NOTE: Very IMPORTANT that the parameters are created with new[] since they
	// will be deleted with delete[].
	//
	const int param_size = 3;
	double* param = new double[param_size];

	for(int i = 0; i < param_size; ++i)
	{
		param[i] = rand()/(RAND_MAX/1.0);
	}

	// Important to create the ceres::Problem as a shared_ptr..
	std::shared_ptr<ceres::Problem> problem(new ceres::Problem());
	
	std::vector<ceres::ResidualBlockId> objectiveIDs;
	std::vector<ceres::ResidualBlockId> equalityConstraintIDs;
	std::vector<ceres::ResidualBlockId> inequalityConstraintIDs;

	ceres::CostFunction* constraint_function_1 =
		new ceres::AutoDiffCostFunction<constraint_1, 2, param_size>(new constraint_1);
	inequalityConstraintIDs.push_back(problem->AddResidualBlock(constraint_function_1, NULL, param));

	ceres::CostFunction* constraint_function_2 =
		new ceres::AutoDiffCostFunction<constraint_2 , 3, param_size>(new constraint_2);
	inequalityConstraintIDs.push_back(problem->AddResidualBlock(constraint_function_2, NULL, param));

	ceres::CostFunction* equality_constraint_function =
			new ceres::AutoDiffCostFunction<equality_constraint, 1, param_size>(new equality_constraint);
	equalityConstraintIDs.push_back(problem->AddResidualBlock(equality_constraint_function, NULL, param));

	ceres::CostFunction* objectiveFunc =
			new ceres::AutoDiffCostFunction<objective, 1, param_size>(new objective);
	objectiveIDs.push_back(problem->AddResidualBlock(objectiveFunc, NULL, param));

	CeresConstrainedProblem* constrainedProb;
	constrainedProb = new CeresConstrainedProblem(problem, objectiveIDs,
												  equalityConstraintIDs,
												  inequalityConstraintIDs);

	return constrainedProb;
}

#endif // PROBLEM_TO_SOLVE_H
