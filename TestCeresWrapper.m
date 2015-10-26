% Function to test the ceres mex wrapper for constrained optimisation.
%
% The problem is setup in the header file 'problemToSolve.h'.
%
% If the mexCeres mex file does not exist please run make.
%
% Neill D.F. Campbell, 2015
%
function TestCeresWrapper()

% Setup constrained solver options..
opts = optimoptions(@fmincon, 'Algorithm', 'sqp');
%opts = optimoptions(@fmincon, 'Algorithm', 'interior-point');
opts = optimoptions(opts, 'Display', 'iter');
opts = optimoptions(opts, 'GradConstr', 'on');
opts = optimoptions(opts, 'GradObj', 'on');
opts = optimoptions(opts, 'TolConSQP', 1E-10);
opts = optimoptions(opts, 'DerivativeCheck', 'on');

% Set an initial value for the optimisation
rand('seed', 0);
x_initial = 15 * rand(3,1);

disp('Input x is');
disp(x_initial');

C = CeresWrapper;

% After optimisation the inequality residuals should be <= 0 and the
% equality residuals should be zero.

[InequalityResiduals, EqualityResiduals] = C.ConstraintsFunc(x_initial)

x_final = fmincon(@C.ObjectiveFunc, x_initial, [], [], [], [], [], [], @C.ConstraintsFunc, opts);

disp('Output x is');
disp(x_final');

[InequalityResiduals, EqualityResiduals] = C.ConstraintsFunc(x_final)

end
