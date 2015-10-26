# ConstrainedCeresMatlabWrapper
Neill D.F. Campbell, 2015

A Matlab mex wrapper to allow ceres AutoDiff cost functions to be used for constrained optimisation via fmincon.

The dependencies are: Eigen3, Ceres, GLog, and GFlags. The Makefile will probably need to be customised on 
different platforms. The file as it is will probably work on MacOSX with eigen3, ceres-solver and glog installed
using homebrew.

The TestCeresWrapper.m file shows an example; the specific problem being solved is specified in problemToSolve.h.
All other files should not need adjusting for specific problems. 

Remember to allocate the parameters in the c++ file using new[] and to make sure you pass the correct number 
of parameters at the appropriate times. 

The objective cost functions will be set as a sum of square of the residuals, the equality constraints set all 
the residuals to zero and the inequality constraints set all the residuals <= 0.
