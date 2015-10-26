# This may need to be customised to point to the installations of Eigen3, Ceres, GLog and GFlags
#
# Neill D.F. Campbell, 2015
#

default: all
	
mexCeres.mexmaci64: ConstrainedCeresProblem.cpp ConstrainedCeresProblem.hpp mexCeres.cpp mexDefs.h
	mex -v -o mexCeres -largeArrayDims -I/usr/local/include -I/usr/local/include/eigen3 mexCeres.cpp ConstrainedCeresProblem.cpp -L/usr/local/lib -lceres -lglog -lgflags

all: mexCeres.mexmaci64

