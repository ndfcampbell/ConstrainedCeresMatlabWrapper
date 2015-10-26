#ifndef MEXDEFS_H
#define MEXDEFS_H

#include <mex.h>
#include <math.h>
#include <matrix.h>

#ifdef mxAssert
#undef mxAssert
#endif
#define mxAssert(stat) if (!(stat)) { mexPrintf("\n\nFailed assert \"%s\" in %s(%d)\n", #stat, __FILE__, __LINE__); mexErrMsgTxt("Failed Assert\n"); }

#ifdef DEBUG
#define mexInfoPrintf mexPrintf
#else
#define mexInfoPrintf(...) ((void) 0)
#endif

#ifndef vdbg
#define vdbg(var) std::cout << #var << " = " << (var) << std::endl;
#endif

#ifndef vvdbg
#define vvdbg(var) std::cout << __FILE__ << "(" << __LINE__ << ") " << #var << " = " << (var) << std::endl;
#endif

#endif // MEXDEFS_H
