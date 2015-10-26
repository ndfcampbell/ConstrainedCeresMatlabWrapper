%
% Matlab mex wrapper for constrained optimisation using ceres and fmincon.
% This should not need editing, the problem setup is in problemToSolve.h.
%
% Neill D.F. Campbell, 2015
%
% Persistent C++ Objects from here: 
% http://uk.mathworks.com/matlabcentral/newsreader/view_thread/278243
%
%
classdef CeresWrapper < handle
    
    properties (Hidden = true, SetAccess = private)
        cpp_handle;
    end
    
    methods
        % Constructor
        function this = CeresWrapper()
            this.cpp_handle = mexCeres('Init');
        end
        
        % Destructor
        function delete(this)
            mexCeres('Delete', this.cpp_handle);
        end
        
        % Get objective and gradient for set of parameters..
        function [objective, gradient] = ObjectiveFunc(this, parameters)
            [objective, gradient] = mexCeres('Objective', this.cpp_handle, parameters);
        end
        
        % Get the constraint residuals and their jacobians for the
        % parameters..
        function [Cineq, Ceq, Jineq, Jeq] = ConstraintsFunc(this, parameters)
            [Cineq, Ceq, Jineq, Jeq] =  mexCeres('Constraints', this.cpp_handle, parameters);
        end
        
        function [numParams] = NumParameters(this)
            [numParams] = mexCeres('NumParameters', this.cpp_handle);
        end
        
        function [parameters] = GetParameters(this)
            [parameters] = mexCeres('GetParameters', this.cpp_handle);
        end
        
        function [lb, ub] = GetLowerAndUpperBounds(this)
            [lb, ub] = mexCeres('GetLowerAndUpperBounds', this.cpp_handle);
        end
        
        function Display(this)
            mexCeres('Display', this.cpp_handle);
        end
        
        function [optParameters, optObjective] = NLOpt(this, parameters)
            [optParameters, optObjective] = mexCeres('NLOpt', this.cpp_handle, parameters);
        end
    end
    
end