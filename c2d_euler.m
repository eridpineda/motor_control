%==========================================================================
%
% c2d_euler  Transforms a continuous transfer function to a discrete 
% transfer function using the forward and backward Euler methods.
%
%   Hz = c2d_euler(Hs,T,'forward')
%   Hz = c2d_euler(Hs,T,'backward')
%
% See also c2d.
%
% Copyright © 2021 Tamas Kis
% Last Update: 2022-09-26
% Website: https://tamaskis.github.io
% Contact: tamas.a.kis@outlook.com
%
% TECHNICAL DOCUMENTATION:
% https://tamaskis.github.io/files/Continuous_to_Discrete_Transfer_Function_Transformation_Using_the_Euler_Methods.pdf
%
% DEPENDENCIES:
%   • Control System Toolbox
%   • Symbolic Math Toolbox
%
%--------------------------------------------------------------------------
%
% ------
% INPUT:
% ------
%   Hs      - (1×1 tf or zpk) continous transfer function
%   T       - (1×1 double) sampling period
%   type    - (char) 'forward' or 'backward'
%   output  - (OPTIONAL) (char) specifies output type ('tf' or 'zpk')
%
% -------
% OUTPUT:
% -------
%   Hz      - (1×1 tf or zpk) discrete transfer function
%
%==========================================================================
function Hz = c2d_euler(Hs,T,type,output)
    
    % defaults "output" to 'tf' if not input
    if (nargin < 4) || isempty(output)
        output = 'tf';
    end
    
    % symbolic variable for z;
    z = sym('z');
    
    % specified Euler approximation of s
    if strcmpi(type,'backward')
        s = (z-1)/(T*z);
    else
        s = (z-1)/T;
    end
    
    % converts transfer function object to symbolic function object
    [num,den] = tfdata(Hs);
    Hz = poly2sym(cell2mat(num),z)/poly2sym(cell2mat(den),z);
    
    % performs Euler transformation
    Hz = simplify(subs(Hz,s));
    
    % obtains numerator and denominator of symbolic expression in MATLAB's
    % "polynomial form"
    [sym_num,sym_den] = numden(Hz);
    num = sym2poly(sym_num);
    den = sym2poly(sym_den);
    
    % creates discrete transfer function model
    Hz = tf(num,den,T);
    
    % converts discrete transfer function model to discrete zero-pole-gain
    % model if specified
    if strcmpi(output,'zpk')
        Hz = zpk(Hz);
    end
    
end