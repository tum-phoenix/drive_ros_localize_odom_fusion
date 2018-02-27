% MATLAB implementation of the Constant Turn Rate and Velocity Vehicle Model
% 
% Used primarly to compute the System Model Jacobian and Limits for Omega -> 0
% using MATLAB Symbolic Toolbox
% 
% Also generates code which can be used directly in Maltab 
% (make sure that the variable order match).

ver = 1.0;

syms x y theta v omega t

% System model matrix
f = [
    x + ( v / omega ) * ( sin(theta + omega * t) - sin(theta) );
    y + ( v / omega ) * ( cos(theta) - cos(theta + omega *t) );
    theta + t * omega;
    v;
    omega;
];

% Compute Jacobian w.r.t state vector
F = jacobian(f, [x,y,theta,v,omega]);

% simplify computed jacobian
Fs = simplify(F);

% Compute Limits for omega -> 0
fLimit = limit(f, omega, 0);
FLimit = limit(F, omega, 0);

% write to file to enable optimization
comment = "Matlab generated code (check the docs) for symbolic expression: ";
ccode(F,'File','F.temp','Comments', comment + "F");
ccode(f,'File','f.temp','Comments', comment + "f");
ccode(FLimit,'File','FLimit.temp','Comments', comment + "FLimit");
ccode(fLimit,'File','fLimit.temp','Comments', comment + "fLimit");


