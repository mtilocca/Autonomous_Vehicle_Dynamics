function Fx = PacejkaMF(X, B, C, D,  E, Sh, Sv)
x = X + Sh; 
Fx0 = D*sin(C*atan(B*x-E*(B*x-atan(B*x))));
Fx = Fx0 + Sv; 
end 



% 
