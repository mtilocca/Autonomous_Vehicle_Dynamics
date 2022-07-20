function Y = PCJMF(Dxa, Cxa, k, alpha, Shxa, Fx0)
Bxa = 23.308 ./(sqrt((19.414.*k).^2+1));
Gxa = Dxa *cos(Cxa*atan(Bxa.*(alpha+Shxa)));
Fx2 = Gxa.*Fx0; 
Y = Fx2;
end 