function Z = Ga(Dxa, Cxa, k, alpha, Shxa)
Bxa = 23.308 ./(sqrt((19.414.*k).^2+1));
Gxa = Dxa *cos(Cxa*atan(Bxa*(alpha+Shxa))); 
Z = Gxa;
end 


% G = D cos(C arctan(Bx)),