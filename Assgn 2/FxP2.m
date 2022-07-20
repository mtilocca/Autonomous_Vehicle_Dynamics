function Fplotting2 = FxP2(X,FX,K,gamma, Fz)

    % ----------------------------------------------------------------------
    %% Compute the residuals - least squares approach - to fit the Fx curve 
    %  with Fz=Fz_nom, IA=0. Pacejka 1996 Magic Formula
    % ----------------------------------------------------------------------

    % Define MF coefficients

   Fz0 = -890; 
   dfz = Fz / Fz0 -1; 
   
   pHx1 = X(1); pCx1 = X(2);
   pDx1 = X(3); pKx1 =X(4);
   pEx1 = X(5); pEx4 =X(6);
   pVx1 = X(7); pDx2 = X(8);
   pDx3 = X(9); pEx2 = X(10);
   pEx3 = 0; pKx2 = X(11); 
   pKx3 = X(12); pHx2 = X(13); 
   pVx2 = X(14);
   
   
   SHx = pHx1 + pHx2*dfz;
   Kx = K + SHx; % eq 1 
   Cx = pCx1; % eq 2 
   Mux = (pDx1+pDx2*dfz)*(1-pDx3 *gamma^2) ;
   Dx = Mux .* Fz;% eq 3
   Kxk = Fz .*(pKx1 + pKx2.*dfz).*exp(-pKx3.*dfz); % eq 4
   Ex = (pEx1 + pEx2.*dfz) .* (1-pEx4.*sign(Kx)); % eq 5 
   Bx = Kxk ./ (Cx.*Dx);
   Svx = Fz.*(pVx1+pVx2.*dfz); % eq 6 
    
   
   Fx0 = Dx.*sin(Cx.*atan(Bx.*Kx- Ex.*(Bx.*Kx-atan(Bx.*Kx))))+Svx;
   
    % Compute the residuals
  Fplotting2 = Fx0; 
    
    

end