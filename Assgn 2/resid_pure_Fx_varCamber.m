function res = resid_pure_Fx_varCamber(X,FX, K, gamma, Fz, X1, X2)

    % ----------------------------------------------------------------------
    %% Compute the residuals - least squares approach - to fit the Fx curve 
    %  with Fz=Fz_nom, IA=0. Pacejka 1996 Magic Formula
    % ----------------------------------------------------------------------

    % Define MF coefficients
   
   Fz0 = -890; 
   dfz = Fz / Fz0 -1; 
   
   pHx1 = X1(1); pCx1 = X1(2);
   pDx1 = X1(3); pKx1 =X1(4);
   pEx1 = X1(5); pEx4 =X1(6);
   pVx1 = X1(7); pDx2 = X2(1);
   pDx3 = X2(2); pEx2 = X2(3);
   pEx3 = X(1); pKx2 = X2(4); 
   pKx3 = X2(5); pHx2 = X2(6); 
   pVx2 = X2(7);
   
   
   SHx = pHx1 + pHx2*dfz;
   Kx = K + SHx; % eq 1 
   Cx = pCx1; % eq 2 
   Mux = (pDx1+pDx2*dfz)*(1-pDx3 .*gamma.^2) ;
   Dx = Mux .* Fz;% eq 3
   Kxk = Fz .*(pKx1 + pKx2.*dfz).*exp(-pKx3.*dfz); % eq 4
   Ex = (pEx1 + pEx2.*dfz) .* (1-pEx4.*sign(Kx)); % eq 5 
   Bx = Kxk ./ (Cx.*Dx);
   Svx = Fz.*(pVx1+pVx2.*dfz); % eq 6 
    
   
   Fx0 = Dx.*sin(Cx.*atan(Bx.*Kx- Ex.*(Bx.*Kx-atan(Bx.*Kx))))+Svx;
   
    % Compute the residuals
    res = sum((FX - Fx0).^2)/sum(FX.^2);

end

