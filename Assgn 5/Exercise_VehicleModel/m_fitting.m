function fitted_param = m_fitting (delta, omega, u, L)

x = omega.*u; 
y = delta - (omega./u)*L; % difference between steering angle and kinematic steering angle 

fitobj = polyfit(x,y,1); 
f1 = polyval(fitobj, x);


figure();
plot(x, y);
hold on;
xlabel('lateral acceleration $a_y$'); ylabel('$delta$ - $omega$/u *$L$ ');
grid on; title(' handling diagram '); 
plot(x, f1); hold off; legend('unfitted', 'fitted');

fitted_param = fitobj; 

end 