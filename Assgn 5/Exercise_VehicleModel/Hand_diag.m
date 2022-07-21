function Hand_diag(delta, omega, u, L)
x = omega.*u; 
y = delta - (omega./u)*L; 

figure(); 
plot(x, y, 'k'); 
grid on; 
xlabel('lateral acceleration $a_y$'); ylabel('$delta$ - $omega$/u *$L$ ');
legend('handling diagram');

end 