function y = posPart(x)

    % Regularized positive part
    regFactor = 0.01;
    y = x*(tanh(x/regFactor)+1)/2;

end