params0 = [1, 1, 2];  % Initial guess
options = optimoptions('lsqnonlin', 'Display', 'iter');  % Display iteration information
[params_opt, resnorm, residual, exitflag, output] = lsqnonlin(@myObjective, params0, [], [], options);

disp('Optimized parameters:');
disp(params_opt);