function res = myObjective(params)
    % params: [a, b]
    % Example: Linear model y = a*x + b
    x = [1, 2, 3, 4];
    y_observed = [2.1, 4.2, 6.1, 8.0];
    y_predicted = params(1)*x/2 + params(2)*x + params(3);
    res = y_predicted - y_observed; % This is our error
end