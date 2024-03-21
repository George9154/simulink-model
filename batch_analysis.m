function [rms_error] = batch_analysis(x)

n = 1;
errors = zeros(1,n);
for i=1:n
    errors(i) = CoreXY_analysis_func(x, strcat('fit_data', num2str(i), '.mat'));
end
    rms_error = rms(errors);
end

