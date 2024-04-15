fun = @batch_analysis;
% x0 = [724.2139,  709.1211, 432.7438, 25.9539, 25.9355];
% x0 = [675.4048e+000     1.8372e+003   734.6023e+000    21.6273e+000, 22.4306e+000];
% x0 = [748.3292e+000     1.9548e+003   578.3874e+000    27.7491e+000    29.8184e+000];
% x0 = [724.8623e+000     1.8054e+003   764.3576e+000    22.4192e+000
% 22.3973e+000]; % tuned only to x
% x0 = [1.0577e+003     2.1317e+003   407.7156e+000    14.9890e+000
% 16.4253e+000]; % tuned only to y
% x0 = [725.2028e+000     2.0049e+003   773.4421e+000    22.1856e+000
% 22.2221e+000]; % tuned to both
% x0 = [550, 1750, 612, 18, 6];
%x0 = [733.4035e+000     2.0184e+003   696.2419e+000    31.4374e+000    31.7209e+000]; % tuned mar28
x0 = [819.5145e+000     2.1583e+003   208.0462e+000    26.4518e+000    35.5071e+000]; % mar28
% x0 = [218.0124e+000     2.0737e+003    59.6783e+000    22.1799e+000    21.8189e+000]; % tuned to y only
% x0 = [926.5861e+000     2.2284e+003   813.3755e+000     5.6521e+000     3.1776e+000]; % tuned to PWM highs only
x0 = [866.3385e+000     2.1871e+003   350.5445e+000    14.0045e+000    15.4994e+000]; % 75pwm tuning apr5
x0 = [868.9758e+000     2.1250e+003   337.6270e+000    14.8056e+000    17.5541e+000]; % 75pwm tuning apr8
x0 = [868.5680e+000     2.1467e+003   316.3237e+000    17.0491e+000    18.9649e+000]; % 75pwm tuning apr9
lower_bounds = [0.01, 0.01, 0.01, 0.01, 0.01];
upper_bounds = [];
options = optimset('PlotFcns',@optimplotfval);
[x,fval,exitflag,output] = fminsearchbnd(fun,x0,lower_bounds,upper_bounds,options);

%%
n = 4;
for i = 1:n

    file = strcat('fit_data', num2str(i), '.mat');
    load(file);

    noLoadCurrent = 2.1; % Amp
    noLoadSpeed = 5600; % RPM
    noLoadSpeed_rad = noLoadSpeed * 2*pi / 60; % rad/s
    Km = 0.04025; % Nm / Amp
    
    R = 0.3; % 0.19; % Ohms
    L = 188e-6; % Henrys
    
    J_X = x(1)/ 1000 / 100^2;
    J_Y = x(2)/ 1000 / 100^2;
    J_theta = x(3)/ 1000 / 100^2;
    B_X_coeff = x(4);
    B_Y_coeff = x(5);
    
    Jm = 3.5^2/4.*[J_X+J_Y J_X-J_Y; J_X-J_Y J_X+J_Y] + J_theta*eye(2);
    J_inv = inv(Jm);
    
    B_X = B_X_coeff *noLoadCurrent * Km / noLoadSpeed_rad; % Nms
    B_Y = B_Y_coeff*noLoadCurrent * Km / noLoadSpeed_rad; % Nms
    
    Bm = [B_X 0; 0 B_Y];
    
    A = zeros(6);
    A(1:2,:) = [0 0 1 0 0 0; 0 0 0 1 0 0;];
    A(3:4,:) = [zeros(2), -J_inv*Bm, J_inv*Km];
    A(5:6,:) = [zeros(2), -Km/L * eye(2), -R/L * eye(2)];
    B = [zeros(4,2); eye(2)*1/L];
    C = [eye(2) zeros(2,4)];
    D = zeros(2);
    
    % k_p = 1.0; % 15;
    % k_i = 0; %10;
    % k_d = 0.0003; %.5;
    
    Kff = minreal(inv(tf(ss(A,B,C,D)))).numerator;
    
    Kff{1,1} = Kff{1,1};
    Kff{1,2} = Kff{1,2};
    Kff{2,1} = Kff{2,1};
    Kff{2,2} = Kff{2,2};
    
    theta_rad_to_xy_cm = [1,1;1,-1]*3.5/2;

    model = 'CoreXY_model_PWM';
    simIn = Simulink.SimulationInput(model);
    simIn = setVariable(simIn,'data',data);
    simIn = setVariable(simIn, 'theta_rad_to_xy_cm',theta_rad_to_xy_cm);
    simIn = setVariable(simIn, 'A',A);
    simIn = setVariable(simIn, 'B',B);
    simIn = setVariable(simIn, 'C',C);
    simIn = setVariable(simIn, 'D',D);
    out = sim(simIn);

    f = figure('pos',[0,0,800,700]);
    subplot(2,1,1);
    % set(gcf,'visible','on','position',[850 0 800 600]);
    hold on
    
    plot(data.Time_ms_ / 1000, data.Left_PWM, '-k','LineWidth',2)
    ylim([-110 110])
    title('Input')
    ylabel('PWM (-100 to 100)')
    
    subplot(2,1,2);
    hold on
    plot(data.Time_ms_ / 1000, data.X_Velocity_cm_s_, 'ro','linest','none');
    plot(data.Time_ms_ / 1000, data.Y_Velocity_cm_s_, 'o','linest','none','color',[0 0.3 1]);
    
    plot(out.simout.Time,out.simout.Data(:,1),'linewidth',2,'color',[0.6 0 0]);
    plot(out.simout.Time,out.simout.Data(:,2),'linewidth',2,'color',[0 0 0.6]);
    
    title('Output');
    legend({'X Measured', 'Y Measured', 'X Sim', 'Y Sim'});
    xlabel("Time (s)");
    ylabel("Velocity (cm/s)");
    grid on
end