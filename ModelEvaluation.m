clear;clc;
 
%Importing model data and real setup data for a simple oscillation
%experiment
model_data = readtable("NewModelDataVd2Vn0.csv");
real_data = readtable("ParameterEvaluation.csv");
 
t = model_data.Var1;
u = model_data.Var2;
model_deflection = model_data.Var3;
model_velocity = model_data.Var4;
 
deflection = real_data.Var3;
velocity = real_data.Var4;
 
%Evaluating model based on two methods:
%1.Correlation - will give an indication on how frequencies and phase align
C_deflection = corr(model_deflection,deflection)
C_velocity = corr(model_velocity, velocity)
 
%2.RMSE - absolute error indication
RMSE_deflection = sqrt(mean((model_deflection-deflection).^2))
RMSE_velocity = sqrt(mean((model_velocity-velocity).^2))
 
%3. Will also compare other signal properties; mean & variance
[d_modelpeaks, d_modellocs] = findpeaks(model_deflection);
[v_modelpeaks, v_modellocs] = findpeaks(model_velocity);
 
[d_peaks, d_locs] = findpeaks(deflection);
[v_peaks, v_locs] = findpeaks(velocity);
 
var_deflection = var(deflection);
var_model_deflection = var(model_deflection)
mean_deflection = mean(abs(d_peaks));
mean_model_deflection = mean(abs(d_modelpeaks))
 
var_velocity = var(velocity);
var_model_velocity = var(model_velocity)
mean_velocity = mean(abs(v_peaks));
mean_model_velocity = mean(abs(v_modelpeaks))
 
%Time Domain Comparison Plots
figure
subplot(1,2,1)
plot(t, deflection, 'b-', 'LineWidth', 0.8)
hold on
plot(t, model_deflection, 'r--', 'LineWidth', 0.8)
hold off
xlabel('Time, t [s]')
ylabel('Beam deflection, \delta [m]')
legend('Measured', 'Model')
grid on
 
subplot(1,2,2)
plot(t, velocity, 'b-', 'LineWidth', 0.8)
hold on
plot(t, model_velocity, 'r--', 'LineWidth', 0.8)
hold off
xlabel('Time, t [s]')
ylabel('Beam deflection velocity, d\delta/dt [m/s]')
legend('Measured', 'Model')
grid on
