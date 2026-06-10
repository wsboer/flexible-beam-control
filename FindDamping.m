%Import Data
data = readtable('StepDampingData4.csv');
noise_data = readtable("NoiseMeasurement.csv");
h = 1e-3;
Fs = 1/h;

t = data.Var1;
u = data.Var2;
x_laser = data.Var4;
x_actuator = data.Var3;

x_laser_noise = -(noise_data.Var4-noise_data.Var4(1));
x_actuator_noise = noise_data.Var3 - noise_data.Var3(1);

deflection_noise = x_laser_noise - x_actuator_noise;

%Processing
%Set measurements to zero
x_laser = -(x_laser - x_laser(1));
x_actuator = x_actuator - x_actuator(1);

deflection = x_laser - x_actuator;

%Find Damping Coeff - 2 methods - Settling time & Log decrement
damp_data = abs(deflection(1538:end));
damp_data = damp_data - max(deflection_noise);
t_damp = t(1538:end);
%Method 1
threshold = 0.10*max(damp_data);
t_settling = 0.0;
for i=1:(length(damp_data)-180)
    if max(damp_data(i:i+180)) <= threshold
        t_settling = t_damp(i)
        break
    end
end

t_settling

%Method 2: We must use valid monotonically decreasing pairs
[pks,locs] = findpeaks(damp_data);
deltas = zeros(size(pks));
for i=1:(length(pks)-1)
    if pks(i) > pks(i+1)
        delta_i = log(abs(pks(i)/pks(i+1)));
        deltas(i) = delta_i;
    end
end

deltas = deltas(deltas ~= 0);
delta = mean(deltas);
delta = abs(delta)


ksi = delta/(sqrt(4*pi^2 + delta^2))


%Time Domain Plots
figure
subplot(1,2,1)
plot(t,x_laser,t,x_actuator)
legend("Cantilever Position", "Cart Position")
subplot(1,2,2)
plot(t, deflection)



%FFT

FFT_Chirp = fft(deflection);
N = length(FFT_Chirp);
f = (0:N-1)*(Fs/N);

figure
plot(f(1:floor(N/2)), abs(FFT_Chirp(1:floor(N/2))))
xlabel('Frequency (Hz)')
ylabel('|Y(f)|')
grid on

%PSD

figure
plot(f(1:floor(N/2)), 1/N*abs(FFT_Chirp(1:floor(N/2))).^2)
xlabel('Frequency (Hz)')
ylabel('1/N|Y(f)|^2')
grid on

