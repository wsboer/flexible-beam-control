clear; clc;

%Beam Measurements

Length = 0.49;
Width = 0.044;
thickness = 0.0015;
h_laser = 0.415;

E = 90000000; %May be determined experimentally by finding natural frequency (using aluminium as initial guess = 90GPa)
I = 1/12*Width*thickness^3;
K = 3*E*I/Length^3;
C = K/100; %Very small damping effect to be expected
m=0.1;
%Other Variables
h = 1e-3; %TimeStep

%Initial State-Space Model
A = [[0 1]
    [-(2*pi*5.5)^2 -2*2*pi*5.5*0.7499]];
B = [-0.07528015464
    0];
C = [[1 0]
    [0 1]]; %interested in seeing the states (deflection) ultimate goal is to create controller that minimises these
D = [0
    0];

%Set up Process and Sensor Noise signals for Kalman Filter & LQG design
%Noise variances: Disturbance & Sensor Noise

Vd = 3;
Vn = 0.01;

sys = ss(A,B,C,D)

sys.StateName = {'Deflection','Rate_of_Deflection'};

eigs = eig(sys)
nyquist(sys)

%Simulation Duration
Tsim = 10;
%Time Vector 
t = 0:h:Tsim;

%Input Vector, now with disturbance noise included
amplitude = 1;
omega = 20;
phase = 0;
u = amplitude*sin(omega*t+phase) + Vd*randn(size(t));
simin1 = [t,u];

Wn = Vn*randn(size(t));

simin2 = [t Wn];

y = lsim(sys,u,t);

combined = [t.' u.' y(:,1) y(:,2)];

%writematrix(combined, 'ModelData.csv')

figure
plot(t,y)
ylim([-0.15,0.15])
legend("Deflection","Deflection Velocity")

deflection_amplitude = max(abs(y(:,1)))

%Frequency Domain Analysis
%FFT
Fs = 1/h;
y_impulse = impulse(sys);
FFT_Model = fft(y_impulse);
N = length(FFT_Model);
f = (0:N-1)*(Fs/N);

figure
plot(f(1:floor(N/2)), abs(FFT_Model(1:floor(N/2))))

%PSD's

omega = logspace(-2,2,length(f))

mag = bode(A,B,C,D,1,omega);
mag = squeeze(mag);

Suy = mag.^2;

figure
plot(omega, Suy)

R = ctrb(A,B);
Rank_control = rank(R);
W = obsv(A,C);
Rank_observe = rank(W);
Ker_W = null(W);
%FULLY CONTROLLABLE + FULLY OBSERVABLE

%Unity feedback system:
sys_cl = feedback(sys,1,1,2);
poles_cl = pole(sys_cl)

K = place(A,B,[-0.2 + 0.1i -0.2-0.1i])

Acl = A - B*K;

Msys_cl = ss(Acl,B,C,D);

Mpoles_cl = pole(Msys_cl)


impulse(Msys_cl)




