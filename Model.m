clear; clc;

%Beam Measurements

Length = 0.49;
Width = 0.044;
t = 0.0015;
h_laser = 0.415;

E = 90000000; %May be determined experimentally by finding natural frequency (using aluminium as initial guess = 90GPa)
I = 1/12*Width*t^3;
K = 3*E*I/Length^3;
C = K/100; %Very small damping effect to be expected
m=0.1;
%Other Variables
h = 1e-3; %TimeStep

%Initial State-Space Model
A = [[0 1]
    [-5.5^2 -C/m]];
B = [-1
    0];
C = [[1 0]
    [0 1]]; %interested in seeing the states (deflection) ultimate goal is to create controller that minimises these
D = [0
    0];

sys = ss(A,B,C,D)

sys.StateName = {'Deflection','Rate_of_Deflection'};

eigs = eig(sys)
nyquist(sys)

%Simulation Duration
Tsim = 10;
%Time Vector 
t = 0:h:Tsim;
%Input Vector
amplitude = 1;
omega = 20;
phase = 0;
u = amplitude*sin(omega*t+phase);

y = lsim(sys,u,t);

figure
plot(t,y)
legend("Deflection","Deflection Velocity")

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

%K = place(A,B,[-0.2 + 0.1i -0.2-0.1i])

%Acl = A - B*K;

%Msys_cl = ss(Acl,B,C,D);

%Mpoles_cl = pole(Msys_cl)


%impulse(Msys_cl)


