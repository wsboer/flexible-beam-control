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
C = [[0.6 0]
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
Tsim = 15;
%Time Vector 
t = 0:h:Tsim;

%Input Vector, now with disturbance noise included
amplitude = 1;
omega = 20;
phase = 0;
u_cycle = amplitude*sin(omega*t+phase);

Tpulse = 0.05;
v=30/Tpulse;
u_step = zeros(size(t));

u_step(t >= 1 & t < 1 + Tpulse*0.75) = v;

u_step(t >= 2 & t < 2 + Tpulse*0.75) = -v;

u_step(t >= 4 & t < 4 + Tpulse) = 1.5*v;

u_step(t >= 8 & t < 8 + Tpulse) = -v;

u_step(t >= 11 & t < 11 + Tpulse) = v;


u = u_step;%Choose which input to use 

%u = amplitude*sin(omega*t+phase);
d = Vd*randn(size(t));
simin = [t.' u.'];
simin1 = [t.' d.'];

Wn = Vn*randn(size(t));

simin2 = [t.' Wn.'];

y = lsim(sys,u,t);

combined = [t.' u.' y(:,1) y(:,2)];

%writematrix(combined, 'ModelData.csv')
VD = Vd*eye(2);

[L,P,E] = lqe(A,VD,C,VD*0.01,Vn*eye(2));

Kalmansys = ss(A-L*C,[B L], eye(2), 0*[B L])

Ak = A-L*C;
Bk = [B L];
Ck = eye(2);
Dk = 0*[B L];

Q = [[40 0]
    [0 80]];

R = [[0.1]];

[Klqr,S,P] = lqr(sys,Q,R)

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

bode(A,B,C,D,1,omega)
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




