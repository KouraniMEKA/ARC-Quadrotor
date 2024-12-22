%% ARC3_Integrated_parameters
%% 
initVars = who;

T=1/500; %sec, Simulation step time.
% Initialization_delay_t=3; %sec
% Initialization_delay_rpy=1; %sec

%% Room Dimentions, Initial Conditions:
hq = 0.3;         % CG offset from leggs
Zg = 0 + hq;      % ground height
X0 = [0 0 Zg];    % initial CG position 
V0 = [0 0 0];     % initial CG velocity 
Xmin= [-3*10 -3*10 Zg]; %Room dimensions: min borders
Xmax= [3*10 3*10 8];    %Room dimensions: max borders
Phi0   = 0*pi/180; %% this subsystem performs better, why?
Theta0 = 0*pi/180;
Psi0   = 0*pi/180;
omega0= [0 0 0];  % initial angulare rates

Ad = 0;
Wd = pi/2;
t_Fd = 10;
Fd = 1;
t_Td = 30;
Td = 1/100;

%% model parameters:
Jx = 0.03;              % moment of inertia
Jy = 0.03;              % moment of inertia
Jyaw = 0.04;           % moment of inertia (z)
I=diag([Jx,Jy,Jyaw]);    % inertia matrix
m = 1.79;              % quadcopter mass
g = 9.81;              % gravity
Tm = 1/15;              % motors time constant (sec)
K = 12;                % motors maximum thrust,each (N)
% Kw = 20;
Ky = 0.4;              % yaw moment coefficient
L = 0.2;               %arm length
Rp = 0.254/2;             % proppeller radius
Rp_e = sqrt(4*Rp^2);    % proppeller equivalant radius for ground effect
rho_ge = 2.5;           % ground effect correction factor
t0=1; % first delay
t1=3; % second delay
Sat=0.7*m*g; % saturation for initializing motors
Rate=Sat/2;  % rate for initializing motors

% Propeller: Assume that u_max results in 8000 RPM, normalize by deviding
% by 8000.
RPM_per=[0;2000;3000;4000;5000;6000;7000;8000]/8000; % RPM percentage of maximum
Thrust_per=[0;0.800;1.823;3.202;5.026;7.206;9.830;12.899]/K; %thrust percentage of maximum

% Limits
LIMIT_CMD_PITCH = 10*pi/180;
LIMIT_CMD_ROLL = 10*pi/180;
LIMIT_uCMD_HEIGHT = m*g;
LIMIT_CMD_xy = 3;


% Anti-Windup:
u_min = 0;
u_max = 4;
PSI = 100;

% path planing:
a = 1;
b = 0.5;
R = 1;


%% Gains:

% Outer Loop (Sideway) Controller
k1 = 4;
k2 = 2;

% Outer Loop (Forward/Backward) Controller
k5 = 4;
k6 = 2;

%X: 
k3 = 0.3; %0.5 %2.8
% to compansate for the delays and filters increase this:
k4 = 30;  %2   % 18 
%Y:
k7 = 0.3;
k8 = 30;

%Z:
k9 = 2; %2
k10 = 2; %0.5

%Psi:
k11 = 5; %10
k12 = 5; %20

% Second order filter parameters for generating rate data
OMEGA_DIFF = 60;            % (rad/s) - For generating rate data
OMEGA_FILTER = 60;          % (rad/s) - For smoothing data
OMEGA_FILTER_OUTER = 40;    % (rad/s) - For smoothing optitrack x and z data

A_1df = [1/40 1];                   % filter for 1 derivative
A_2df = conv([1/40 1],[1/60 1]);    % filter for 2 derivatives

%% Coupeling:
yaw_control_ON=1; %zero or one, enable maneuvre with yaw angle ~= 0

%% Adaptive Control Parameters:
m_add = 0.2*0.5; %kg
m_add_time=27; %40; %sec
rx = 0.1;
ry = 0.1;
rz = -0.1;

I_xx_add =  m_add*(ry^2+rz^2);
I_yy_add =  m_add*(rx^2+rz^2);
I_zz_add =  m_add*(rx^2+ry^2);
I_xy_add = -m_add*rx*ry;
I_xz_add = -m_add*rx*rz;
I_yz_add = -m_add*ry*rz;
I_add = [I_xx_add I_xy_add I_xz_add
         I_xy_add I_yy_add I_yz_add
         I_xz_add I_yz_add I_zz_add]*1; % kg.m2  [0.005 0.005 0]
I_add_time=m_add_time;

% Tx (y-direction motion), Ty (x-direction motion) and Tz:
T_add_x = -ry*m_add*g;
T_add_y =  rx*m_add*g;

T_add = [T_add_x T_add_y 0]*1; % (N) inside the brackets, (N.m) overall: [0.1*0*L 0.5*0*L 0]
T_add_time=m_add_time; %40; %sec

u_hover_mod=0.605;
u_hover_exp=0.580;
d0=4*K*(u_hover_mod-u_hover_exp)/m;

d1=T_add(1,2)/Jx; % (m/s2), y-direction motion
d2=T_add(1,1)/Jy; % (m/s2), x-direction motion
d3=(u_hover_mod^2-u_hover_exp^2)*K/m;%;
d4 = 0;

% Initialize parameters:
theta1 = [K*L/Jy d1]; % X-motion
theta2 = [K*L/Jx d2]; % Y-motion
theta3 = [K/m 0.75*0]; % from simulation test: d3=0.73, not the estimated value
theta4 = [Ky/Jyaw 0];
thetax = 0;
thetay = 0;
theta1_0 = 0.9*theta1.*[1 0];
theta2_0 = 0.9*theta2.*[1 0];
theta3_0 = 0.9*[theta3(1) theta3(2)*0];
theta4_0 = 1.1*theta4;
thetax_0 = 0;
thetay_0 = 0;
% parametric uncertainties bounds:
theta1_Max = [(1.2*K*L)/(Jy) abs(d1)+5]; %0.8
theta2_Max = [(1.2*K*L)/(Jx) abs(d2)+5];
theta3_Max = [(1.2*K)/(m) 5*d3*0+10^-5]; % 1.2
theta4_Max = [1.2*Ky/Jyaw 0.5];
thetax_Max = 2;
thetay_Max = 2;

theta1_Min = [(0.8*K*L)/(2*Jy) -(abs(d1)+5)]; %1.2
theta2_Min = [(0.8*K*L)/(2*Jx) -(abs(d2)+5)];
theta3_Min = [(0.8*K)/(1.2*m) -5*d3*0-10^-5];%0.9*d3
theta4_Min = [0.8*Ky/Jyaw -0.5];
thetax_Min = -2;
thetay_Min = -2;

% Adaptation gains:
Gamma1 = diag([0.5*10^5 0]);
Gamma2 = diag([10^5 0]);
Gamma3 = diag([3 0.03*0]);
Gamma4 = diag([300 3]);
Gammax = 4;
%% IARC:

%---------------- Z ---------------:
%adjust mu values to compensate for the uncertainties frequencies.
mu1_z=0.9; % forgetting factor: 0<mu1<1, mu1=0 (no forgetting)
mu2_z=0.3; % 0<mu2<2

alpha_z=1; % alpha higher for faster convergence
nu_z=0.9; % nu smaller for faster convergence, keep it less than 1

gamma_z_0=diag([2 0]);% [5 0.2; 0.2 0.01]; % Positive definite
Rho_M_z=[20 10 ; 10  10^-5]; % upper bound for the covariance matrix Gamma
Rho_L_z=[1 -10 ; -10 -10^-5]; % Lower bound for the covariance matrix Gamma
theta_dot_M_z=[50 10]; % rate limite for updating the estimated parameter

[~,D] = eig(Rho_M_z);
v_z=D(:,1); %D(:,2)

Tau_z=1/50; % filter time constant

% fast dynamics
dc_M_z=0.3;
gamma_d_z=0.4;

%---------------- X---------------:

%adjust mu values to compensate for the uncertainties frequencies.
mu1_x=0.6; % forgetting factor: 0<mu1<1, mu1=0 (no forgetting)
mu2_x=0.3; % 0<mu2<2

alpha_x=1; % alpha higher for faster convergence
nu_x=0.9; % nu smaller for faster convergence, keep it less than 1

gamma_x_0 = 1500;% [5 0.2; 0.2 0.01]; % Positive definite
Rho_M_x = 5000; % upper bound for the covariance matrix Gamma
Rho_L_x = 1000; % Lower bound for the covariance matrix Gamma
theta_dot_M_x=50; % rate limite for updating the estimated parameter

[~,D] = eig(Rho_M_x);
v_x=D(:,1); %D(:,2)

Tau_x=1/50; % filter time constant

% fast dynamics
dc_M_x=0.3;
gamma_d_x=0.4;


%----------- Roll-Pitch-----------:

%adjust mu values to compensate for the uncertainties frequencies.
mu1_rp=0.98; % forgetting factor: 0<mu1<1, mu1=0 (no forgetting)
mu2_rp=2; % 0<mu2<2

alpha_rp=0.98;
nu_rp=0.09;

gamma_rp_0=diag([0.5*10^5 1.5*10^3]);% [10^5 15000; 15000 700]; % Positive definite
Rho_M_rp=[1.5*gamma_rp_0(1,1),1*10^5;1*10^5,2*10^3]; % upper bound for the covariance matrix Gamma
Rho_L_rp=[1000,-1*10^5;-1*10^5,200]; % Lower bound for the covariance matrix Gamma

theta_dot_M_rp=[30 9]; % rate limite for updating the estimated parameter
[~,D] = eig(Rho_M_rp);
v_rp=D(:,1); %D(:,2)

Tau_rp=1/30; % filter time constant
% fast dynamics
dc_M_rp=0.003;
gamma_d_rp=1*1;

%-------------- Yaw --------------:

%adjust mu values to compensate for the uncertainties frequencies.
mu1_yaw=0.9; % forgetting factor: 0<mu1<1, mu1=0 (no forgetting)
mu2_yaw=0.5; % 0<mu2<2
alpha3=0.9;
nu3=0.5;
gamma4_0=diag([0.5*10^5 9000]);% [5 0.2; 0.2 0.01]; % Positive definite
Rho_M_yaw=[5*10^5,1*10^4;1*10^4,10000]; % upper bound for the covariance matrix Gamma
Rho_L_yaw=[10^4,-1*10^4;-1*10^4,8000]; % Lower bound for the covariance matrix Gamma
theta_dot_M_yaw=[0.5 1]; % rate limite for updating the estimated parameter

[~,D] = eig(Rho_M_yaw);
v_yaw=D(:,1); %D(:,2)

Tau_yaw=1/30; % filter time constant

%% Robust Control Parameters:

delta1=0;
delta2=0;
delta3=0.1*4*K/m;
delta4=0;

Epsilon1 = 15; %0.00001
Epsilon2 = 15;
Epsilon3 = 10;
Epsilon4 = 0.05;

% error_bound_10=sqrt(Epsilon3/k10);

%%
% uam=m*g;
% K_eq=k10+uam^2/(4*Epsilon3);
% zeta_d=10;
% Ki=K_eq^2/(4*zeta_d^2);
% w_d=sqrt(Ki);
% W=diag(theta3_Max-theta3_Min);
% phi=[uam;1];
% S=phi'*W.^2*phi;
% gamma=K_eq^2/(4*zeta_d^2*S);
% Gamma3=gamma*W.^2;



%% Filters

% Notch:
% an1=[0.337540151883547,-0.310253311913592,0.337540151883547];
% bn1=[1,-0.310253311913592,-0.324919696232906];
% an2=[0.327054913984048,0.404148045226953,0.327054913984048];
% bn2=[1,0.404148045226953,-0.345890172031905];
% an3=[0.420807779837732,0.806706851924986,0.420807779837732];
% bn3=[1,0.806706851924986,-0.158384440324536];

%
an1=[0.337540151883547,-0.271994199011296,0.337540151883547];
bn1=[1,-0.271994199011296,-0.324919696232906];
an2=[0.327054913984048,0.404148045226953,0.327054913984048];
bn2=[1,0.404148045226953,-0.345890172031904];
an3=[0.420807779837732,0.806706851924986,0.420807779837732];
bn3=[1,0.806706851924986,-0.158384440324536];

an1_v=[0.4208 -0.8097 0.4208];
bn1_v=[1 -0.8097 -0.1584];

%% Custom Variables
% Add your variables here:
% myvariable = 0;

% Register variables after the project is loaded and store the variables in
% initVars so they can be cleared later on the project shutdown.
endVars = who;
initVars = setdiff(endVars,initVars);
clear endVars;

% sim('ARC_1_ODE1')
