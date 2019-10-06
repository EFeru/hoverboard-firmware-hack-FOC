%% Parameters for Electric Engine Dyno Example

% This example shows how to model an electric vehicle dynamometer test. 
% The test environment contains an asynchronous machine (ASM) and an 
% interior permanent magnet synchronous machine (IPMSM) connected back-
% to-back through a mechanical shaft. Both machines are fed by high-
% voltage batteries through controlled three-phase converters. The 164 kW 
% ASM produces the load torque. The 35 kW IPMSM is the electric machine 
% under test. The Control Machine Under Test (IPMSM) subsystem controls the 
% torque of the IPMSM. The controller includes a multi-rate PI-based 
% control structure. The rate of the open-loop torque control is slower 
% than the rate of the closed-loop current control. The task scheduling 
% for the controller is implemented as a Stateflow(TM) state machine. The 
% Control Load Machine (ASM) subsystem uses a single rate to control the 
% speed of the ASM. The Visualization subsystem contains scopes that 
% allow you to see the simulation results.

% Copyright 2016-2017 The MathWorks, Inc.

%% Machine Parameters
Pmax = 35000;      % Maximum power                        [W]
Tmax = 205;        % Maximum torque                       [N*m]
Ld   = 0.00024368; % Stator d-axis inductance             [H]
Lq   = 0.00029758; % Stator q-axis inductance             [H]
L0   = 0.00012184; % Stator zero-sequence inductance      [H]
Rs   = 0.010087;   % Stator resistance per phase          [Ohm]
psim = 0.04366;    % Permanent magnet flux linkage        [Wb]
p    = 8;          % Number of pole pairs

%% High-Voltage Battery Parameters
Cdc  = 0.001;      % DC-link capacitor [F]
Vnom = 325;        % Nominal DC voltage[V] 
V1   = 300;        % Voltage V1(< Vnom)[V]

%% PMSM Control Parameters
Ts   = 1e-5;       % Fundamental sample time               [s]
fsw  = 10e3;       % PMSM drive switching frequency        [Hz]
Tsi  = 1e-4;       % Sample time for current control loops [s]

Kp_id = 0.8779;     % Proportional gain id controller
Ki_id = 710.3004;   % Integrator gain id controller
Kp_iq = 1.0744;     % Proportional gain iq controller
Ki_iq = 1.0615e+03; % Integrator gain iq controller

%% Zero-Cancellation Transfer Functions
numd_id = Tsi/(Kp_id/Ki_id);
dend_id = [1 (Tsi-(Kp_id/Ki_id))/(Kp_id/Ki_id)];
numd_iq = Tsi/(Kp_iq/Ki_iq);
dend_iq = [1 (Tsi-(Kp_iq/Ki_iq))/(Kp_iq/Ki_iq)];

%% Current References
load pe_ipmsm_35kW_ref_idq;

%% ASM Parameters
Pn  = 164e3;  % Nominal power                    [W]
Vn  = 550;    % rms phase-to-phase rated voltage [V]
fn  = 60;     % Rated frequency                  [Hz]

Rs2 = 0.0139; % Stator resistance                                     [pu]
Lls = 0.0672; % Stator leakage inductance                             [pu]
Rr  = 0.0112; % Rotor resistance, referred to the stator side         [pu]
Llr = 0.0672; % Rotor leakage inductance, referred to the stator side [pu]
Lm  = 2.717;  % Magnetizing inductance                                [pu]
Lr  = Llr+Lm; % Rotor inductance                                      [pu]
Ls  = Lls+Lm; % Stator inductance                                     [pu]
p2  = 2;      % Number of pole pairs                                  [pu]

Vbase = Vn/sqrt(3)*sqrt(2); % Base voltage, peak, line-to-neutral [V]
Ibase = Pn/(1.5*Vbase);     % Base current, peak                  [A]
Zbase = Vbase/Ibase;        % Base resistance                     [Ohm]
wbase = 2*pi*fn;            % Base electrical radial frequency    [rad/s]
Tbase = Pn/(wbase/p2);      % Base torque                         [N*m]

Rss = Rs2*Zbase; % Stator resistance                                   [Ohm]
Xls = Lls*Zbase; % Stator leakage reactance                            [Ohm]
Rrr = Rr*Zbase;  % Rotor resistance, referred to the stator side       [Ohm]
Xlr = Llr*Zbase; % Rotor leakage reactance, referred to the stator side[Ohm]
Xm  = Lm*Zbase;  % Magnetizing reactance                               [Ohm]

%% ASM Control Parameters
fsw2 = 2e3;        % ASM drive switching frequency [Hz]
Tsc  = 1/(fsw2*5); % ASM control sample time       [s]

% ASM PI parameters
Kp_ids = 1.08;
Ki_ids = 207.58;
Kp_imr = 52.22;
Ki_imr = 2790.51;
Kp_iqs = 1.08;
Ki_iqs = 210.02;
Kp_wr  = 10;
Ki_wr  = 100;

%% Coupling Parameters
Jm   = 0.1234;     % Inertia             [Kg*m^2]
ce   = 25;         % Damping coefficient [N*m/(rad/s)]