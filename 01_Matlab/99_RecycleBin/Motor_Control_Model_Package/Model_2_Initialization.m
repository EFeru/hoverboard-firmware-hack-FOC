%% Copyright 2018 The MathWorks, Inc.%%
clear all;
bdclose all;
clc;
%% PMSM parameters
Ld = 1.7e-3;
Lq = 3.2e-3;
Rs = 0.02;
Lambda_m = 0.2205;
Polepairs = 4;

%% Current Loop PI Controller parameters

Gain_P = 7.74;
Gain_I = 26.84;

%% Speed Loop PI Controller Parameters
Speed_Gain_P = 0.1;
Speed_Gain_I = 0.1;


