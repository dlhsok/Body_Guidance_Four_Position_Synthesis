clear
clc

%%
% 已知theta_O在O系
theta_O_double = [90/180*pi, 0/180*pi, 5/180*pi, 2/180*pi]; 
% 已知Om系原点在O系坐标
Om_O_double = [[500, -50]', [0, 000]', [210, -18]', [180, -14]'];
Mat2LaTex(theta_O_double)
Mat2LaTex(Om_O_double)
