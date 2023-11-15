%% 程序说明：
%          Kinematics.m 主要是基于位置综合的结果进行机构尺度计算以及运动参量可视化
%          本程序中的O系指的是和机架固连的坐标系，Om系指的是和连杆固连的坐标系
%          Pm_Om指的是连杆平面中的点在Om中的坐标
%          Pm_O指的是连杆平面中的点在O中的坐标
%          以此类推，点的下标代表其所处的坐标系

%          形如abc_val的变量是由syms变量abc为纯数值的时候赋值而来的

%          1号杆坐标原点位于圆心点，x正方向沿着杆远离圆心点
%          2号杆坐标系就是位置综合的时候的连杆坐标系

%% 运行位置综合
Body_Guidance_Four_Position_Synthesis;

%% 清除
clear all
clc

%% 导入位置综合的圆点等数据
load('Synthesis.mat');

%% 计算圆点圆参数
syms xc_O yc_O radius_sq  % 圆心点坐标、圆半径平方
assume(radius_sq, 'positive')
eqn = [];
for i = 1:3
    eqn = [eqn; (p_c_O_double(1, i)-xc_O)^2+(p_c_O_double(2, i)-yc_O)^2 - radius_sq];
end
s = solve(eqn);
xc_O_val = s.xc_O;
yc_O_val = s.yc_O;
r_val = sqrt(vpa(s.radius_sq));




%% 计算滑点直线参数
syms lambda alpha  % 直线距圆点的距离、直线倾角
assume(lambda, 'positive')

eqn = [];
for i = 1:2
    eqn = [eqn; (p_l_O_double(1, i)+lambda*sin(alpha))*sin(alpha)-(p_l_O_double(2, i)-lambda*cos(alpha))*cos(alpha)];
end
s = solve(eqn);
lambda_O_val = s.lambda;
alpha_O_val = s.alpha;

%%
syms plot_helper_x plot_helper_y

subplot(1, 3, 3);
hold on
lambda_O_double = double(lambda_O_val);
alpha_O_double = double(alpha_O_val);
line_eq = (plot_helper_x+lambda_O_double*sin(alpha_O_double))*sin(alpha_O_double)-(plot_helper_y-lambda_O_double*cos(alpha_O_double))*cos(alpha_O_double);
% vpa(circle_eq, 2)
h = ezplot(char(line_eq),[-600, 600]);
set(h,'color','k' ,'LineWidth',1)
% plot(xc_O_val, yc_O_val, 'b*');
text(xc_O_val, yc_O_val, '圆心点');

circle_eq = (plot_helper_x - double(xc_O_val))^2+(plot_helper_y - double(yc_O_val))^2 - double(r_val)^2
% vpa(circle_eq, 2)
h = ezplot(char(circle_eq),[-500, 500]);
set(h,'color','k' ,'LineWidth',1)
hold off

%% 将O系里的圆点、滑点变换到Of系里
M_O_Of = M_O_Om(alpha_O_val, [xc_O_val, yc_O_val]);
[~, n] = size(p_c_O_double);
for i = 1:n
    p_c_Of_double(:, i) = double(M_O_Of\p_c_O_double(:, i));
end
[~, n] = size(p_l_O_double);
for i = 1:n
    p_l_Of_double(:, i) = double(M_O_Of\p_l_O_double(:, i));
end
% 
% %% 计算圆点圆参数
% syms xc_Of yc_Of radius_sq  % 圆心点坐标、圆半径平方
% assume(radius_sq, 'positive')
% eqn = [];
% for i = 1:3
%     eqn = [eqn; (p_c_Of_double(1, i)-xc_Of)^2+(p_c_Of_double(2, i)-yc_Of)^2 - radius_sq];
% end
% s = solve(eqn);
% xc_Of_val = s.xc_Of;
% yc_Of_val = s.yc_Of;
% r_val = sqrt(vpa(s.radius_sq));

% %% 计算滑点直线参数
% syms lambda alpha  % 直线距圆点的距离、直线倾角
% assume(lambda, 'positive')
% 
% eqn = [];
% for i = 1:2
%     eqn = [eqn; (p_l_Of_double(1, i)+lambda*sin(alpha))*sin(alpha)-(p_l_Of_double(2, i)-lambda*cos(alpha))*cos(alpha)];
% end
% s = solve(eqn);
% lambda_Of_val = s.lambda;
% alpha_Of_val = s.alpha;

%% 构建机构运动约束
p0_line_Of = [p_l_Of_double(1, 2); p_l_Of_double(2, 2)]; % 滑点直线参数方程起始点

syms t_line_Of % 滑点直线的参数方程自变量，数值上等于从参数方程起始点到当前点的距离
syms theta_circle_1_Of % 1号杆机架系Of转角（圆点的机架系转角）
syms theta_circle_2_Of % 2号杆机架系Of转角（位置综合时连杆系的转角）

l1 = double(r_val);
l2 = sqrt((p_l_Om_double(1)-p_c_Om_double(1))^2+(p_l_Om_double(2)-p_c_Om_double(2))^2);
d = p0_line_Of(2);
eqn = [l1*cos(theta_circle_1_Of) - l2*cos(theta_circle_2_Of)-(t_line_Of+p0_line_Of(1));
           l1*sin(theta_circle_1_Of) - l2*sin(theta_circle_2_Of)-d; ];
vpa(eqn,5)
s1 = solve(eqn, [theta_circle_2_Of, t_line_Of]);

%%
% syms x2_Of y2_Of % 2号杆原点
% M_Of_2 = M_O_Om(theta_circle_2_Of, [x2_Of, y2_Of]); % 从2号杆坐标系转换到机架系Of
% 
% p_l_Of= M_Of_2 * [p_l_Om_double; 1]; % 滑点在Of系坐标
% p_c_Of = M_Of_2 * [p_c_Om_double; 1]; % 圆点在Of系坐标
% % double(subs(p_c_O, [t_line_O, theta_circle_2_O], [0, 0]))
% 
% eqn = [eq_circle_Of - p_c_Of(1:2);% 共点约束1：圆点在Of系坐标 与 1号杆末端坐标 共点
%            eq_line_Of - p_l_Of(1:2)]; % 共点约束2：滑点 与 滑点直线机架系Of参数方程自变量为t_line_Of对应点 共点
% 
% s1 = solve(eqn(3:4), [x2_Of, y2_Of]);
% vpa(subs(eqn(1:2),[x2_Of, y2_Of], [s1.x2_Of, s1.y2_Of]), 2)
% 
% s2 = solve(subs(eqn(1),[x2_Of, y2_Of], [s1.x2_Of, s1.y2_Of]), [t_line_Of])
% s3 = solve(subs(eqn(2),[x2_Of, y2_Of], [s1.x2_Of, s1.y2_Of]), [t_line_Of])
% vpa(s2-s3, 2)
% s6 = solve(s2-s3, theta_circle_2_Of, 'ReturnConditions', true);
% 
% s4 = solve(subs(eqn(1),[x2_Of, y2_Of], [s1.x2_Of, s1.y2_Of]), [theta_circle_2_Of], 'ReturnConditions', true)
% s5 = solve(subs(eqn(2),[x2_Of, y2_Of], [s1.x2_Of, s1.y2_Of]), [theta_circle_2_Of], 'ReturnConditions', true)
% % s2 = solve(eqn(2), [t_line_O]);

%% 画图

syms time_t
omega_l1 = 10/180*pi;  % 杆1角速度[rad/s]
% syms omega_l1(time_t)
% 将ω1*time_tt代入解中，得到位移关于时间的方程
% theta_circle_1_Of = time_t*omega_l1;
theta_circle_2_Of = subs(s1.theta_circle_2_Of(1), theta_circle_1_Of, time_t*omega_l1);
t_line_Of = subs(s1.t_line_Of(1), theta_circle_1_Of, time_t*omega_l1);
theta_circle_1_Of = subs(theta_circle_1_Of, theta_circle_1_Of, time_t*omega_l1);

% 对time_t求导，得到角速度
d1_theta_circle_2_Of = diff(theta_circle_2_Of, time_t)
d1_t_line_Of = diff(t_line_Of, time_t)

% 对time_t求导，得到角加速度
d2_theta_circle_2_Of = diff(d1_theta_circle_2_Of, time_t)
d2_t_line_Of = diff(d1_t_line_Of, time_t)

subplot(3, 3, 2);
grid on
time_t_double = .1:.11:10;
yyaxis left
plot(time_t_double, double(subs(theta_circle_2_Of, time_t, time_t_double)), 'b');
ylabel("杆2转角θ_2 [rad]")
% subplot(3, 3, 2);
yyaxis right
plot(time_t_double, double(subs(t_line_Of, time_t, time_t_double)), 'r');
ylabel("滑块沿杆移动距离l [mm]")
xlabel("时间t [s]")

subplot(3, 3, 5);
grid on
yyaxis left
plot(time_t_double, double(subs(d1_theta_circle_2_Of, time_t, time_t_double)), 'b');
ylabel("杆2转角ω_2 [rad/s]")
% subplot(3, 3, 5);
yyaxis right
plot(time_t_double, double(subs(d1_t_line_Of, time_t, time_t_double)), 'r');
ylabel("滑块沿杆移动速度v  [mm/s]")
xlabel("时间t [s]")

subplot(3, 3, 8);
grid on
yyaxis left
plot(time_t_double, double(subs(d2_theta_circle_2_Of, time_t, time_t_double)), 'b');
ylabel("杆2转角α_2 [rad/s^2]")
% subplot(3, 3, 8);
yyaxis right
plot(time_t_double, double(subs(d2_t_line_Of, time_t, time_t_double)), 'r');
ylabel("滑块沿杆移动加速度a [mm/s^2]")
xlabel("时间t [s]")

%%
subplot(1, 3, 3);
title("机构")

%% 瞬心法验证
subplot(3, 3, 1);
grid on
yyaxis right
plot(time_t_double, double(subs(d1_t_line_Of / omega_l1, time_t, time_t_double)), 'b');  % vc/ω1
ylabel("vc/ω_1 [mm]")
yyaxis left
plot(time_t_double, double(subs(l1*sin(theta_circle_2_Of-theta_circle_1_Of)/cos(theta_circle_2_Of), time_t, time_t_double)), 'r');
ylabel("AP13 [mm]")
xlabel("时间t [s]")
%% 公式导出
syms alpha_O_latex xc_O_latex yc_O_latex
M_O_Of = M_O_Om(alpha_O_latex, [xc_O_latex, yc_O_latex]);
latex(vpa(M_O_Of,2))
% ezplot(char(s1.theta_circle_2_Of(1)),[-.1, 2], [-.2, .5*pi]);

% subplot(2, 2, 2);
% ezplot(char(s2));