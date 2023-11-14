%% 程序说明：
%          本程序中的O系指的是和机架固连的坐标系，Om系指的是和连杆固连的坐标系
%          Pm_Om指的是连杆平面中的点在Om中的坐标
%          Pm_O指的是连杆平面中的点在O中的坐标
%          以此类推，点的下标代表其所处的坐标系

%% 描述连杆的运动（原点和转角）
clear all
clc
clf

% 已知theta_O在O系
theta_O_double = [90/180*pi, 0/180*pi, 5/180*pi, 2/180*pi]'; 
% 已知Om系原点在O系坐标
Om_O_double = [[500, -50]', [0, 000]', [210, -18]', [180, -14]'];

[n1, ~] = size(theta_O_double);

%% 在O系绘制各Om系xy轴，单位长度
vec_origin = [0, 0, 1]';
vec_unit_x = [100, 0, 1]';
vec_unit_y = [0, 100, 1]';
subplot(3, 3, 1);
hold on
for i = 1:n1
    vec_origin_plot = M_O_Om(theta_O_double(i), Om_O_double(:, i)) * vec_origin;
    vec_unit_x_plot = M_O_Om(theta_O_double(i), Om_O_double(:, i)) * vec_unit_x;
    vec_unit_y_plot = M_O_Om(theta_O_double(i), Om_O_double(:, i)) * vec_unit_y;
    tmp_x = [vec_origin_plot(1), vec_unit_x_plot(1)];
    tmp_y = [vec_origin_plot(2), vec_unit_x_plot(2)];
    plot(tmp_x, tmp_y, 'r');
    axis equal
    grid on
    tmp_x = [vec_origin_plot(1), vec_unit_y_plot(1)];
    tmp_y = [vec_origin_plot(2), vec_unit_y_plot(2)];
    plot(tmp_x, tmp_y, 'g');
    txt = num2str(i);
    text(vec_origin_plot(1),vec_origin_plot(2), append('Om', txt),'FontSize',10);
end
hold off

%% 坐标变换
% Pm在Om系坐标
% Pm_Om = [[1.6, 1.7]',  [1.5, 1.2]'];
syms Pm_Om [2 1]
% assume(Pm_Om<ones(2, 1)*100);

% Pm在O系坐标
syms Pm_O [2 n1]
% assume(Pm_O<ones(2, n)*100);

% 将Pm在Om系里的坐标转换至O系里
syms tmp
for i = 1:n1
    tmp = M_O_Om(theta_O_double(i), Om_O_double(:, i))*[Pm_Om;1];
     Pm_O(:, i) = tmp(1:2);
end

%% 解方程

% 圆点半径平方
syms radius_sq
assume(radius_sq, 'positive')

% 圆心在O系位置
syms Circle_Center_O [2 1]

% 先用前三个方程消掉Circle_Center_O和radius_sq，得到只含有圆点坐标的解s
eqn = [(Pm_O(1,1)-Circle_Center_O1)^2+(Pm_O(2,1)-Circle_Center_O2)^2 - radius_sq;
           (Pm_O(1,2)-Circle_Center_O1)^2+(Pm_O(2,2)-Circle_Center_O2)^2 - radius_sq;
           (Pm_O(1,3)-Circle_Center_O1)^2+(Pm_O(2,3)-Circle_Center_O2)^2 - radius_sq;
           (Pm_O(1,4)-Circle_Center_O1)^2+(Pm_O(2,4)-Circle_Center_O2)^2 - radius_sq;
           ];
       
% eq4 = (Pm_O(1,4)-Circle_Center_O(1))^2+(Pm_O(2,4)-Circle_Center_O(2))^2 == radius^2;
s1 = solve(eqn(1:3), [Circle_Center_O(1), Circle_Center_O(2), radius_sq]);

% 将s代入第四个方程
eqn_left = subs(eqn(4), {Circle_Center_O1, Circle_Center_O2, radius_sq}, {s1(1).Circle_Center_O1, s1(1).Circle_Center_O2, s1(1).radius_sq});
[n2, ~] = size(eqn_left);

% 化简第四个方程
for i = 1:n2
    eqn_left(i) = collect(eqn_left(i), [Pm_Om1, Pm_Om2]);
end

color_list_line_rgbcmyk = ['r', 'g', 'b', 'c', 'm', 'y', 'k'];

%% 画圆点曲线
% 提取第四方程左侧表达式分子分母，并令其=0绘图
[num, den] = numden(eqn_left);
for i = 1:n2
    subplot(3, 3, 4);
    h=ezplot(char(num(i)), [-1000, 1000]); % 分子=0
    hold on
    set(h,'color',color_list_line_rgbcmyk(i) ,'LineWidth',1)
    title('Burmester Cruve')
    grid on
    h=ezplot(char(den(i)), [-1000, 1000]); % 分母=0
    set(h,'color',color_list_line_rgbcmyk(i+2) ,'LineWidth',1)
end

%% 画圆点

p_c_Om_double = [0; 200]; % 圆点在连杆系坐标（手动修改）

% 选定一个y坐标
num_x =  vpa(solve(subs(num(1), Pm_Om2, p_c_Om_double(2))));
p_c_Om_double(1) = num_x(1);
subplot(3, 3, 4);
hold on
plot(p_c_Om_double(1), p_c_Om_double(2), 'b*');
hold off

subplot(3, 3, 1);
for j = 1:n1
    p_c_O_double(:, j) = M_O_Om(theta_O_double(j), Om_O_double(:, j)) * double([p_c_Om_double; 1]);
    txt = num2str(j);
    txt = append(', ', txt);
    txt = append(num2str(i), txt);
    hold on
    plot(p_c_O_double(1, j), p_c_O_double(2, j), 'ro');
    text(p_c_O_double(1, j), p_c_O_double(2, j), txt, 'FontSize', 10);
end

%% 解方程

% 滑点所在直线参数
syms alpha lambda
assume(lambda, 'positive')

% 共线方程
eqn = [(Pm_O(1,1)+lambda*sin(alpha))*sin(alpha)-(Pm_O(2,1)-lambda*cos(alpha))*cos(alpha);
           (Pm_O(1,2)+lambda*sin(alpha))*sin(alpha)-(Pm_O(2,2)-lambda*cos(alpha))*cos(alpha);
           (Pm_O(1,3)+lambda*sin(alpha))*sin(alpha)-(Pm_O(2,3)-lambda*cos(alpha))*cos(alpha);
           (Pm_O(1,4)+lambda*sin(alpha))*sin(alpha)-(Pm_O(2,4)-lambda*cos(alpha))*cos(alpha);];

% 消去alpha，lambda
s1 = solve(eqn(1), eqn(2), [alpha, lambda]);

% 将s1代入剩余个方程
eqn_left = [subs(eqn(3),{alpha, lambda}, {s1.alpha(1), s1.lambda(1)});
                 subs(eqn(4),{alpha, lambda}, {s1.alpha(1), s1.lambda(1)});];
             
[n2, ~] = size(eqn_left);

% 化简
for i = 1:n2
    eqn_left(i) = collect(eqn_left(i), [Pm_Om1, Pm_Om2]);
end

color_list_line_rgbcmyk = ['r', 'g', 'b', 'c', 'm', 'y', 'k'];

%% 画滑点曲线
for i = 1:n2
    subplot(3, 3, 7);
    h=ezplot(char(eqn_left(i)), [-1000, 1000]); % 分子=0
    hold on
    set(h,'color',color_list_line_rgbcmyk(i+1) ,'LineWidth',1)
end

%% 画滑点曲线交点
s2 = solve(eqn_left(1:2), [Pm_Om1, Pm_Om2]);
p_l_Om_double = [];
[n3, ~] = size(s2.Pm_Om1);

for i = 1:n3
    re = real(double(s2.Pm_Om1(i)^2+s2.Pm_Om2(i)^2));
    im = imag(double(s2.Pm_Om1(i)^2+s2.Pm_Om2(i)^2));
    if abs(re) < 10000 && im == 0
        subplot(3, 3, 7);
        txt = num2str(i);
        hold on
        plot(s2.Pm_Om1(i), s2.Pm_Om2(i), 'b*');       
        p_l_Om_double = double([p_l_Om_double, [s2.Pm_Om1(i);s2.Pm_Om2(i)]]);% 滑点在连杆系坐标
        text(s2.Pm_Om1(i), s2.Pm_Om2(i), txt, 'FontSize',14);
        subplot(3, 3, 1);
        for j = 1:n1
            p_l_O_double(:, j) = M_O_Om(theta_O_double(j), Om_O_double(:, j)) * double([s2.Pm_Om1(i);s2.Pm_Om2(i); 1]);
            txt = num2str(j);
            txt = append(', ', txt);
            txt = append(num2str(i), txt);
            hold on
            plot(p_l_O_double(1, j), p_l_O_double(2, j), 'b+');
            text(p_l_O_double(1, j), p_l_O_double(2, j), txt, 'FontSize', 10);
        end
    end
end

hold off
subplot(3, 3, 7);
grid on
title('滑点曲线')
subplot(3, 3, 4);
title('圆点曲线')
%% 保存工作区到 Synthesis.mat
save('Synthesis.mat', 'p_l_O_double', 'p_c_O_double', 'theta_O_double', 'Om_O_double', 'p_l_Om_double', 'p_c_Om_double');
