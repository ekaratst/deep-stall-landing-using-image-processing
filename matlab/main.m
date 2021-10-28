convert_to_degree = 180/pi;
deepstall_angle = -25;
flare_angle = -15;
% adjust_angle = 3;
adjust_time = 1;
adjust_time2 = 3; %12

t_span = [0 15];      %Time period [0 15]
Y_init = [   0       % Velocity(V)
             0       % Angle of descent(gramma) -30*pi/180
             0       % Pitch rate(q)
             0       % Pitch angle(theta) 20*pi/180
             15       % height(h)
             0];     % horizontal distance(r)

% figure(1)
% for i = 1:4
%     vel = [8,12,16,20];
%     Y_init(1) = vel(i);
%     [T1,Y1] = ode45(@(t,y) longitudinal_equation(t,y,deepstall_angle),t_span,Y_init);  
%     plot(Y1(:,6),Y1(:,5)); 
%     hold on 
% end
% ylim([0 22])
% xlabel('Horizontal Distance [m]') 
% ylabel('Vetical Distance [m]') 
% title("Simulated DSL Trajectory [\delta = -45]")
% legend('8 m/s', '12 m/s', '16 m/s', '20 m/s')
% hold off         

%---------------------use------------------------
% figure(1)
% for i = 1:4
%     vel = [8,12,16,20];
%     Y_init(1) = vel(i);
%     [T1,Y1] = ode45(@(t,y) adjust_longitudinal_equation(t,y,deepstall_angle,adjust_angle,adjust_time),t_span,Y_init);  
%     plot(Y1(:,6),Y1(:,5)); 
%     hold on 
%     
% end
% ylim([0 22])
% xlabel('Horizontal Distance [m]') 
% ylabel('Vetical Distance [m]') 
% title("Simulated DSL Trajectory [\delta = -30, \delta2 = " + adjust_angle + " at t = " + adjust_time + "]")
% legend('8 m/s', '12 m/s', '16 m/s', '20 m/s')
% hold off
%---------------------use------------------------

figure(1)
for i = 1:8
    adjust_angle_list = [3,0,-5,-10,-15,-20,-25,-30];
    adjust_angle = adjust_angle_list(i);
    Y_init(1) = 8;
    [T1,Y1] = ode45(@(t,y) adjust_longitudinal_equation(t, y, deepstall_angle, adjust_angle, adjust_time, flare_angle, adjust_time2), t_span, Y_init);  
    plot(Y1(:,6),Y1(:,5)); 
    hold on 
    
end
ylim([0 22])
xlabel('Horizontal Distance [m]') 
ylabel('Vertical Distance [m]') 
title("Simulated DSL Trajectory 8 m/s [\delta = -30, at t = " + adjust_time + "]")
%title("Simulated DSL Trajectory 8 m/s [CMalpha = -0.031]")
%title("Simulated DSL Trajectory 8 m/s [CMalpha = 0]")
legend('3\circ', '0\circ', '-5\circ', '-10\circ', '-15\circ', '-20\circ', '-25\circ', '-30\circ')
hold off

% figure(2)
% for i = 1:8
%     adjust_angle_list = [3,0,-5,-10,-15,-20,-25,-30];
%     adjust_angle = adjust_angle_list(i);
%     Y_init(1) = 8; %
%     [T1,Y1] = ode45(@(t,y) adjust_longitudinal_equation(t, y, deepstall_angle, adjust_angle, adjust_time, flare_angle, adjust_time2), t_span, Y_init);  
%     plot(Y1(:,6),Y1(:,4)*convert_to_degree); 
%     hold on 
%     
% end
% % xlim([0 40])
% xlabel('Horizontal Distance [m]') 
% ylabel('Pitch angle [deg]') 
% title("Pitch angle [\delta = -30, at t = " + adjust_time + "]")
% %title("Pitch angle [CMalpha = -0.031]")
% %title("Pitch angle [CMalpha = 0]")
% legend('3\circ', '0\circ', '-5\circ', '-10\circ', '-15\circ', '-20\circ', '-25\circ', '-30\circ')
% hold off
% 
% figure(3)
% for i = 1:8
%     adjust_angle_list = [3,0,-5,-10,-15,-20,-25,-30];
%     adjust_angle = adjust_angle_list(i);
%     Y_init(1) = 8; %
%     [T1,Y1] = ode45(@(t,y) adjust_longitudinal_equation(t, y, deepstall_angle, adjust_angle, adjust_time, flare_angle, adjust_time2), t_span, Y_init);  
%     plot(Y1(:,6),(Y1(:,4)-Y1(:,2))*convert_to_degree); 
%     hold on 
%     
% end
% % xlim([0 40])
% xlabel('Horizontal Distance [m]') 
% ylabel('Alpha [deg]') 
% title("Pitch angle [\delta = -30, at t = " + adjust_time + "]")
% %title("Alpha [CMalpha = -0.031]")
% %title("Alpha [CMalpha = 0]")
% legend('3\circ', '0\circ', '-5\circ', '-10\circ', '-15\circ', '-20\circ', '-25\circ', '-30\circ')
% hold off

% figure(3)
% for i = 1:8
%     adjust_angle_list = [3,0,-5,-10,-15,-20,-25,-30];
%     adjust_angle = adjust_angle_list(i);
%     Y_init(1) = 8;
%     [T1,Y1] = ode45(@(t,y) adjust_longitudinal_equation(t, y, deepstall_angle, adjust_angle, adjust_time, flare_angle, adjust_time2), t_span, Y_init);  
%     plot(T1,Y1(:,5)); 
%     hold on 
%     
% end
% % xlim([0 40])
% xlabel('time [sec]') 
% ylabel('Vetical Distance [m]') 
% title("Height [\delta = -30, at t = " + adjust_time + "]")
% legend('3\circ', '0\circ', '-5\circ', '-10\circ', '-15\circ', '-20\circ', '-25\circ', '-30\circ')
% hold off

% figure(3)
% for i = 1:4
%     vel = [8,12,16,20];
%     Y_init(1) = vel(i);
%     [T1,Y1] = ode45(@(t,y) longitudinal_equation(t,y,deepstall_angle),t_span,Y_init);  
%     plot(T1,Y1(:,2)*convert_to_degree); 
%     hold on 
% end
% xlabel('Time[sec]') 
% ylabel('Gramma') 
% title("Gramma")
% legend('8 m/s', '12 m/s', '16 m/s', '20 m/s')
% hold off

%{
figure(3)
for i = 1:4
    vel = [8,12,16,20];
    Y_init(1) = vel(i);
    [T1,Y1] = ode45(@(t,y) longitudinal_equation(t,y,deepstall_angle),t_span,Y_init);  
    plot(T1,Y1(:,4)*convert_to_degree); 
    hold on 
end
xlabel('Time[sec]') 
ylabel('Theta[degree]') 
title("Theta [\delta = -45]")
legend('8 m/s', '12 m/s', '16 m/s', '20 m/s')
hold off

figure(4)
for i = 1:4
    vel = [8,12,16,20];
    Y_init(1) = vel(i);
    [T1,Y1] = ode45(@(t,y) adjust_longitudinal_equation(t,y,deepstall_angle,adjust_angle,adjust_time),t_span,Y_init);  
    plot(T1,Y1(:,4)*convert_to_degree); 
    hold on 
end
xlabel('Time[sec]') 
ylabel('Theta[degree]') 
title("Theta [\delta = -45, \delta2 = " + adjust_angle + " at t = " + adjust_time + "]")
legend('8 m/s', '12 m/s', '16 m/s', '20 m/s')
hold off
%}

% figure(2)
% for i = 1:4
%     vel = [8,12,16,20];
%     Y_init(1) = vel(i);
%     [T1,Y1] = ode45(@(t,y) adjust_longitudinal_equation(t,y,deepstall_angle,adjust_angle,adjust_time),t_span,Y_init);
%     plot(Y1(:,6),Y1(:,4)*convert_to_degree); 
%     hold on 
% end
% xlabel('Horizontal Distance [m]') 
% ylabel('Pitch angle[degree]') 
% title("theta")
% legend('8 m/s', '12 m/s', '16 m/s', '20 m/s')
% hold off

%{
figure(6)
for i = 1:4
    vel = [8,12,16,20];
    Y_init(1) = vel(i);
    [T1,Y1] = ode45(@(t,y) adjust_longitudinal_equation(t,y,deepstall_angle,adjust_angle,adjust_time),t_span,Y_init);  
    plot(T1,Y1(:,2)*convert_to_degree); 
    hold on 
end
xlabel('Time[sec]') 
ylabel('Angle of descent[degree]') 
title("Angle of descent [\delta = -45, \delta2 = " + adjust_angle + " at t = " + adjust_time + "]")
legend('8 m/s', '12 m/s', '16 m/s', '20 m/s')
hold off


% 
figure(7)
for i = 1:4
    vel = [8,12,16,20];
    Y_init(1) = vel(i);
    [T1,Y1] = ode45(@(t,y) longitudinal_equation(t,y,deepstall_angle),t_span,Y_init);  
    plot(T1,Y1(:,1)); 
    hold on 
end
xlabel('Time[sec]') 
ylabel('V[m/s]') 
title("Velocity [\delta = -45]")
legend('8 m/s', '12 m/s', '16 m/s', '20 m/s')
hold off

figure(8)
for i = 1:4
    vel = [8,12,16,20];
    Y_init(1) = vel(i);
    [T1,Y1] = ode45(@(t,y) adjust_longitudinal_equation(t,y,deepstall_angle,adjust_angle,adjust_time),t_span,Y_init);  
    plot(T1,Y1(:,1)); 
    hold on 
end
xlabel('Time[sec]') 
ylabel('V[m/s]') 
title("Velocity [\delta = -45, \delta2 = " + adjust_angle + " at t = " + adjust_time + "]")
legend('8 m/s', '12 m/s', '16 m/s', '20 m/s')
hold off
%}
% figure(5)
% for i = 1:4
%     vel = [8,12,16,20];
%     Y_init(1) = vel(i);
%     [T1,Y1] = ode45(@(t,y) longitudinal_equation(t,y,deepstall_angle),t_span,Y_init);  
%     plot(T1,Y1(:,2)/pi*180); 
%     hold on 
% end
% xlabel('Time[sec]') 
% ylabel('\gamma [\circ]') 
% title("Angle of Descent [\delta = -45]")
% legend('8 m/s', '12 m/s', '16 m/s', '20 m/s')
% hold off
% 
% figure(6)
% for i = 1:4
%     vel = [8,12,16,20];
%     Y_init(1) = vel(i);
%     [T1,Y1] = ode45(@(t,y) adjust_longitudinal_equation(t,y,deepstall_angle,adjust_angle,adjust_time),t_span,Y_init);  
%     plot(T1,Y1(:,2)/pi*180); 
%     hold on 
% end
% xlabel('Time[sec]') 
% ylabel('\gamma [\circ]') 
% title("Adjusted Angle of Descent [\delta = -45 -> -15]")
% legend('8 m/s', '12 m/s', '16 m/s', '20 m/s')
% hold off


%------------------------------------------------------------------------------------
% figure(3)
% for i = 1:4
%     vel = [8,12,16,20];
%     Y_init(1) = vel(i);
%     [T1,Y1] = ode45(@(t,y) longitudinal_equation(t,y,deepstall_angle),t_span,Y_init);  
%     plot(T1,Y1(:,2)*convert_to_degree); 
%     hold on 
% end
% xlabel('Time[sec]') 
% ylabel('\gamma') 
% title("Simulated DSL Angle of descent(\gamma) [\delta = -45]")
% legend('8 m/s', '12 m/s', '16 m/s', '20 m/s')
% hold off
% 
% figure(4)
% for i = 1:7
%     deepstall_angle = [-15, -25, -35, -45, -55, -65, -75];
%     Y_init(1) = 15;
%     [T1,Y1] = ode45(@(t,y) longitudinal_equation(t,y,deepstall_angle(i)),t_span,Y_init);  
%     plot(Y1(:,6),Y1(:,5)); 
%     hold on 
% end
% ylim([0 20])
% xlabel('Horizontal Distance [m]') 
% ylabel('Vetical Distance [m]') 
% title("Simulated DSL Trajectory [Each Elevator Angle]")
% legend('-15', '-25', '-35', '-45', '-55', '-65', '-75')
% hold off




% figure(5)
% Y_init(1) = 15;
% if T1 >=0
%     deepstall_angle = -45;
%     [T1,Y1] = ode45(@(t,y) longitudinal_equation(t,y,deepstall_angle(i)),t_span,Y_init);  
% else
%     deepstall_angle = -65;
%     [T1,Y1] = ode45(@(t,y) longitudinal_equation(t,y,deepstall_angle(i)),t_span,Y_init);  
% end
% plot(Y1(:,6),Y1(:,5)); 
% 
% deepstall_angle = [0, 1, 2.5, 5, 5.2, 10];
% p7vals = [pi/9, 0.3, exp(-1), sqrt(2), -3];  %one fewer
% num_interval = length(p7vals);
% t = cell(num_interval,1);
% x = cell(num_interval, 1);
% for idx = 1 : num_interval
%     p7 = p7vals(idx);
%     [t{idx},x{idx}] = ode45(@(t,x) try_eqns(t, x, p7), deepstall_angle(idx:idx+1), x0);
%     x0 = x{idx}(end,:);
% end
% 



