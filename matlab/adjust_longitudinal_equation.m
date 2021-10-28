function dy = adjust_longitudinal_equation(t, y, deepstall_angle, adjust_angle, adjust_time, flare_angle, adjust_time2)
    dy = zeros(6,1);
    V = y(1);
    gramma = y(2);
    q = y(3);
    theta = y(4);
    h = y(5);
    r = y(6);
    
%     disp(h);
%     if t>= 3
%         elevator_angle = flare_angle;
%     disp(adjust_angle)
%     if t >= 3
%         elevator_angle = adjust_angle;
%     elseif t >= 1
%         elevator_angle = deepstall_angle;
%     else
%         elevator_angle = 0;
%     end
    
    if h <= 1
        elevator_angle = flare_angle; % flare_angle
    elseif h <= 9
        elevator_angle = adjust_angle; % adjust_angle
    elseif h <= 10
        elevator_angle = deepstall_angle;
    else
        elevator_angle = 0;
    end
    
    
    convert_to_rad = pi/180;

    %---------input--------------------------------
    alpha0 = 20 * convert_to_rad;
    alpha = theta* convert_to_rad - gramma* convert_to_rad; % alpha > alpha0(20)
    delta = elevator_angle * convert_to_rad;
    T = 0;
    %-----------------------------------------------

    M = 50;
    CL0 = 0.062;
    CLalpha = 6.098;
    CD0 = 0.098;
    K = 0.012;
    CM0 = 0.028;
    CMalpha = -0.031; %-0.031
    CLq = 0;
    CLdelta = -1.72;
    CDq = 0;
    CDdelta = -0.814;
    CMq = -13.1;
    CMdelta = -0.325;
    Iyy = 0.388;
    m = 1.5;
    g = 9.81;
    b = 1.3;
    c = 0.31;
    S = 0.28;
    p = 1.225;

    sigma = (1 + exp(-M*(alpha-alpha0))+ exp(M*(alpha+alpha0))) / ((1+exp(-M*(alpha-alpha0))) * (1+exp(M*(alpha+alpha0))));
    CL = (1-sigma)*(CL0+CLalpha*alpha) + sigma*(2*sign(alpha)*((sin(alpha))^2)*cos(alpha));
    CD = CD0 + (1-sigma)*K*((CL0+CLalpha*alpha)^2) + sigma*(2*sign(alpha)*((sin(alpha))^3));
    CM = CM0 + CMalpha*alpha;
%     CM = M / (0.5*p*(V^2)*S*c);
  
    L = 0.5*p*S*(V^2)*(CL + ((CLq*c*q)/(2*V)) + CLdelta*delta);
    D = 0.5*p*S*(V^2)*(CD + ((CDq*c*q)/(2*V)) + CDdelta*delta);
    M = 0.5*p*S*(V^2)*c*(CM + ((CMq*c*q)/(2*V)) + CMdelta*delta);
%     Se = 0.041;
%     le = -0.5;
%     lw = 0.025;
%     Me = -0.5*p*(V^2)*Se*le*( (0.8*cos(2*alpha+2*delta)) + (1.4*sin(alpha)*(sin(alpha+delta))^2) + 0.1*sin(alpha) );
%     Mw = -lw*(L*cos(alpha) + D*sin(alpha));
%     M = Me + Mw;

 
    dy(1) = (T*cos(alpha) - D - m*g*sin(gramma)) / m;
    dy(2) = (T*sin(alpha) + L - m*g*cos(gramma)) / (m*V);
    dy(3) = M / Iyy;
    dy(4) = q;
    dy(5) = V*sin(gramma);
    dy(6) = V*cos(gramma);
end
