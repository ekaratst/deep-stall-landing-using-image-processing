Iyy = 0.388;
m = 1.5;
density_air = 1.225;
S = 0.28;
mean_chord = 0.31;
CL0 = 0.062;
CLalpha = 6.098;
CD0 = 0.098;
K = 0.012;
CM0 = 0.028;
CMalpha = -0.031;
CLq = 0;
CLdelta = -1.72;
CDq = 0;
CDdelta = -0.814;
CMq = -13.1;
CMdelta = -0.325;
alpha0 = 20*pi/180;
M = 50;
delta = 20*pi/180;

x_degrees = linspace(-10,180);
x = x_degrees.*pi./180;

sigma = ((1 + exp(-M*(x - alpha0)) + exp(M*(x + alpha0))) ./ ((1 + exp(-M*(x - alpha0))) .* (1 + exp(M*(x + alpha0)))));
CL = (1 - sigma).*(CL0 + CLalpha.*x) + sigma.*(2.*sign(x).*sin(x).^2 .* cos(x));
CLreg = CL0 + CLalpha .* x;
CLdsl = 2*sign(x).*(sin(x).^2) .* cos(x);
sin_test = sin(x).^2 ;

CD = CD0 + (1 - sigma) .* K .* (CL0 + CLalpha.*x).^2 + sigma .* (2.*sign(x).*sin(x).^3);
CDreg = CD0 + K .* (CL0 + CLalpha .* x) .^2;
CDdsl = 2*sign(x).*(sin(x).^3);
CM = CM0 + CMalpha * x;


subplot(2,1,1);
plot(x_degrees, CL);
hold on
plot(x_degrees, CLreg);
plot(x_degrees, CLdsl);
ylim([-2 2])
title('C_L - AOA')
xlabel('Angel of Attack [deg]') 
ylabel('Lift Coefficient') 
legend({'CL','CL_r_e_g','CL_D_S_L'},'Location','northeast')
hold off

subplot(2,1,2);
plot(x_degrees, CD);
hold on
plot(x_degrees, CDreg);
plot(x_degrees, CDdsl)
xlim([-10 90])
title('C_D - AOA')
xlabel('Angel of Attack [deg]') 
ylabel('Drag Coefficient') 
legend({'CD','CD_r_e_g','CD_D_S_L'},'Location','northeast')
hold off
