function [cd,clmax,Vstall,K,Vcruise,Vdmin,R,E,Max,e,delta_tr,vmax] = calcs(WF,b,c_root,c_tip,S,tc,tr,c,D,lf,ch,cv,Sh,Sv,Delta,W,density,A,Wfinal,eff,fc,Pav)
%CALCS Performs drag and relevant calculations.

% Constants:
cf = 0.0043; %skin friction coefficient
M = 0.0662143; %mach number

%% For Wings
S1 = c_root * (WF/2);
beta = atan((c_root - c_tip)/(b/2)); %angle due to taper
c_fuse = c_root - tan(beta)*(WF/2);
S2 = 0.5*(c_root - c_fuse)*(WF/2); 
Sfuse = (S1 - S2); % area of wing under fuselage
Sexp = S - Sfuse; %actual exposed wing area
tau = 1; %(t/c)_tip/(t/c)_root
Qwing = 1; %interference factor
SwetW = 2*Sexp*(1+0.25*tc*((1+tau*tr)/(1+tr)));
xt = 0.15*c;
K = (1+(0.6/xt)*(tc)+100*(tc)^4)*(1.34*(M^0.18)*cos(Delta)^0.28); %form factor

cdw = (K*Qwing*cf*SwetW)/S ; %Total Drag Coeff for Wings

%% For Fuselage
QF = 1; %Interference factor for fuselage

per = pi*(3*(WF/2+D/2)- sqrt((3*WF/2+D/2)*(WF/2+3*D/2))); %crossectional perimeter
diam = per/pi; %fuselage diameter
lambda_f = lf/diam;
KF = 1+60/(lf/diam)^3+(lf/diam)/400;
SwetF = pi*diam*lf*(1-2/lambda_f)^(2/3)*(1+1/lambda_f^2);

cdF = (KF*QF*cf*SwetF)/S; %total drag coefficient for fuselage

%% For Horizontal Tail
Sfuseh = ch*(WF/2); % area of wing under fuselage
Sexph = Sh - Sfuseh; %actual exposed wing area
tau = 1; %(t/c)_tip/(t/c)_root
Qh = 1.05; %interference factor
trh = 1; %taper ratio of horizontal wing
Sweth = 2*Sexph*(1+0.25*tc*((1+tau*trh)/(1+trh)));

xth = 0.15*ch;
Deltah = 0;
Kh = (1+(0.6/xth)*(tc)+100*(tc)^4)*(1.34*(M^0.18)*cos(Deltah)^0.28); %form factor

cdh = (Kh*Qh*cf*Sweth)/S ; %Total Drag Coeff for Wings

%% For Vertical Tail

Sfusev = cv*(D/2); % area of wing under fuselage
Sexpv = Sv - Sfusev; %actual exposed wing area
tcv = 0.122; %max thickness of vert tail
tau = 1; %(t/c)_tip/(t/c)_root
Qv = 1.05; %interference factor
trv = 1; %taper ratio of vert tail
Swetv = 2*Sexpv*(1+0.25*tcv*((1+tau*trv)/(1+trv)));

xtv = 0.225*cv;
Deltav = 0;

Kv = (1+(0.6/xtv)*(tcv)+100*(tcv)^4)*(1.34*(M^0.18)*cos(Deltav)^0.28); %form factor

cdv = (Kv*Qv*cf*Swetv)/S ; %Total Drag Coeff for Wings


%% Drag Versus Velocity chart
cd = cdw + cdF + cdh + cdv; %total parasitic drag

vel = zeros(400,1);
N = length(vel);
%oswald efficiency factor approximation:
if Delta >= 30
    e = 4.61*(1 - 0.045*A^0.68)*((cosd(Delta))^0.15) - 3.1;
else
    delta_tr = 0.45*exp(-0.0375*Delta); %optimal tr
    dtr = -0.357 + delta_tr;
    f = 0.0524*dtr^4 - 0.15*dtr^3 + 0.1659*dtr^2 - 0.0706*dtr + 0.0119;
    etheo = 1/(1 + f*A); %theoretical oswald eff.
    kef = 1-2*(diam/b)^2; %fuselage correction factor
    kedo = 0.804; %parasitic drag correction factor
    kem = 1; %mach # correction factor
    e = etheo*kef*kedo*kem;
end
for i = 1:N
    vel(i) = i;
    q = (0.5)*density*(vel(i)^2)*S; % dynamic pressure
    drag(i) = q*cd; % parasitic drag
    cl(i) = W/q;
    % Ma(i) = vel(i)/1057.1733; %mach number at 15000 ft
    
    Ktemp = (1/(pi*A*e)); %form factor
    cdi(i) = (Ktemp*cl(i)^2); %induced drag coefficient
    drag_ind(i) = q*cdi(i); % induced drag
    drag_tot(i) = 1.1*drag(i) + drag_ind(i);
    cdl = 0.1*cd; %leakage/protuberance drag
    cdt(i) = cd + cdi(i) + cdl; %total drag coeeficient
    cl_cd(i) = cl(i)/(cdt(i)); %cl/cd ratio
    P(i) = drag_tot(i)*vel(i)*0.00182; %power required [hp]
    RC(i) = (Pav-P(i))*550/W; %rate of climb
    
end
vmax = 2;
for i=25:400
        vel(i) = i;
    q = (0.5)*density*(vel(i)^2)*S; % dynamic pressure
    drag(i) = q*cd; % parasitic drag
    cl(i) = W/q;
    % Ma(i) = vel(i)/1057.1733; %mach number at 15000 ft
    
    Ktemp = (1/(pi*A*e)); %form factor
    cdi(i) = (Ktemp*cl(i)^2); %induced drag coefficient
    drag_ind(i) = q*cdi(i); % induced drag
    drag_tot(i) = 1.1*drag(i) + drag_ind(i);
    cdl = 0.1*cd; %leakage/protuberance drag
    cdt(i) = cd + cdi(i) + cdl; %total drag coeeficient
    cl_cd(i) = cl(i)/(cdt(i)); %cl/cd ratio
    P(i) = drag_tot(i)*vel(i)*0.00182; %power required [hp]
    if(P(i)> 3.4 )
        vmax = i-1;
        break;
    else 
        continue;
    end
    
end

[Max,I] = max(cl_cd); %finding clmax
[Min,G] = min(drag_tot); % finding min drag conditions
[Minp,X] = min(P); %min power maxes endurance
Vdmin = vel(G);
clmax = 1.8; %based on GOE 803 airfoil
clcruise = cl(X); %to max endurance
Vstall = sqrt(2*W/(density*clmax*S)); %stall speed
Vcruise = sqrt(2*W/(density*clcruise*S)); %cruise speed
% figure (2)
% hold on
% plot(vel,drag_tot,'LineWidth',2);
% plot(vel,drag,'LineWidth',2);
% plot(vel,drag_ind,'LineWidth',2);
% legend('Total Drag','Parasitic Drag','Induced Drag');
% xlabel('Velocity [ft/s]');
% ylabel('Drag [slugs*ft/s^2]');
% axis([30 N 0 40]);
% grid on
% hold off
%% Range & Endurance
tsfc = fc/550/3600; % converting to 1/ft
R = (eff/tsfc)*(cl_cd(I))*(log(W/Wfinal)); %Proposed Range of UAV
E = (eff/tsfc)*((cl(I)^1.5)/(cdt(I)))*sqrt(2*density*S)*((Wfinal^(-0.5))-(W^(-0.5))); %Proposed Endurance of UAV

end

