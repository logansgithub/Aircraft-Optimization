function [incidence,CL_adot,CD0,CD_a,CD_de,Cm0,Cm_adot] = stabderiv(a_w,a_t,S,Sh,W,density,hachtemp,c,ep,hcg,hn,Vstallbest,vmaxbest,A,Vcruise,e,cd)
%stabderiv Calculates stability derivatives needed for autopilot


CMac = -0.2049434738; %thin airfoil theory estimation (check)
lt = (hachtemp-hcg) * c; %distance between cg and ac of horizontal tail
Vh = lt/c * Sh/S; %tail-volume ratio
Cli = -a_t * Sh/S; %lift curve slope of tail wrt incidence angle
C_mi = a_t*Vh; 
Cl_alpha = a_w + a_t * Sh/S * (1-ep); %lift curve slope of wing
Cm_alpha = Cl_alpha * (hcg-hn);

%% getting incidence trim angle
V_incAng = floor(Vstallbest):ceil(vmaxbest);
incAng = zeros(1,length(V_incAng));
for j = 1:length(V_incAng)
    Cl = 2*W / (density*V_incAng(j)^2*S); %lift coeff for given speed and weight
    % incidence of tail for trimmed flight
    i = -(CMac*Cl_alpha + Cm_alpha*Cl) / (Cl_alpha*C_mi - Cm_alpha*Cli);
    incAng(j) = i;
end
Clinc = 2*W / (density*Vcruise^2*S); %lift coeff for given speed and weight
    % incidence of tail for trimmed flight
incidence = -(CMac*Cl_alpha + Cm_alpha*Clinc) / (Cl_alpha*C_mi - Cm_alpha*Cli);

figure
plot(V_incAng, incAng, 'linewidth', 2)
%axis([floor(Vstall) ceil(vmaxbest) 0 6])
xlabel('Velocity [ft/s]')
ylabel('Inicidence for Trimmed Flight [deg]')

%% getting stability derivatives
downwash = (2*Cl)/(pi*A); %epsilon
Vh1 = Vcruise*cosd(downwash); %velocity at horizontal tail
eta = (0.5*density*Sh*Vh1^2)/(0.5*density*S*Vcruise^2); %tail efficiency factor (Qt/Q), 1 for t-tail config
tau = (1/e) - 1;

 
% aero data - linear stability coefficients
CL0     =    0.375;              % []      from airfoil
CL_a    =    5.02;   % [/rad]      Bray pg 33
CL_adot =    -2*eta*Vh*a_t*ep;               % [/rad]      Bray pg 33
CL_q    =    0.35;               % [/rad]      Bray pg 33
CL_de   =    -0.33;              % [/rad]      Bray pg 33

CD0     =    cd;              % []          Bray pg 33
CD_a    =    2*Cl/(pi*e*A)*CL_a;              % [/rad]      Bray pg 33
CD_de   =    2*Cl/(pi*e*A)*CL_de;             % [/rad]      Bray pg 33

Cm0     =   CMac + (Vh*a_t*i);              % []          Bray pg 33
Cm_a    =   2.78;               % [/rad]      Bray pg 33
Cm_adot =  -2*CL_a*eta*Vh*lt/c*ep;                % [/rad]      Bray pg 33
Cm_q    =  -2.84;                % [/rad]      Bray pg 33
Cm_de   =   -0.007;               % [/rad]      Bray pg 33


% optional?? - lat-dir derivatives
CY_beta =   -0.306;              % [/rad]      Bray pg 33 -
CY_dr   =    0.191;              % [/rad]      Bray pg 33 -

Cl_beta =   -0.008;              % [/rad]      Bray pg 33 
Cl_p    =   -0.475;              % [/rad]      Bray pg 33
Cl_r    =    0.124;              % [/rad]      Bray pg 33
Cl_da   =   -0.161;              % [/rad]      Bray pg 33 (sign reversed)
Cl_dr   =   -0.00229;            % [/rad]      Bray pg 33

Cn_beta =    0.033;              % [/rad]      Bray pg 33
Cn_p    =   -0.068;              % [/rad]      Bray pg 33
Cn_r    =   -0.013;              % [/rad]      Bray pg 33
Cn_da   =    0.0200;             % [/rad]      Bray pg 33 (sign reversed)
Cn_dr   =   -0.0917;             % [/rad]      Bray pg 33



end

