function [cg,cgwing,hn,hcg,sm,sm1,a_w,ep,a_t,hachtemp] = CG(W,Ww,Wau,Wf,Wfs,Wfu,Wht,Wvt,Wlg,Wp,Wpl,Wsc,c,ch,cv,space,Delta,tr,c_root,c_tip,lf,hac,hach,hacv,lh,b,Sh,S,e,A,bh)
% CG Function gets center of gravity of whole UAV
%   Gets cg of each part and then uses arms and weights to calcualte
%   overall cg
    %% Getting moment arms(individual cg loaction)
    %% CG for wing
    if tr == 1 && Delta == 0
        cgwing = c/2; %
    elseif tr ~= 1 && Delta == 0
        cgwing = (c_root^2 +(c_root*c_tip)+c_tip^2)/(3*(c_root+c_tip)); %from wing LE
    elseif tr == 1 && Delta ~= 0
        % Todo: Fix this (up to line 18=still good)
        phi = 90 - Delta*180/pi; %angle of corners in parallelogram
        b1 = c*cosd(phi); %part of parallelogram length to make into rectangle
        h = c*sind(phi); % height of parallelogram
        a = (b/2)*acos(Delta);%length of parallelogram
        cgx = 0.5*(a+b1); %for parallelogram
        cgy = 0.5*h; %for parallelogram
        a2 = h*a; %area of parallelogram
        h1 = (cgx)*sin(Delta);
        h2 = (cgy)*sin(Delta- (pi/2));
        cgwing = h1+h2;
    elseif tr ~=1 && Delta ~= 0
        % TODO: double check
        % split wing into triangle + parallelogram to get cg
        phi = 90 - Delta*180/pi; %angle of corners in parallelogram
        b1 = c_tip*cosd(phi); %part of parallelogram length to make into rectangle
        h = c_tip*sind(phi); % height of parallelogram
        a = ((b/2)*acos(Delta)) - b1;%length of parallelogram
        cgx = 0.5*(a+b1); %for parallelogram
        cgy = 0.5*h; %for parallelogram
        a2 = h*a; %area of parallelogram
        h1 = (cgx)*sin(Delta);
        h2 = (cgy)/cos(Delta);
        
        p1 = (c_root - c_tip); %triangle side, y-dir
        x3 = cos(Delta)*(a+b1); %x-dir of tip of wing
        y3 = sin(Delta)*(a+b1); %y-dir of tip of wing
        cg1 = (x3)/3; %cg in x-dir
        cg2 = (p1 + y3)/3; %cg needed in y-dir
        a1 = p1*(a+b1)*sind(phi)/2; %area of triangle
        
        ay = a1*(cg2) +a2*(h1+h2);
        cgwing = ay/(a1+a2);
    end
    
    armwing = space + cgwing;
    armfuse = lf/2; % assume symmetric fuselage
    armhtail = space + hac*c + lh - hach*ch +(ch/2); %assumed rectangle for now
    armvtail = space + hac*c + lh - hacv*cv +(0.3994*cv); %using naca 4 digit cg for now
    % armsc = space + hac*c +lh/2; %surface controls
    armsc1 = space + 0.5*c; %arm for surface controls on wing
    armsc2 = space + lh; %arm for surface controls on tail
    
    armlg = 1; %no landing gear rn, this number is irrelevant
    
    %% cg of engine system
    armps = -(4/12); %assuming prop engine in front, 8 in long
    armfs = (3/12); %fuel sys
    
    armfu = armfs+(13.5/12); %cg of fuel
    
    
    %% cg of avionics and payload broken down
    x10 = armfs+ (1/12); %cg of battery (fits next to camera)
    W10 = 1.2;
    mom10 = x10 * W10;
    
    x2 = x10+(2.5/12); %cg of video transmitter
    W2 = 0.32628376; %weight of video transmitter
    mom2 = x2*W2;
    
    x3 = x2 ; %cg of AHRS, fits side by side with video transmitter
    W3 = 0.10361714; %weight of AHRS
    mom3 = x3*W3;
 
    x4 = x3 + (2.5/12) ; %cg of altimeter
    W4 = 0.661386; %weight of altimeter
    mom4 = x4*W4;
    
    x9 = x4 + (2/12); %cg of ecu
    W9 = 0.1322772;
    mom9 = x9*W9;
    


    
   
    x1 = armfs + (9/12); %cg of camera
    W1 = 4.3431014; %weight of camera
    mom1 = x1*W1;

    
    %% cg of auto control system
    x5 = x1 + (1/12); %cg of cpu
    W5 = 160*0.00220462; %weight of cpu in lb
    mom5 = x5*W5;
    x6 = x5; %cg of otu, stacked ontop/side of cpu
    W6 = 120*0.00220462; %weight of otu in lb
    mom6 = x6*W6;
    x7 = 1.5; %cg of pitot, located outside fuselage
    W7 = 74*0.00220462; %weight of pitot in lb
    mom7 = x7*W7;
    x8 = x6+(1/12); %cg of gpsm
    W8 = 49*0.00220462; %weight of gpsm in lb
    mom8 = x8*W8;
    
    %% Multiply weights by arms for all mass to calculate moments.
    momwing = Ww*armwing; %wing
    momfuse = Wf*armfuse; %fuselage
    momhtail = Wht*armhtail; %horizontal tail
    momvtail = Wvt*armvtail; %vertical tail
    momlg = Wlg*armlg; %
    momps = Wp*armps; %engine
    momfs = Wfs*armfs; %fuel system
    
    %% Extreme laoding testing
    
    momfu = Wfu*armfu;
    momsc1 = 0.4*Wsc*armsc1; %for servos on wing
    momsc2 = 0.6*Wsc*armsc2; %for servos on tail
    
    momavi = (mom3+mom4+mom5+mom6+mom7+mom8+mom9+mom10); %added battery
    mompl = (mom1+mom2);
    
    tot_mom = momwing+momfuse+momhtail+momvtail+momlg+momps+momfs+momfu+momsc1+momsc2+momavi+mompl;
    tot_mom1 = momwing+momfuse+momhtail+momvtail+momlg+momps+momfs+momsc1+momsc2+momavi+mompl;
    
    cg = tot_mom/W; %actual cg point
    cg1 = tot_mom1/(W-Wfu); % no fuel
    
    %% Getting Neutral point
    
    aw = ((1.23-0.35)/5)*(180/pi);  %wing lift curve slope --> from airfoils
    at = ((1.23-0.35)/5)*(180/pi);  %Tail Lift curve slope --> from airfoils
    a_t = at/(1 + at/(pi*(bh*bh/Sh)*e));  %3d Tail Lift curve slope
    a_w = aw/(1 + aw/(pi*A*e)); %lift-curve slope of 3d wing
    %ep = (43.8/8.68);  %Downwash angle
    ep = (2*a_w)/(pi*A*e); %Downwash angle
    const = (Sh/S)*(a_t/a_w)*(1-ep); %constant in eqn
    hachtemp = (space + hac*c + lh + hach*ch)/c;
    hn = (hac + hachtemp*const)/(1 + const);
    hcg = (cg - space)/c;
    sm = abs(hn - hcg); %static margin at max W_TO
    hcg1 = (cg1 - space)/c;
    sm1 = abs(hn - hcg1); %static margin without fuel
end

