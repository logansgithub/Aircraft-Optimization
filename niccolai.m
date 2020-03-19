function [W,Ww,Wau,Weng,Wf,WF,Wfs,Wfu,Wht,Wvt,Wlg,Wp,Wpl,Wsc,Delta,tr,S,lf,lh,tc,D,N] = niccolai(W,A,Sh,Sv,bh,bv,hac,c,ch,cv,hach,hacv,space)
%niccolai gets weight estimates of UAV
%   uses niccolai equations to get weight estimates of different parts of
%   the UAV including sensors and landing gear.

    Wto(1) = W;     %Inserted this to plot initial guess
    density = .001496; %altitude of 15000
    velocity = 73; %scaneagle velocity
    
    for i = 2:15
        Wto(i) = W;

        
        C_l = 1.2; %Estimation

        S = W/(.5*density*velocity^2 * C_l );
        
        
        %% Component Weight Estimates- Nicolai
        %% Wing Weight

        N = 13;              %Changed to 5 as said in lecture
                          %Ultimate Load Factor (1.5 times limit load factor)(GIVEN)
        Delta = rand()*30*pi/180; %Deg %Wing 1/4 chord sweep angle
        tr=0.2+ .8*rand();               %Taper Ratio
        %tr=0.4485; %optimum tr
        tc=0.063;            %Maximum Thickness Ratio (GIVEN)
        Ve=80;%kts         %Equivalent Vmax at SL

        Ww=96.948*((W*N/10^5)^0.65*(A/cos(Delta))^0.57*(S/100)^0.61*((1+tr)/(2*tc))^0.36*(1+Ve/500)^0.5)^0.993;

        %% Fuselage Weight
        lf = space + c + ch + 6*rand();
        %lf = 6+rand()*10; %ft       %Fuselage Length
        WF = 0.5+rand()*2; %ft        %Fuselage Width
        D = 0.5+rand()*2; %ft          %Fuselage Max Depth

        Wf=200*((W*N/10^5)^0.286*(lf/10)^0.857*((WF+D)/10)*(Ve/100)^0.338)^1.1;

        %% Horizontal Tail Weight
        lh = lf - space - (hac*c) - ((1-hach)*(ch)) ; 
       % lh= 35/12 + (.5 - hac) * c - (.5 - hach) * ch; %ft       %Distance from Wing MAC to Tail MAC
        thr=ch*tc*12; %in?      %horizontal tail max root thickness (chord * thick/chord)

        Wht=127*((W*N/10^5)^0.87*(Sh/100)^1.2*(lh/10)^0.483*(bh/thr)^0.5)^0.458;

        %% Vertical Tail Weight

        tvr=cv*.12*12; %in    %Vertical Tail Max Root Thickness (chord * thick/chord * in/ft)

        Wvt= (2)*  98.5*((W*N/10^5)^0.87*(  (.5)*  Sv/100)^1.2*(  (.5)*  bv/tvr)^0.5)^0.458;

        %% Landing Gear Weight

        %Llg=18; %in     %Length of Main Landing Gear Strut
        %Nland=2;        %Ultimate Load Factor at Wland
        %Wlg=0.054*(Llg)^0.501*(W*Nland)^0.684;

        %don't need niccolai if we have specific landing gear 
        %Wlg = 1.4;  % use this if we fing appropriate landing gear

         Wlg = 0; %chose no landing gear
        %% TOTAL STRUCTURAL WEIGHT

        Wstruct=Ww+Wf+Wht+Wvt+Wlg;


        %% Total Propulsion Unit (minus Fuel system) Weight

        Weng = 4.850164;
        %Weng=4.4; %(lbs)     %Bare Engine Weight
        Neng=1;             %# Engines

        %Wp=2.575*(Weng)^0.922*Neng;    %this equation likely over-estimates propulsion unit weight for a small UAV
        Wp = 5.26683718; %engine + propeller+fuel pump and sensors of known engine system

        %% Fuel Weight
        %Wfu = 4.32; % for first engine selection
        Wfu = 8;   %(lbs)  
        %% Fuel System Weight

         %rhof = 6.739; %lb/gal fuel mass density JP-8
         rhof = 6.25905; %lb/gal for RON 95+ fuel
         Fg = Wfu / rhof; %gal               %Total Fuel
         tankint=1; %percent         %Percent of Fuel Tanks that are integral
         Nt=1;                         %Number of Separate Fuel Tanks
         Wfs=2.49*((Fg)^0.6*(1/(1+tankint))^0.3*Nt^0.2*Neng^0.13)^1.21;

        % specific fuel system weights (fuel tanks, lines) likely can be found for your aircraft, if so, use those actual values instead of the niccolai equations.
        %Wfs = .66;

        %% Surface Controls Weight

       % Wsc=1.066*W^0.626; 
        Wsc = 5*(1.1464); %actual actuators that we're using

        %% Avionics Weight - use weights of specific sensors you choose

        Wau=1.66889734;  %sensor weight       


        %% Payload Weight

        Wpl= 4.6693; %weight of camera, transmitter, battery

        %% TOTAL WEIGHT

        Wto(i)=Wstruct+Wp+Wfs+Wsc+Wpl+Wfu;
        W = Wto(i);

    end
    
   % figure; 
    %grid on;
%     hold on
% 
%     plot(Wto,'.-m')
end

