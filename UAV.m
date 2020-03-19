clc; clear all; close all;
%% initializing best aircraft specs
Ebest = 0;
Rbest = 0;
valid_aircrafts = 0; %will tally valid aircraft
validWeights = []; %will record all valid weights
unstable = 0; %tally unstable models
invalid_endurance=0; %tally short endurance aircraft
rng(1);
iterations = 1000; 

% figure
% hold on
% title('Weight to Surface Area Plot')
% xlabel('Weight (lb)')
% ylabel('Surface Area of Wing (sqft)')

%%Loop finding best aircraft specs
for i=1:iterations
W=60; % initial weight guess

% Flight Conditions
density = .001496; %altitude of 15000 (slugs/ft^3)
%velocity = 81; %scaneagle velocity
velocity = 30 + rand()*100;
C_l = 1.2; %Estimation
S = W/(.5*density*velocity^2 * C_l );

%aircraft dimensions
A = 1+10*rand();  %Aspect Ratio rand(1-11)
Sh = 1+2*rand(); %Area of horizontal tail rand(0-5) 
Sv = 1+rand(); %Area of vertical tail (0-5)
bh = 1+3*rand(); %wingspan of horizontal tail (0-5)
bv = 1 + rand(); %wingspan of vertical tail
space = 1+3*rand(); %distance (ft) from leading edge of fuselage to leading edge of wing
c = S/(sqrt(A*S)); %Avg Chord length of wing?
hac = 0.25;  %fraction of leading edge of wing to ac

ch = Sh/bh;  %Chord of horizontal tail
cv = Sv/bv;  %Chord of vertical tail
hach = 0.25; %fraction of LE of reg wing to horizontal wing ac
hacv = 0.25; %fraction of LE of vertical tail to vtail ac

%Engine specs
eff= 0.750; %prop efficiency
fc = 0.6575947224; %specific fuel consumption [lbs/hp/hr]
ratio = 12; %Lift-to-Drag Ratio
Pav = 3.4; % power available [hp]

[W,Ww,Wau,Weng,Wf,WF,Wfs,Wfu,Wht,Wvt,Wlg,Wp,Wpl,Wsc,Delta,tr,S,lf,lh,tc,D,N] = niccolai(W,A,Sh,Sv,bh,bv,hac,c,ch,cv,hach,hacv,space);
Wfinal = W - Wfu;


b = sqrt(A*S); % Wingspan
c = S/b; %Chord length of wing
c_tip = (2*S/((1+tr)*b))*(1- ((1-tr)*b/b)); %Tip Chord length of wing
c_root = (2*S/((1+tr)*b)); %updated Chord length of wing
[cd,clmax,Vstall,K,Vcruise,Vdmin,R,E,Max,e,delta_tr,vmax] = calcs(WF,b,c_root,c_tip,S,tc,tr,c,D,lf,ch,cv,Sh,Sv,Delta,W,density,A,Wfinal,eff,fc,Pav);
[cg,cgwing,hn,hcg,sm,sm1,a_w,ep,a_t,hachtemp] = CG(W,Ww,Wau,Wf,Wfs,Wfu,Wht,Wvt,Wlg,Wp,Wpl,Wsc,c,ch,cv,space,Delta,tr,c_root,c_tip,lf,hac,hach,hacv,lh,b,Sh,S,e,A,bh);

ac = space + hac*c;
xn = hn*c + space;
    %find if aircraft is valid
    if(sm1>0.05 && sm1<0.3 && sm>0.05 && sm<0.3 && E>29000)
         valid_aircrafts = valid_aircrafts+1;
         validWeights(valid_aircrafts) = W;
         S = real(S);
         W = real(W);
       % scatter(W,S,'filled', 'b');
        %scatter(W,Wfs,'filled','g'); %Fuel Weight vs Plane Weight
    else
         S = real(S);
         W = real(W);
        %find Reason for failure
        if(sm1<0.05 || sm1>0.3 || sm<0.05 || sm>0.3)
            unstable = unstable+1;
        end
        if(E<29000)
            invalid_endurance = invalid_endurance+1;
        end
    end
    
   

    if(E>Ebest && cv<ch && bv<bh && ch<c_root && R>Rbest && lh>c+0.5 && E>29000 && sm1>0.05 && sm1<0.3 && sm>0.05 && sm<0.3)
        %store all the specs if its better than the last best one
        Abest = A;
        a_wbest = a_w;
        a_tbest = a_t;
        acbest = ac;
        bbest = b;
        bhbest = bh;
        bvbest = bv;
        cbest = c;
        cdbest = cd;
        chbest = ch;
        c_tipbest = c_tip;
        clmaxbest = clmax;
        c_rootbest = c_root;
        cvbest = cv;
        Dbest = D;
        Deltabest = Delta;
        ebest = e;
        Ebest = E;
        epbest = ep;
        fcbest = fc;
        hcgbest = hcg;
        hnbest = hn;
        hachtempbest = hachtemp;
        lfbest = lf;
        lhbest = lh;
        Pavbest = Pav;
        Rbest = R;
        Sbest = S;
        Shbest = Sh;
        smbest = sm;
        sm1best = sm1;
        spacebest = space;
        Svbest = Sv;
        tcbest = tc;
        trbest = tr;
        Vcruisebest = Vcruise;
        vmaxbest = vmax;
        Vstallbest = Vstall;
        Wbest = W;
        Wwbest = Ww;
        Wfbest = Wf;
        Whtbest = Wht;
        Wvtbest = Wvt;
        WFbest = WF;

        
        
        continue;
    else
        continue;
    end
end

%% Prop Design
%Engine speed in rpm (8500 rpm for our engine):
engspeed = 8500;

%Convert engine speed to rps (divide by 60):
n = engspeed/60;

%Compute average weight of aircraft throughout flight:
Wav = Wbest - Wfu/2;

%Solve for V at takeoff:
Vt = 1.2*Vstallbest;

%Calculate k:
kbest = 1/(pi*Abest*ebest);

%Obtain the CL, Drag, Preq for takeoff conditions:
Clt = Wbest/(0.5*density*(Vt^2)*Sbest);
Dt = 0.5*density*(Vt^2)*Sbest*(cdbest + kbest*(Clt^2));
Preqt = (Dt*Vt)/550; 

%Obtain the CL, Drag, Preq for cruising conditions:
Clcr = Wav/(0.5*density*(Vcruisebest^2)*Sbest);
Dcr = 0.5*density*(Vcruisebest^2)*Sbest*(cdbest + kbest*(Clcr^2));
Preqcr = (Dcr*Vcruisebest)/550;

%Solve for Vclimb (Vclimb should be the V at minimum power):
Clcl = sqrt((3*cdbest)/kbest);
Vcl = sqrt(Wbest/(0.5*density*Sbest*Clcl));

%Obtain the Drag and Preq for climbing conditions:
Dcl = 0.5*density*(Vcl^2)*Sbest*(cdbest + kbest*(Clcl^2));
Preqcl = (Dcl*Vcl)/550;

%Compute the Cs values:
Cs1 = Vcruisebest*(density/(Preqcr*550*n^2))^0.2;    %cruise
Cs2 = vmaxbest*(density/(Pav*550*n^2))^0.2;          %top speed
Cs3 = Vcl*(density/(Preqcl*550*n^2))^0.2;            %climb
Cs4 = Vt*(density/(Preqt*550*n^2))^0.2;              %takeoff

% Propeller diameter
J1 = 0.475;
Dprop1 = Vcruisebest/(n*J1); %prop diameter in ft (estimated using NACA report)
Dprop2 = 1.600;
Dprop3 = 1.062;
Dprop4 = 1.077;

% Compute the power coefficient:
Cp1 = (Preqcr*550)/(density*Dprop1*n^3);            %cruise 
Cp2 = (Pav*550)/(density*Dprop1*n^3);               %top speed
Cp3 = (Preqcl*550)/(density*Dprop1*n^3);            %climb
Cp4 = (Preqt*550)/(density*Dprop1*n^3);             %takeoff

% Compute thrust for each case (T=D):
T1 = Dcr;
T2 = (550*Pav)/vmaxbest;
T3 = (550*Pav)/Vcl;
T4 = (550*Pav)/Vt;

% Compute the thrust coefficient:
Ct1 = T1/(density*(n^2)*((Dprop1)^4));           %cruise 
Ct2 = T2/(density*(n^2)*((Dprop1)^4));           %top speed 
Ct3 = T3/(density*(n^2)*((Dprop1)^4));           %climb
Ct4 = T4/(density*(n^2)*((Dprop1)^4));           %takeoff

% Compute the Mach # using each individually computer diameter:
a = 1125.33; %speed of sound in ft/sec
Mtcr = Vcruisebest/a*sqrt(1 + (pi*n*Dprop1/Vcruisebest)^2);    %cruise
Mttop = vmaxbest/a*sqrt(1 + (pi*n*Dprop2/vmaxbest)^2);          %top speed
Mtcl = Vcl/a*sqrt(1 + (pi*n*Dprop3/Vcl)^2);                    %climb
Mtt = Vt/a*sqrt(1 + (pi*n*Dprop4/Vt)^2);                      %takeoff

% Compute the Mach # using the optimal diameter:
Mt1 = Mtcr;                                                   %cruise
Mt2 = vmaxbest/a*sqrt(1 + (pi*n*Dprop1/vmaxbest)^2);          %top speed
Mt3 = Vcl/a*sqrt(1 + (pi*n*Dprop1/Vcl)^2);                    %climb
Mt4 = Vt/a*sqrt(1 + (pi*n*Dprop1/Vt)^2);                      %takeoff

%% Outputting Critical Information
fprintf('Valid Aircraft #%.0d \n', valid_aircrafts);
fprintf('                   \n');
fprintf('Best Aircraft Specs: \n');
fprintf('                   \n');
fprintf('Estimated Weight(lbs) = %.4f \n', Wbest);
fprintf('Static Margin = %.4f \n', smbest);
fprintf('Static Margin w/o fuel = %.4f \n', sm1best);
fprintf('C_l,max = %.4f \n', clmaxbest);
fprintf('Stall Speed(ft/s) = %.4f \n', Vstallbest);
fprintf('Cruise Speed(ft/s) = %.4f \n', Vcruisebest);
fprintf('Coefficient of Parasitic Drag = %.4f \n', cdbest);
fprintf('Endurance(hrs) = %.4f \n', Ebest/3600);
fprintf('Range(mi) = %.4f \n', Rbest/5280);

%% Print propeller design values:
fprintf('                   \n');
fprintf('Propeller Design: \n');
fprintf('                   \n');
fprintf('Speed (cruise) = %.4f \n', Vcruisebest);
fprintf('Speed (top speed) = %.4f \n', vmaxbest);
fprintf('Speed (climb) = %.4f \n', Vcl);
fprintf('Speed (takeoff) = %.4f \n', Vt);

fprintf('Preq (cruise) = %.4f \n', Preqcr);
fprintf('Pav (top speed) = %.4f \n', Pav);
fprintf('Preq (climb) = %.4f \n', Preqcl);
fprintf('Preq (takeoff) = %.4f \n', Preqt);

fprintf('Cs (cruise) = %.4f \n', Cs1);
fprintf('Cs (top speed) = %.4f \n', Cs2);
fprintf('Cs (climb) = %.4f \n', Cs3);
fprintf('Cs (takeoff) = %.4f \n', Cs4);

fprintf('Propeller Diameter = %.4f \n', Dprop1);

fprintf('Cp (cruise) = %.4f \n', Cp1);
fprintf('Cp (top speed) = %.4f \n', Cp2);
fprintf('Cp (climb) = %.4f \n', Cp3);
fprintf('Cp (takeoff) = %.4f \n', Cp4);

fprintf('Ct (cruise) = %.4f \n', Ct1);
fprintf('Ct (top speed) = %.4f \n', Ct2);
fprintf('Ct (climb) = %.4f \n', Ct3);
fprintf('Ct (takeoff) = %.4f \n', Ct4);

fprintf('Mt (cruise) = %.4f \n', Mt1);
fprintf('Mt (top speed) = %.4f \n', Mt2);
fprintf('Mt (climb) = %.4f \n', Mt3);
fprintf('Mt (takeoff) = %.4f \n', Mt4);

%% Plotting Valid Weights Histogram
validWeights = real(validWeights);
figure;
hist(validWeights, 10);
title('Valid Weights histogram')
xlabel('Weight')
ylabel('Number Valid Aircraft')
%Plotting the Reasons for Failure
figure
ReasonsForFailure = [unstable, invalid_endurance];
bar(ReasonsForFailure);
set(gca,'xticklabel',{'unstable','not enough endurance'});

%Create Plots
plotcalcs(WFbest,bbest,c_rootbest,c_tipbest,Sbest,tcbest,trbest,cbest,Dbest,lfbest,chbest,cvbest,Shbest,Svbest,Delta,Wbest,density,Abest,Wbest-8,eff,fcbest,Pavbest);

%% INCIDENCE ANGLE FOR TRIMMED FLIGHT AS A FUNCTION OF VELOCITY & stability derivatives
[incidence,CL_adot,CD0,CD_a,CD_de,Cm0,Cm_adot] = stabderiv(a_wbest,a_tbest,Sbest,Shbest,Wbest,density,hachtempbest,cbest,epbest,hcgbest,hnbest,Vstallbest,vmaxbest,Abest,Vcruisebest,ebest,cdbest);

%% make weight pie chart
weights = [Wwbest,Whtbest,Wvtbest,Wau,Wfbest,Wfs,Wfu,Wp,Wpl,Wsc];
labels = {'wing','horizontal tail','vertical tail','sensors',...
    'fuselage','fuel system','fuel','engine system','payload','actuators'};
figure
pie(weights)
legend(labels,'Location','NorthEast');
%% 3D model of Aircraft
figure
modelaircraft(lfbest,WFbest,Dbest,spacebest,cbest,bbest,chbest,bhbest,cvbest,bvbest);
