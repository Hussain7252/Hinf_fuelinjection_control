% H-infinity design example using the LMI toolbox
%
% Design of a controller for  fuel injection control
%
% Reference:  Kuraoka et. al.,  Application of Hinfty Design to
%             Automotive fuel control
%             IEEE Control Systems Magazine, 10, 3, April 1990, 102-106
%

%% First generate the plants
close all;
%    plant at 0 Celsius
no = [-0.01736 493.9 -313700];
do = [1 98.34 9223 87710];
po = tf(no,do);
%    plant at 25 Celsius (nominal)
n1 = [5.498 400.7 -444400];
d1 = [1 93.72 9530 121400];
p1 = tf(n1,d1);
%    plant at 60 Celsius
n2 = [4.677 -285.9 -505300];
d2 = [1 91.53 10080 176200];
p2 = tf(n2,d2);

%% Now look at the frequency responses
om1  = logspace(-1,4,100);
outo = freqresp(po,om1);
outo=squeeze(outo);
out1 = freqresp(p1,om1);
out1=squeeze(out1);
out2 = freqresp(p2,om1);
out2=squeeze(out2);
loglog(om1,abs(outo),om1,abs(out1),om1,abs(out2))
legend('0C','25C','60C')
pause
%% Multiplicative Uncertainty
% Use multiplicative uncertainty to cover the family of plants
deltao = (outo-out1)./out1;
delta2 = (out2-out1)./out1;
figure
% Plot the freq. response of the uncertainty
loglog(om1,abs(deltao),om1,abs(delta2))
legend('Delta 0','Delta 60')
 pause
%% Finding the covering weighing function, first try
% Need to cover this uncertainty with a weighting function
% Try the following
w2 = tf(22*[1 10],[1 1000]);  % uncertainty weight
outw2 = freqresp(w2,om1);
outw2=squeeze(outw2);
figure
% Plot the freq. response of the uncertainty
loglog(om1,abs(deltao),om1,abs(delta2),om1,abs(outw2))
legend('Delta 0','Delta 60','uncertainty weigh')


%% Finding the covering weighing function, second try
% Oops, too conservative at high frequency, try again
w2 = tf(2.2*[1 10],[1 100]);
outw2 = freqresp(w2,om1);
outw2=squeeze(outw2);
figure
% Plot the freq. response of the uncertainty
loglog(om1,abs(deltao),om1,abs(delta2),om1,abs(outw2))
legend('Delta 0','Delta 60','uncertainty weigh')

pause
%% Achieving robust stability
% Much better, try now to achieve robust stability:
% first need to generate the augmented  plant
%
%       [0   W2*P]
%  Pa = [        ]
%       [-1   -P ]
PS=[0 w2*p1;
    -1 -p1];
PS=minreal(PS);
hinfnorm(PS)
  
%% H-infity design with hinfsyn
% Now try an H infinity design
%
%

[A,B,C,D]=ssdata(PS);
PSS=ltisys(A,B,C,D);
% [k,g,gam] = hinfsyn(PSS,1,1,0,10,0.1);

%% H-infity design with hinflmi
% Oops, does not satisfy the conditions
% Let's use the LMI formulation
% (alternative: bilinear transf, Safonov and Chiang)
%
[A,B,C,D]=ssdata(PS);
PSS=ltisys(A,B,C,D);
[gam,k] = hinflmi(PSS,[1 1],0,1e-4);

%% Closed-loop inspection
% Success: lets look at the closed loop:
[numk,denk]=ltitf(k);
k=tf(numk,denk);
l = p1*k;
l=minreal(l);
outl = freqresp(l,om1);
loglog(om1,abs(squeeze(outl)));



%% Controller Transfer Function
% Something fishy: transfer function is 0, lets look at the controller


%% Bode plot of controller
outk = freqresp(k,om1);
loglog(om1,abs(squeeze(outk)));

%% Finding another solution
% The controller is K=0 !!
%
% By the way, notice that we got very large numbers
% Compare to the Riccati equation approch
%
% Since the plant does not satisfy the conditions
% need to perturb the plant a tiny bit
% (alternative: bilinear transf, Safonov and Chiang)
pause
close all
[a,b,c,d] = tf2ss(n1,d1);

%% Change d slightly
d11 = 1e-3;
p11 = ss(a,b,c,d11);

%% Comparison with the original plant
% Now compare the original and perturbed plants:
out11 = freqresp(p11,om1);
loglog(om1,abs(out1),om1,abs(squeeze(out11)));

pause

%% Generate the augmented plant
% Close enough!
%
% Generate the augmented plant using the perturbed plant
PS=[0 w2*p11;
    -1 -p11];
PS=minreal(PS);
[A,B,C,D]=ssdata(PS);
PSS=ltisys(A,B,C,D);
close all


%% Try H-infinity again
[k,g,gam] = hinfsyn(PSS,1,1,0,10,0.1);


%% Closed-loop inspection
% Success: lets look at the controller
[numk,denk]=sys2tf(k)
%%
%% The controller is 0 !!
%%
pause
%% Performance
% Need to incorporate performance specifications
% Need to decide on a performance weight
% Try
w1 = tf([1 100],[1 1]); %% low frequency tracking
outw1 = freqresp(w1,om1);
outw1=squeeze(outw1);
loglog(om1,abs(outw2),om1,abs(outw1))
legend('Uncertainty','Performance')
pause
%% Augmented plant
% Now create again the augmented plant
%
%       [W1  -W1  -W1*P]
%  Pa=  [0    0    W2*P]
%       [1   -1     - P]
Pa =[ w1 -w1 -w1*p1
    0 0 w2*p1
    1 -1 -p1];
Pa =balreal(Pa);
Pa=minreal(Pa,1e-3);
[A,B,C,D]=ssdata(Pa);
Pa=pck(A,B,C,D);

%% Minimum realization + Balancing
% Now get rid of unobservable/uncontrollable states

%% Check open-loop performance
hinfnorm(Pa)

%% Another try
% Does not satisfy the robust performance conditions
% Ready to try Hinfty design again:
  options=[0,0,1e-3,1];
 [gam,k]=hinflmi(Pa,[1 1],0,1e-4,options);
g = starp(Pa,k);
outg = frsp(g,om1); 

vplot('liv,lm',outg);

%% Checking Performance and Stability separately
% Oops: it doesn't satisfy the condition. However, need to 
% check g(1,1) (performance) and g(2,2) (stability)
hinfnorm(sel(g,1,1))
pause
%% Too much performance expectation!
% Doesn't satisfy: need to backoff performance a little bit, try:
w1n =tf(0.1*[1 100],[1 1]);
% plot the new weight
outw1n = freqresp(w1n,om1);
outw1n=squeeze(outw1n);
loglog(om1,abs(outw2),om1,abs(outw1n),om1,abs(outw1))
legend('Uncertainty','New Performance','Old Performance')

pause
%% Recalculate the augmented plant with the new performance weighing function
% Need to recalculate the augmented plant
Pa =[ w1n -w1n -w1n*p1
    0 0 w2*p1
    1 -1 -p1];
Pa=balreal(Pa);
Pa=minreal(Pa,1e-3);
[A,B,C,D]=ssdata(Pa);
Pa=pck(A,B,C,D);

%% H-infity again
% Try Hinfty again
[gam,k] = hinflmi(Pa,[1 1],0,1e-4);
[Ak,Bk,Ck,Dk]=unpck(k);
[numk,denk]=ss2tf(Ak,Bk,Ck,Dk);
ktf=tf(numk,denk);
ktf=minreal(ktf,1e-3);

% [k,g,gam] = hinfsyn(PS,1,1,0,10,0.1);
g = starp(Pa,k);
outg = frsp(g,om1);
vplot('liv,lm',outg);
%% H-infinity norms
% Check the hinfty norms of g11 and g22
hinfnorm(sel(g,1,1))
hinfnorm(sel(g,2,2))
outg = frsp(g,om1);
pause
%% Success!!
% Lets check robust performance
% check || WT WS || infinity
% (remember: if this norm < 1/sqrt(2) then robust performance
% (necessary condition only)
out = frsp(sel(g,[1:2],1),om1);
vplot('liv,lm', vnorm(out));
pkvnorm(out)*sqrt(2)
pause
%% Robust performance
% Less than 1, horray: robust performance.
close all

%% Closed-loop inspection
% Now form the closed loop system and check step responses
% First for the nominal plant
T = [0:0.02:0.4];
cl1=feedback(p1*ktf,1,-1);  %nominal plant
cl0=feedback(po*ktf,1,-1);  % plant at 0C
cl2=feedback(p2*ktf,1,-1);  % plant at 60C

%% Step response to the 0 Celsius plant
figure
step(cl1)
hold;

%% Adding step response of the nominal plant
 step(cl0)

%% Adding step response of the 60 Celsius plant
step(cl2)
legend('nominal','0','60')
%% Finished!!