%START OF MAIN CODE

%Example partial code to compute closed loop z-transfer function and characteristic equation=
syms z
%TASK1: Fill in your system parameter, controller, and time step values here
b=5; m=0.5 ; mhat=0.5 ; bhat=01.5*b ; T=0.1 ;

%Z-transforms of three common signals
Term1=(T*z/(z-1)^2);
Term2=z/(z-1);
%Term3=z/(z-exp(-a*T));

% TASK2: Complete next line for the open loop z-transfer function:
%Example open-loop z-transfer function includes zero-order hold and linear combination of ramp, step, and exponential functions.
Gzop=-bhat*Term1 + mhat*Term2;
[Gop_num ,Gop_den]=numden(Gzop); %Gets numerator and denominator
phiz_cl=collect(Gop_num+Gop_den); %Computes characteristic equation
ccs0=coeffs(vpa(collect(phiz_cl)));
ccs=ccs0(end:-1:1)
% TASK3: Interpret roots of characteristic equation
rrs=roots(ccs); %compute roots given polynomial coefficients
%Closed loop transfer function
Gzcl=collect(Gop_num/(Gop_num+Gop_den))
%END OF MAIN CODE
%%%%%%%%%%%%%%%
%START OF PLOTTING CODE

%(BONUS) plot discrete time response
%Closed loop analysis to plot Time Response
[Gzcl_num ,Gzcl_den]=numden(Gzcl);
ccsz0=coeffs(Gzcl_num);
ccsz1=coeffs(Gzcl_den);
AA=ccsz0(2);
BB=ccsz0(1);
CC=ccsz1(3);
DD=ccsz1(2);
EE=ccsz1(1);
rk=3;
yk_1=0;
yk_2=0;
for ii=1:100
%Task: write an expression for yk in terms of the input rk and past values yk_1, yk_2
%(Assume the discrete transfer function is Gzcl=(a*z+b)/(c*z^2+d*z+e).
%If you get stuck, use: Gzcl=(1.1036*z + 0.7927)/( z^2 - 0.2642*z + 1.1606);
yk=0; %<==Edit this line
%This code is ready
yk_1=yk;
yk_2=yk_1;
yk_save(ii)=yk;
end
plot(yk_save)
%END OF plotting CODE
%%%%%%%%%%%%%%%