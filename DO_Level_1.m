tic();

%% SPECIFICATIONS AND DATA FROM RESEARCH PAPERS REFERED:------------------------------

    L_road    = 1000;          %% unit = m
    L_vehicle = 14.6;          %% unit = m
    safe_dist =  0  ;          %% unit = m - from gov site - for car travelling in city - avg speed
    Max_vel   = 25  ;          %% unit = m/s - from gov site- highway traffic
    R_time= 1       ;          %% unit = sec - from research paper
    mass = 1378.921 ;
    Dia_of_tyre = 0.81026;
% Gearing considerations:-
    DIFF_GEAR_RATIO  = 5.142;
    TRANS_GEAR_RATIO = 3.742;
    GR= DIFF_GEAR_RATIO * TRANS_GEAR_RATIO;
% power data from specs:-
    Power_min=0;
    Power_max=90229.7;


    
  % FILES:-
    load('Velocity_leader.mat')
    load('p.mat'); 
    load('enginedata.mat');
%% Calculations : --------------------------------------------------------
    
    vel_frnt=(Velocity_leader(:,2))';
    time_step=1:length(vel_frnt);
    n=length(time_step)-10;
    r = linspace(0.5*pi,2*pi,n);
    density=abs(1*sin (r));
    MAX_density=max(density);
  
    
    %% INITIALIZATION:-----------------------------------------------------
    T = zeros();
    U0=zeros(1,n);  
    d_travelled=zeros();
    U0=zeros(1,n);
    zeta=zeros();
    Total_mf=zeros();
    mf = zeros(); 
    current_vel = zeros();
  
%% OPTIMIZATION----------------------------------------------------------- 
   
    Torque_engine_max=max(eng_consum_trq);
    Torque_engine_min=min(eng_consum_trq);
    N_engine_max=max(eng_consum_spd);
    N_engine_min=min(eng_consum_spd);
    
    % Equation from engine data
    func=@(x) (p(1) + p(2) .* x(1) + p(3) .* x(2) + p(4) .* (x(1).^2) + p(5) .* x(1) .* x(2) + p(6) .* (x(2).^2) + p(7) .* (x(1).^3) + p(8) .* (x(1).^2) .* x(2) + p(9) .* x(1) .* (x(2).^2) + p(10) .* (x(2).^3) + p(11) .* (x(1).^4) + p(12) * (x(1).^3) .* (x(2)) + p(13) .* (x(1).^2) .* (x(2).^2) + p(14) .* x(1) .* (x(2).^3) + p(15) .* (x(2).^4))*(x(1)*x(2))/(0.77*1e3); 
    x0=[100,100]; 

for i=1:n
    Aeq = [1,0];                                                % Torque   constraint
    Beq = (vel_frnt(i)-safe_dist)*mass*(0.5*Dia_of_tyre)/GR;    % Distance constraint  
    A = [];
    B = [];
    ub = [Torque_engine_max N_engine_max];                      % Upper limit boundaries for Engine torque and Engine speed
    lb = [100  50];                                             % Lower limit boundaries for Engine torque and Engine speed
    [x,fval]=fmincon(func,x0,A,B,Aeq,Beq,lb,ub);
    T(i)=x(1);
    current_vel(i)=x(1)*GR/(mass*0.5*Dia_of_tyre);              % Optimum Velocity of car under consideration
    mf(i) = fval ;                                              % Optimum mass consuption of fuel per time step
    U0(i)=current_vel(i);               
    d_travelled(i)=U0(i)+current_vel(i)*0.5;                    % Distance travelled by car under consideration
    safe_dist=(vel_frnt(i)-current_vel(i));                     % Safe Distance behind the car in front
    x0=x;
end

%%% Calculating Total mass of fuel consumed -
for j=1:n-1
    Total_mf(j)=mf(j+1)+mf(j);
    fuel_cons(j)=abs(mf(j+1)-mf(j));        % Fuel conserved per time step
end

cyc_mps_a=[1:length(vel_frnt);vel_frnt];
cyc_mps=cyc_mps_a';

%% GRAPHS AND FIGURES-----------------------------------------------------

%Optimum Fuel Consumption
figure() 
plot(1:n,mf);
title('Optimum fuel consumption per time step');
xlabel('Number of time steps (One time step = 1sec)')
ylabel('Fuel consumption (L/s)')
grid on

%Velocity Profile comparison
figure()
plot(1:n,current_vel,'r',1:n,vel_frnt(1:n),'b');
title('velocity vs time steps');
xlabel( 'Number of time steps (One time step = 1sec)')
ylabel('Velocity (m/s)')
grid on;

%Optimum Distance travelled
figure()
plot(1:n,d_travelled);
title('Distance Travelled');
xlabel( 'Number of time steps (One time step = 1sec)')
ylabel('Distance (m)')
grid on;
 

%Velocity profile of the leader car derived from PDE model
figure()
plot(1:length(vel_frnt),vel_frnt,'-');
title('Velocity Profile for Car in front');
xlabel( 'Number of time steps (One time step = 1sec)')
ylabel('Velocity (m/s)')
grid on;




