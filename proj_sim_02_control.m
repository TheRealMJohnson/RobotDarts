set(0,'DefaultFigureWindowStyle','docked'); tic
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% q = [shoulder; elbow];
%
% Arash Arami 15/03/2018
% repurposed by Matthew Johnson 07/04/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% control gains
Kp=50;
Kd=14.14;

%% Renaming variables
Q_ref_s=q(:,1);
Q_ref_e=q(:,2);
Q_dot_ref_s=q_dot(:,1);
Q_dot_ref_e=q_dot(:,2);

Q_ref = q';
Q_dot_ref = q_dot';
%%
%sim time and sample rate based on proj_sim_01 values

% GRAVITY ON:1 or OFF:0 (keep it off for HW 2)
Gravity = 1;

% Initialization of actual kinematics
q_act= q(1,:)';
qdot_act = [0,0]';
angle = zeros(T/dt,2);
t = zeros(T/dt,1);

% Simulation parameters
Mass_S = 0.1; Mass_E = 0.1; % in kg
Length_S = l(2); Length_E = l(3); % in m
CoM_S = Length_S/2; CoM_E = Length_E/2;   % in m
MoI_S = Mass_S*CoM_S^2; MoI_E = Mass_E*CoM_E^2;  % in kg.m^2, for a point mass

% Euler method
UpdateAngle = @(q,qdot)([q(1)+dt*qdot(1);
    q(2)+dt*qdot(2)]);

UpdateVel = @(qdot,qddot)([qdot(1)+dt*qddot(1);
    qdot(2)+dt*qddot(2)]);

% Mass matrix
ComputeMassMatrix = @(q)[MoI_S+MoI_E+Mass_S*CoM_S^2+Mass_E*(Length_S^2+CoM_E^2+2*Length_S*CoM_E*cos(q(2))), MoI_E+Mass_E*(CoM_E^2+Length_S*CoM_E*cos(q(2)));
    MoI_E+Mass_E*(CoM_E^2+Length_S*CoM_E*cos(q(2))), MoI_E+Mass_E*CoM_E^2];
EstimateMassMatrix = @(q)[MoI_S*1.1+MoI_E*1.1+Mass_S*CoM_S^2*0.81+Mass_E*(Length_S^2*0.81+CoM_E^2*0.81+0.81*2*Length_S*CoM_E*cos(q(2))), MoI_E*1.1+0.81*Mass_E*(CoM_E^2+Length_S*CoM_E*cos(q(2)));
    MoI_E*1.1+0.81*Mass_E*(CoM_E^2+Length_S*CoM_E*cos(q(2))), 1.1*MoI_E+0.81*Mass_E*CoM_E^2];
% Cqdot matrix
ComputeCqdotMatrix = @(q,qdot)[-Mass_E*Length_S*CoM_S*qdot(2)*(2*qdot(1)+qdot(2))*sin(q(2));
    Mass_E*Length_S*CoM_E*qdot(1)^2*sin(q(2))];
EstimateCqdotMatrix = @(q,qdot)[-Mass_E*Length_S*CoM_S*qdot(2)*(2*qdot(1)+qdot(2))*sin(q(2))*0.81;
    Mass_E*Length_S*CoM_E*qdot(1)^2*sin(q(2))*0.81];

% Gravity matrix
% ComputeGravityMatrix = @(q)[-(Mass_S*CoM_S+Mass_E*Length_S)*9.8*cos(q(1))-Mass_E*9.8*CoM_E*cos(q(1)+q(2));
%                          -Mass_E*CoM_E*9.8*cos(q(1)+q(2))];
ComputeGravityMatrix = @(q)[(Mass_S*CoM_S+Mass_E*Length_S)*9.8*cos(q(1))+Mass_E*9.8*CoM_E*cos(q(1)+q(2));
    Mass_E*CoM_E*9.8*cos(q(1)+q(2))];
EstimateGravityMatrix = @(q)[(Mass_S*CoM_S*0.9+Mass_E*Length_S*0.9)*9.8*cos(q(1))+Mass_E*9.8*CoM_E*cos(q(1)+q(2))*0.9;
    Mass_E*CoM_E*9.8*cos(q(1)+q(2))*0.9];
% Dynamics
JointAccel = @(Torque,H,Cqdot,G)(H\(Torque-Cqdot-G));
%H*acc+Cqdot+G = Torque

%% Compute the FF torques here before the main loop for CONTROLLER II
% this feedforward torque should account for desired acceleration
Torque_FF = zeros(2,T/dt);
Torque_FF_Comp = zeros(2,T/dt);
for i=1:T/dt
    % add the feedforward torque computatio here
    Torque_FF(:,i) = ( EstimateMassMatrix(q(i,:)')*(q_dotdot(i,:)') );    
end
for i=1:T/dt
    t(i) = i*dt;
    %% measurement of joint angles and angular velocity part b and c
    
    q_m=q_act +randn(2,1)*0.0044; %
    qdot_m=qdot_act+randn(2,1)*0.044;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% complete the PD controller equation
    % such that Torque be a vector of 2x1 (row 1: shoulder row 2: elbow)
    % only one controller torque should be uncommented at each simulation
    %% CONTROLLER I: PD controller torque
    Torque_PD = Kp*(Q_ref(:,i) - q_m) + Kd*(Q_dot_ref(:,i) - qdot_m);
    Torque1 = Torque_PD;
    %% CONTROLLER II: PD-FF controller torque
    Torque2= Torque_PD + Torque_FF(:,i);
    %% CONTROLLER III: PD-FF + gravity and C compensation (controller torque)
    Cqdot_est = EstimateCqdotMatrix(q(i,:)',q_dot(i,:)');
    G_est =  EstimateGravityMatrix(q(i,:)');
    Torque3 = Torque_PD + (Torque_FF(:,i)+G_est+Cqdot_est);
    
    Torque = Torque3;
    %%%%%%%%%
    %%
    
    % Calculate H, Cqdot and G
    H = ComputeMassMatrix(q_act);
    
    Cqdot = ComputeCqdotMatrix(q_act,qdot_act);
    
    if Gravity==1
        G = ComputeGravityMatrix(q_act);
    else
        G = 0*ComputeGravityMatrix(q_act);
    end
    
    % Update joint angle
    qddot_act = JointAccel(Torque,H,Cqdot,G);
    qdot_act = UpdateVel(qdot_act,qddot_act);
    q_act = UpdateAngle(q_act,qdot_act);
    
    % Collect data
    angle(i,1) = q_act(1)*180/pi; %in degree
    angle(i,2) = q_act(2)*180/pi;
    velocity(i,1) = qdot_act(1);
    velocity(i,2) = qdot_act(2);
    acc(i,1) = qddot_act(1);
    acc(i,2) = qddot_act(2);
    Cmatrix(i,1) = Cqdot(1);
    Cmatrix(i,2) = Cqdot(2);
    T1(i,1) = Torque(1);
    T1(i,2) = Torque(2);
end
figure
plot(t(:,1),angle(:,1),'r','linewidth',2)
hold on
plot(t(:,1),angle(:,2),'g','linewidth',2)
plot(t(:,1),Q_ref_s(1:T/dt)'*180/pi,'r--','linewidth',1)
plot(t(:,1),Q_ref_e(1:T/dt)'*180/pi,'g--','linewidth',1)
xlabel('time [s]')
ylabel('angle [degree]')
legend('shoulder','elbow')
set(gca,'fontsize',24)
figure
plot(t(:,1),T1(:,1),'r','linewidth',2)
hold on
plot(t(:,1),T1(:,2),'g','linewidth',2)
xlabel('time [s]')
ylabel('Torque [N.m]')
legend('shoulder','elbow')
set(gca,'fontsize',24)
%%  end effector position
% forward kinematics
for i=1:T/dt
    c1=cosd(angle(i,1));
    s1=sind(angle(i,1));
    c12=cosd(angle(i,1)+angle(i,2));
    s12=sind(angle(i,1)+angle(i,2));
    xact(i) = l(1) * c1 + l(2) * c12 ;
    yact(i) = l(1) * s1 + l(2) * s12 ;
end
figure
plot(x_path(2:end),z_path(2:end),'r--','linewidth',2)
hold on
plot(xact,yact,'linewidth',2)
xlabel('x [m]')
ylabel('y [m]')
legend('desired','actual', 'location', 'best')
set(gca,'fontsize',24)
%% Performance metrics
% compute Root Mean Square Error (RMSE) of elbow and shoulder angle
% compute Root Mean Square Torque (RMST) for elbow and shoulder
% compute Root Mean Square EE position error
% repeat it with noise and without noise
RMSEs=sqrt((Q_ref_s(2:end)*180/pi-angle(:,1))'*(Q_ref_s(2:end)*180/pi-angle(:,1))/200);
RMSEe=sqrt((Q_ref_e(2:end)*180/pi-angle(:,2))'*(Q_ref_e(2:end)*180/pi-angle(:,2))/200);
RMSTs=sqrt(T1(:,1)'*T1(:,1)/length(T1));
RMSTe=sqrt(T1(:,2)'*T1(:,2)/length(T1));
RMSEee=sqrt(((x_path(2:end)-xact)*(x_path(2:end)-xact)'+(z_path(2:end)-yact)*(z_path(2:end)-yact)')/length(yact));
[RMSEs RMSEe RMSTs RMSTe RMSEee]

