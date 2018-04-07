%% ME547 - Project
clear;
clear all;
close all;
clc;

%%-----------------ORIENT ROBOT TO CONVERT INTO 2D PROBLEM------------------%%

	% Location of the target and the robot base
	targetPos = [740 ; 0 ; 500]; 
	robotPos = [0 ; 0 ; 50]; 

	% Define the lengths between each joint [ L23, L34, L46 ]
	l = [ 260, 290, 78 ];

	% Orient q1 so that the robot faces the target (angle in radians)
	q1 = atan((targetPos(1) - robotPos(1)) / (targetPos(2) - robotPos(2)));

	% Orient q5 so that the gripper on the end effector opens parallel to the floor
	q5 = pi / 2;	

%%------------------------ DRAW THE WORKSPACE OF THE END EFFECTOR ------------------------%%
	
	set(0,'DefaultFigureWindowStyle','docked')
	figure(1);

	% Discretize the workspace with 'n' points
	n = 2000; 

	% Define the full range of motion for each joint
	Q2 = (110*pi/180 + 120*pi/180).*rand(n,1) + -110*pi/180; 		% [-110 , 120]
	Q3 = (205*pi/180 + 69*pi/180).*rand(n,1) - 205*pi/180 + pi/2;	% [-69  , 205]
	Q4 = (120*pi/180 + 120*pi/180).*rand(n,1) - 120*pi/180;			% [-120 , 120]

	% Generate the coordinate points in configuration space
	X = l(1).*sin(Q2) + l(2).*sin(Q2 + Q3) + l(3).*sin(Q2 + Q3 + Q4);
	Z = l(1).*cos(Q2) + l(2).*cos(Q2 + Q3) + l(3).*cos(Q2 + Q3 + Q4);
	
	% Draw the workspace
	WS = boundary(X,Z);
	scatter(X, Z);
	xlim([-800 800]);
	ylim([-800 800]);
	hold on;
	plot(X(WS),Z(WS),'b', 'LineWidth',2);

%%-------------- DRAW THE 'RELEASE LOCATION' WORKSPACE OF THE END EFFECTOR --------------%%
	
	% Discretize the 'release-location' workspace with 'n' points
<<<<<<< HEAD
	n = 2; 
=======
	n = 1; 
>>>>>>> 7441c0d3a5c0129f958d63864a9cbad0bbc0d025

	% Define the range of motion for the 'release-location' workspace
	q2 = ones(n,1)*60*pi/180; % This sets a fixed value for joint 2 to simplify the math
	q3 = (60*pi/180 + 10*pi/180).*rand(n,1) - 60*pi/180 + pi/2;
	q4 = (60*pi/180 + 10*pi/180).*rand(n,1) - 60*pi/180;

	% Generate the joint coordinates in configuration space at time of release
	x3 = l(1) .* sin(q2);
	z3 = l(1) .* cos(q2);
	x4 = l(1).*sin(q2) + l(2).*sin(q2 + q3);
	z4 = l(1).*cos(q2) + l(2).*cos(q2 + q3);
	x = l(1).*sin(q2) + l(2).*sin(q2 + q3) + l(3).*sin(q2 + q3 + q4);
	z = l(1).*cos(q2) + l(2).*cos(q2 + q3) + l(3).*cos(q2 + q3 + q4);
	
	% Draw the workspace	
	ws = boundary(x,z);
	scatter(x3, z3, 'filled');	
	hold on;
	scatter(x4, z4, 'filled');
	hold on;
	scatter(x, z, 'filled');
	hold on;
	plot(x(ws),z(ws),'r', 'LineWidth',1);
	hold on;

<<<<<<< HEAD
% %%---------- CALCULATE PROJECTILE TRAJECTORY AND FIND RELEASE VELOCITY FOR BALL ----------%%
% 	
% 	L36 = sqrt((z3 - z).^2 + (x - x3).^2); % Distance from joint 3 to the end effector
% 	
% 	% Target position at the point of release
% 	targetD = sqrt(targetPos(1)^2 + targetPos(2)^2); % Floor distance from the target to joint 3
% 	targetH = targetPos(3); % Height of target
% 	
% 	% Ball release angle, THETA, is calculated as tangent to the motion of L36
% 	% This value is currently assumed and assumes q4_dot is zero at release...which is not the case.
% 	theta = [random('uniform', atan((targetPos(3)-z)./(targetPos(1)-x)), pi*90/180), random('uniform', atan((targetPos(3)-z)./(targetPos(1)-x)), pi*90/180), random('uniform', atan((targetPos(3)-z)./(targetPos(1)-x)), pi*90/180)];
% 
% 	% These are place holders
% 	a = (targetD - x) .* tan(theta);
% 	b = 4.9 * (targetD - x).^2 ./ cos(theta).^2;
% 	c = targetH - z;
% 
% 	% Tangential and angular velocity of end effector
% 	v = sqrt(b ./ (a - c));
% 	w = (v ./ L36)*180/pi;
% 
% %%------------------------------------ SIMULATE BALL  -------------------------------------%%
% 
% 	j = 20;
%     h = 3; % h should be consistent with the size of theta
% %     xBall = zeros(n,j);
% %     zBall = zeros(n,j);
%     xBall = zeros(n,j, 3);
%     zBall = zeros(n,j, 3);
% 	xBall(:, 1, 1) = x;
% 	zBall(:, 1, 1) = z;
%     xBall(:, 1, 2) = x;
% 	zBall(:, 1, 2) = z;
%     xBall(:, 1, 3) = x;
% 	zBall(:, 1, 3) = z;
% 
% 	for J = 1 : n
%         for H = 1 : h
%             K = 1;
% 		    for t = 0.01 : 0.01 : j  
% 		        % Ball position
% 		        xBall(J, K+1, H) = xBall(J, 1, H) + v(J, H) .* cos(theta(J, H)) * t;
% 		        zBall(J, K+1, H) = zBall(J, 1, H) + v(J, H) .* sin(theta(J, H)) * t - 0.5 * 9.81 * t^2;
% 
% 		        K = K + 1;
%             end
%             plot(xBall(J, :, H), zBall(J, :, H), 'g', 'lineWidth', 3);
%             hold on;
%         end
% 	end	
% 	
% 	title("Workspace");
% 	xlabel("X Position (mm)");
% 	ylabel("Z Position (mm)");
% 	
% 	% Plot points of interest
% 	text(0, 0,'O','Color','red','FontSize',14);
% 	text(targetPos(1), targetPos(3),'Target','Color','red','FontSize',14);

%%---------- CALCULATE PROJECTILE TRAJECTORY AND FIND RELEASE VELOCITY FOR BALL ----------%%
	
	L36 = sqrt((z3 - z).^2 + (x - x3).^2); % Distance from joint 3 to the end effector
	
	% Target position at the point of release
	targetD = sqrt(targetPos(1)^2 + targetPos(2)^2); % Floor distance from the target to joint 3
	targetH = targetPos(3); % Height of target
	
	% Ball release angle, THETA, is calculated as tangent to the motion of L36
	% This value is currently assumed and assumes q4_dot is zero at release...which is not the case.
	%theta = pi/2 - atan((z3 - z) ./ (x - x3));
	theta = [random('uniform', atan((targetPos(3)-z)./(targetPos(1)-x)), pi*90/180), random('uniform', atan((targetPos(3)-z)./(targetPos(1)-x)), pi*90/180), random('uniform', atan((targetPos(3)-z)./(targetPos(1)-x)), pi*90/180)];

	% These are place holders
	a = (targetD - x) .* tan(theta);
	b = 4.9 * (targetD - x).^2 ./ cos(theta).^2;
	c = targetH - z;

	% Tangential and angular velocity of end effector
	v = sqrt(b ./ (a - c));
	w = (v ./ L36)*180/pi;

%%------------------------------------ SIMULATE BALL  -------------------------------------%%

	j = 100; % Total duration of ball travel
	h = 3; % Number of angles for each release-location
	
	xBall = zeros(n,j,3);
	zBall = zeros(n,j,3);
 
	xBall(:, 1, 1) = x;
	zBall(:, 1, 1) = z;
    xBall(:, 1, 2) = x;
	zBall(:, 1, 2) = z;
    xBall(:, 1, 3) = x;
	zBall(:, 1, 3) = z;
	
	for J = 1 : 1 : n
		for H = 1 : 1 : h
			K = 1; % Index for ball position at each time step
			for t = 0.01 : 0.01 : j  
			    % Ball position
			    xBall(J, K+1, H) = xBall(J, 1, H) + v(J, H) .* cos(theta(J, H)) * t;
			    zBall(J, K+1, H) = zBall(J, 1, H) + v(J, H) .* sin(theta(J, H)) * t - 0.5 * 9.81 * t^2;

			    K = K + 1;   
            end
            plot(xBall(J, :, H), zBall(J, :, H), 'g', 'lineWidth', 2);
	hold on;
		end
    end	
    
	title("Workspace");
	xlabel("X Position (mm)");
	ylabel("Z Position (mm)");
	
	% Plot points of interest
	text(0, 0,'O','Color','red','FontSize',14);
	text(targetPos(1), targetPos(3),'Target','Color','red','FontSize',14);


%%------------------------- DEFINE INITIAL JOINT POSITION -------------------------%%

	% Sampling period
	dt = 0.01; % Interval duration in seconds
	T = 0.5; % Simulation duration in seconds

	% Initial joint angles [q3, q4]
	q(1,:) = [ 60*pi/180 + pi/2, -110*pi/180];
	q_2 = 60*pi/180;

	s1 = sin(q_2 + q(1,1));
	s12 = sin(q_2 + q(1,1) + q(1,2));
	c1 = cos(q_2 + q(1,1));
	c12 = cos(q_2 + q(1,1) + q(1,2));

%%------------------------------- PATH PLANNING - X -------------------------------%%

	x_path(1) = l(1).*sin(q_2) + l(2).*s1 + l(3).*s12; % End-effector initial position	
	xRelease = x; % End-effector final position
	x_dot = 0; % End-effector initial speed
	xRelease_dot = v .* cos(q3); % End-effector final speed
	
	x_0 = x(1);
	x_1 = x_dot;
	x_2 = 3 * (xRelease - x_path(1)) / T^2 - 2 * (xRelease_dot - x_dot) / T;
	x_3 = -2 * (xRelease - x_path(1)) / T^3 + (xRelease_dot - x_dot) / T^2;

%%------------------------------ PATH PLANNING - Z ------------------------------%%

	z_path(1) = l(1).*cos(q_2) + l(2).*c1 + l(3).*c12; % End-effector initial position	
	zRelease = z; % End-effector final position
	z_dot = 0; % End-effector initial speed
	zRelease_dot = v .* sin(q3); % End-effector final speed
	
	z_0 = z(1);
	z_1 = z_dot;
	z_2 = 3 * (zRelease - z_path(1)) / T^2 - 2 * (zRelease_dot - z_dot) / T;
	z_3 = -2 * (zRelease - z_path(1)) / T^3 + (zRelease_dot - z_dot) / T^2;

%%-------------------------- SIMULATION - END EFFECTOR --------------------------%%
    x_path = zeros(1,T/dt);
    z_path = zeros(1,T/dt);
    Vx = zeros(1,T/dt);
    Vz = zeros(1,T/dt);
	k = 1;
	for t = dt : dt : T
	        
	    % End-effector's position
	    x_path(k+1) = x_0 + x_1*(t/T) + x_2*(t/T)^2 + x_3*(t/T)^3;
	    z_path(k+1) = z_0 + z_1*(t/T) + z_2*(t/T)^2 + z_3*(t/T)^3;

	    % End-effector velocity
	    Vx(k) = ( x_path(k+1) - x_path(k) ) / dt;
	    Vz(k) = ( z_path(k+1) - z_path(k) ) / dt;
	    
	    s1 = sin(q_2 + q(k,1));
	    s12 = sin(q_2 + q(k,1) + q(k,2));
	    c1 = cos(q_2 + q(k,1));
	    c12 = cos(q_2 + q(k,1) + q(k,2));

		% Compute Jacobian (J)
	    J(1,1) = - l(2) * s1 - l(3)*s12;
	    J(1,2) = - l(3) * s12;
	    J(2,1) =   l(2) * c1 + l(3)*c12;
	    J(2,2) =   l(3) * c12;

	    % Calculating joint angular velocities from end effector velocity ([Vx(k);Vz(k)])
	    qr_dot = inv(J) * [Vx(k); Vz(k)];

	    % Save the angles of the joints
		q_dot(k+1,:) = qr_dot;
        q_dotdot(k+1,:)=(q_dot(k+1,:)-q_dot(k,:))/dt;
	    % Numerical integration 
	    q(k+1,:) = q(k,:) + dt * qr_dot;
	    
	    k = k + 1;   
	end

	% Velocity on the last position V(t=T)=0 
	% It's a point-to-point task; therefore, the end-effector stops (Vx=Vz=0)
	% This is written so to have the same vector length with t for the plot
	Vx(k) = 0;
	Vz(k) = 0;

%%--------------------------------- PLOT ANGLES ---------------------------------%%
    % Plotting reference joint angles
	figure(2);
	t = [0 : dt : T];
	subplot(2,2,1); 
	plot(t, q(:,1)*180/pi, 'b', 'lineWidth', 2);
	xlim([0.1 1.9]);
    grid on;
	ylabel('q_3 (degrees)'); 
	xlabel('t (sec)');  
	subplot(2,2,2); 
	plot(t, q(:,2)*180/pi, 'b', 'lineWidth', 2);
    xlim([0.1 1.9]);
	grid on;
	ylabel('q_4 (degrees)'); 
	xlabel('t (sec)');  

	% Plotting angular velocities of the joints
	subplot(2,2,3); 
	plot(t, q_dot(:,1)*180/pi, 'r', 'lineWidth', 2);
	xlim([0.1 1.9]);
    grid on;
	ylabel('qdot_3 (degrees/s)'); 
	xlabel('t (sec)');  
	subplot(2,2,4); 
	plot(t, q_dot(:,2)*180/pi, 'r', 'lineWidth', 2);
    xlim([0.1 1.9]);
	grid on;
	ylabel('qdot_4 (degrees/s)'); 
	xlabel('t (sec)');  
	axis 'auto x';

%%------------------------------ PLOT END EFFECTOR ------------------------------%%
	figure(3);

	subplot(2,2,1); 
	plot(t, x_path, 'b', 'lineWidth', 2);
	grid on;
    xlim([0.1 1.9]);
	ylabel('x (mm)'); 
	xlabel('t (sec)');  
	subplot(2,2,2); 
	plot(t, z_path, 'b', 'lineWidth', 2);
	grid on;
    xlim([0.1 1.9]);
	ylabel('z (mm)'); 
	xlabel('t (sec)');  
	subplot(2,2,3); 
	plot(t, Vx, 'r', 'lineWidth', 2);
	grid on;
    xlim([0.1 1.9]);
	ylabel('Vx (mm/s)'); 
	xlabel('t (sec)');
	subplot(2,2,4); 
	plot(t, Vz, 'r', 'lineWidth', 2);
	grid on;
    xlim([0.1 1.9]);
	ylabel('Vz (mm/s)'); 
	xlabel('t (sec)');
	axis 'auto x';

%%------------------------------ PLOT KINEMATICS ------------------------------%%

	figure(4);

	subplot(2,2,1); 
	plot(x_path, z_path, 'b', 'lineWidth', 2);
	grid on;
    xlim([-800 800]);
	ylabel('z (mm)'); 
	xlabel('x (mm)');  
	  
	subplot(2,2,3); 
	plot(Vx, Vz, 'r', 'lineWidth', 2);
	grid on;
    xlim([-100 100]);
	ylabel('Vz (mm/s)'); 
	xlabel('Vx (mm/s)');
	
	axis 'auto x';

%%--------------------------------- PLOT ANGLES ---------------------------------%%
    % Plotting reference joint angles
	figure(2);
	t = [0 : dt : T];
	subplot(2,2,1); 
	plot(t, q(:,1)*180/pi, 'b', 'lineWidth', 2);
	xlim([0.1 1.9]);
    grid on;
	ylabel('q_3 (degrees)'); 
	xlabel('t (sec)');  
	subplot(2,2,2); 
	plot(t, q(:,2)*180/pi, 'b', 'lineWidth', 2);
    xlim([0.1 1.9]);
	grid on;
	ylabel('q_4 (degrees)'); 
	xlabel('t (sec)');  

	% Plotting angular velocities of the joints
	subplot(2,2,3); 
	plot(t, q_dot(:,1)*180/pi, 'r', 'lineWidth', 2);
	xlim([0.1 1.9]);
    grid on;
	ylabel('qdot_3 (degrees/s)'); 
	xlabel('t (sec)');  
	subplot(2,2,4); 
	plot(t, q_dot(:,2)*180/pi, 'r', 'lineWidth', 2);
    xlim([0.1 1.9]);
	grid on;
	ylabel('qdot_4 (degrees/s)'); 
	xlabel('t (sec)');  
	axis 'auto x';

%%------------------------------ PLOT END EFFECTOR ------------------------------%%
	figure(3);

	subplot(2,2,1); 
	plot(t, x_path, 'b', 'lineWidth', 2);
	grid on;
    xlim([0.1 1.9]);
	ylabel('x (mm)'); 
	xlabel('t (sec)');  
	subplot(2,2,2); 
	plot(t, z_path, 'b', 'lineWidth', 2);
	grid on;
    xlim([0.1 1.9]);
	ylabel('z (mm)'); 
	xlabel('t (sec)');  
	subplot(2,2,3); 
	plot(t, Vx, 'r', 'lineWidth', 2);
	grid on;
    xlim([0.1 1.9]);
	ylabel('Vx (mm/s)'); 
	xlabel('t (sec)');
	subplot(2,2,4); 
	plot(t, Vz, 'r', 'lineWidth', 2);
	grid on;
    xlim([0.1 1.9]);
	ylabel('Vz (mm/s)'); 
	xlabel('t (sec)');
	axis 'auto x';

%%------------------------------ PLOT KINEMATICS ------------------------------%%

	figure(4);

	subplot(2,2,1); 
	plot(x_path, z_path, 'b', 'lineWidth', 2);
	grid on;
    xlim([-800 800]);
	ylabel('z (mm)'); 
	xlabel('x (mm)');  
	  
	subplot(2,2,3); 
	plot(Vx, Vz, 'r', 'lineWidth', 2);
	grid on;
    xlim([-100 100]);
	ylabel('Vz (mm/s)'); 
	xlabel('Vx (mm/s)');
	
	axis 'auto x';
	