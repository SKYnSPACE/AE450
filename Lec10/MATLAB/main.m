close all;
clc; clear;

addpath('./lib');
%% DEFINE
R2D = 180/pi;
D2R = pi/180;

%% INIT. PARAMS.
drone1_params = containers.Map({'mass','armLength','Ixx','Iyy','Izz'},...
    {1.25, 0.265, 0.0232, 0.0232, 0.0468});

drone1_initStates = [0, 0, -5, ...                                              % x, y, z
    0, 0, 0, ...                                                                % dx, dy, dz
    0, 0, 0, ...                                                                % phi, theta, psi
    0, 0, 0]';                                                                  % p, q, r

drone1_initInputs = [0, ...                                                     % ThrottleCMD
    0, 0, 0]';                                                                  % R, P, Y CMD

drone1_body = [ 0.265,      0,     0, 1; ...
                    0, -0.265,     0, 1; ...
               -0.265,      0,     0, 1; ...
                    0,  0.265,     0, 1; ...
                    0,      0,     0, 1; ...
                    0,      0, -0.15, 1]';
			  
drone1_gains = containers.Map(...
	{'P_phi','I_phi','D_phi',...
	'P_theta','I_theta','D_theta',...
	'P_psi','I_psi','D_psi',...
	'P_zdot','I_zdot','D_zdot'},...
    {0.2, 0.0, 0.15,...
	0.2, 0.0, 0.15,...
	0.8, 0.0, 0.3,...
	10.0, 0.2, 0.0});

simulationTime = 2;                                                          % [sec]

%% BIRTH OF A DRONE
drone1 = Drone(drone1_params, drone1_initStates, drone1_initInputs, drone1_gains, simulationTime);

%% Init. 3D Fig.
fig1 = figure('pos',[0 200 800 800]);
h = gca;
view(3);
fig1.CurrentAxes.ZDir = 'Reverse';
fig1.CurrentAxes.YDir = 'Reverse';
% view([-60 30]);
% view([90 0])   % To check phi (roll attitude) only
% view([0 0])    % To check theta (pitch attitude) only
% view([270 90]) % Top view
axis equal;
grid on;
xlim([-5 5]);
ylim([-5 5]);
zlim([-8 0]);
xlabel('X[m]');
ylabel('Y[m]');
zlabel('Height[m]');

hold(gca, 'on');
drone1_state = drone1.GetState();
wHb = [RPY2Rot(drone1_state(7:9))' drone1_state(1:3); 0 0 0 1];
% [Rot(also contains shear, reflection, local sacling), displacement; perspective ,global scaling]
drone1_world = wHb * drone1_body; % [4x4][4x6]
drone1_atti = drone1_world(1:3, :); 
    
fig1_ARM13 = plot3(gca, drone1_atti(1,[1 3]), drone1_atti(2,[1 3]), drone1_atti(3,[1 3]), ...
        '-ro', 'MarkerSize', 5);
fig1_ARM24 = plot3(gca, drone1_atti(1,[2 4]), drone1_atti(2,[2 4]), drone1_atti(3,[2 4]), ...
        '-bo', 'MarkerSize', 5);
fig1_payload = plot3(gca, drone1_atti(1,[5 6]), drone1_atti(2,[5 6]), drone1_atti(3,[5 6]), ...
        '-k', 'Linewidth', 3);
fig1_shadow = plot3(gca,0,0,0,'xk','Linewidth',3);

hold(gca, 'off');

%% Init. Data Fig.
fig2 = figure('pos',[800 400 800 600]);
subplot(2,3,1)
title('phi[deg]')
grid on;
hold on;
subplot(2,3,2)
title('theta[deg]')
grid on;
hold on;
subplot(2,3,3)
title('psi[deg]')
grid on;
hold on;
subplot(2,3,4)
title('X[m]')
grid on;
hold on;
subplot(2,3,5)
title('Y[m]')
grid on;
hold on;
subplot(2,3,6)
title('Zdot[m/s]')
grid on;
hold on;

%% Main Loop
commandSig(1) = -10.0*D2R; %phi
commandSig(2) = 10.0*D2R; %theta
commandSig(3) = 10.0*D2R; %psi
commandSig(4) = 1.0; %zdot
for i = 1:simulationTime/0.01
    %% Take a step
    drone1.AttitudeCtrl(commandSig);
    drone1.UpdateState();
	
	drone1_state = drone1.GetState();
    
    %% 3D Plot
    figure(1)
    wHb = [RPY2Rot(drone1_state(7:9))' drone1_state(1:3); 0 0 0 1];
    drone1_world = wHb * drone1_body;
    drone1_atti = drone1_world(1:3, :);
    
	set(fig1_ARM13, ...
        'XData', drone1_atti(1,[1 3]), ...
        'YData', drone1_atti(2,[1 3]), ...
        'ZData', drone1_atti(3,[1 3]));
    set(fig1_ARM24, ...
        'XData', drone1_atti(1,[2 4]), ...
        'YData', drone1_atti(2,[2 4]), ...
        'ZData', drone1_atti(3,[2 4]));
    set(fig1_payload, ...
        'XData', drone1_atti(1,[5 6]), ...
        'YData', drone1_atti(2,[5 6]), ...
        'ZData', drone1_atti(3,[5 6]));  
	set(fig1_shadow, ...
		'XData', drone1_state(1), ...
		'YData', drone1_state(2));
    
    %% Data Plot
    figure(2)
    subplot(2,3,1)
        plot(i/100,drone1_state(7)*R2D,'.');
    subplot(2,3,2)
        plot(i/100,drone1_state(8)*R2D,'.');    
    subplot(2,3,3)
        plot(i/100,drone1_state(9)*R2D,'.');
    subplot(2,3,4)
        plot(i/100,drone1_state(1),'.');
    subplot(2,3,5)
        plot(i/100,drone1_state(2),'.');
    subplot(2,3,6)
        plot(i/100,drone1_state(6),'.');
		
    drawnow;
    
    
    %% BREAK WHEN CRASH
    if (drone1_state(3) >= 0)
        h = msgbox('Crashed!!!', 'Error','error');
        break;
    end
end