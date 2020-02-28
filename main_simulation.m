clear all
close all
clc
%%




%%%%%%%%%%%%%%%%
%%% Setings %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plot_on = 1;
n_vehicles = 25;            % number of vehicles
type = 'IDM';               % Driving model (OVM/IDM)
v_0 = 10;                  % maximum initial velocity (m/s)
r = 200;                  % ring radius (m)
L = 5;                      % vehicles' length
eps = 0.001;                % linearization gradient step
dt = 0.01;                   % simulation time step
motion_pace = 1;            % fast forward (1 = normal) - for real-time visualization only
simulation_duration = 100;  % simulation duration [s]

data_frequency_scalar = 1; % the higher, the more data is recorded
std_friction_max = .01;
std_elevation_max = .0000;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%



A = zeros(2*n_vehicles);
x = zeros(2*n_vehicles,1);
a = zeros(n_vehicles,1);
vel_vector = zeros(n_vehicles,1);
pos_vector = zeros(n_vehicles,1);
hdw_vector = zeros(n_vehicles,1);
pos_history = zeros(simulation_duration/dt,n_vehicles);
vel_history = zeros(simulation_duration/dt,n_vehicles);
hdw_history = zeros(simulation_duration/dt,n_vehicles);
data = fopen('vehicles_data.csv','w+');
fprintf(data,'%s, %s, %s, %s, %s, %s, %s\n','ID', 'time', 'latitude', ...
                    'longitude', 'altitude', 'speed', 'friction');



% uniformly assume measurement uncertanities and properties
id_list = (1:n_vehicles);
std_friction =  rand(1, n_vehicles) * std_friction_max;
std_elevation = rand(1, n_vehicles) * std_elevation_max;
sampling_probability = data_frequency_scalar * rand(1, n_vehicles) * dt; % probability of sending a single measurement per second  


% build a random matrix from random values
rand_vector = rand(n_vehicles,1)*(2*pi*r);

% sort the values of random vector
rand_vector = sort(rand_vector,'descend');

% fill state matrix with initial random values
for i=1:n_vehicles
    x(2*i-1) = rand_vector(i);
    x(2*i) = rand()*v_0;
    
end
%% 


% fig_ring = figure('Position', [1200, 100, 500, 500]);
% fig_ring.Name = 'Visualization';
% % % fig_ring.Visible = 'off';
%     ax1 = polaraxes;
%     polarplot(0, r, 'o', 'LineWidth',2);
%     hold on 


% start simiulation!
for t=0:1:simulation_duration/dt

    for i=1:n_vehicles 
        j = i-1; % agent j is the leading agent
        if j == 0 % close the agents' chain
            j = n_vehicles;
            % obtain acceleration of the first agent from DRIVER model
            % (first agent is an exeption because it should be connected to the last agent)
            a(i) = DRIVER(type, x(2*i-1), x(2*j-1)+2*pi*r, x(2*i), x(2*j));
        else
            % obtain acceleration of the agents from DRIVER model
            a(i) = DRIVER(type, x(2*i-1), x(2*j-1), x(2*i), x(2*j));
        end
        
        % update positions and velocities
        x(2*i-1) = x(2*i-1) + x(2*i)*dt + (1/2)*a(i)*dt^2 ;
        x(2*i) = x(2*i) + a(i)*dt;
        
        vel_vector(i) = x(2*i);
        pos_vector(i) = x(2*i-1);
        if i == 1
            hdw_vector(i) = (x(2*n_vehicles-1)+2*pi*r) - x(1) - L;
        else
            hdw_vector(i) = x(2*(i-1)-1) - x(2*i-1) - L;
        end
        
        % record a measurement for agent i into the database
        normalized_pos = rem(pos_vector(i)/r, 2*pi);
        if rand() < sampling_probability(i)
            normalized_pos = rem(pos_vector(i)/r, 2*pi);
            z = get_measurement(normalized_pos, ...
                               std_friction(i), ...
                               std_elevation(i));
            
            fprintf(data,'%u, %.5f, %.8f, %.8f, %.8f, %.5f, %.5f\n', ...
                i, t*dt, r*cos(normalized_pos)/800000 + 40.861785, ...
                         r*sin(normalized_pos)/800000 - 77.836118, ...
                         z(2), vel_vector(i), z(1));
        end
        
        
        
    end    
    pos_history(t+1,:) = pos_vector;
    vel_history(t+1,:) = vel_vector;
    hdw_history(t+1,:) = hdw_vector;
 
%     ax1 = polaraxes('Parent', fig_ring);
%     r_vector = r*ones(agent_numbers,1);
%     polarscatter(ax1, pos_vector/r, r_vector, 'o', 'filled');
% 
%     
% 
% 
%         pause (dt/motion_pace);
%        ax1 = polaraxes('Parent', fig_ring);
%         hold (ax1, 'off')
%         
    
end

data = fclose(data); %close file

time = linspace(0,simulation_duration,simulation_duration/dt+1);


fig_pos = figure('Position', [50, 50, 800, 400]);
fig_pos.Name = 'Position';
    for i=1:n_vehicles
    plot(time,pos_history(:,i),'b-');
    hold on
    end
    xlim([0 simulation_duration])
    %ylim([0 simulation_duration*r])
    xlabel('time [s]')
    ylabel('positions [m]')
    grid on
    
fig_vel = figure('Position', [50, 580, 800, 400]);
fig_vel.Name = 'Velocities';
    for i=1:n_vehicles
    plot (time,vel_history(:,i),'r-');
    hold on
    end
    xlim([0 simulation_duration])
    %ylim([0 20])
    xlabel('time [s]')
    ylabel('velocity [m/s]')
    grid on
    
fig_hdw = figure('Position', [900, 580, 800, 400]);
fig_hdw.Name = 'Headways (Spacing)';
    for i=1:n_vehicles
    plot (time,hdw_history(:,i),'k-');
    hold on
    end
    xlim([0 simulation_duration])
    %ylim([0 20])
    xlabel('time [s]')
    ylabel('headway [m]')
    grid on
%% 
