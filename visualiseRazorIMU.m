% Visualise attitude/orientation of the Razor IMU
%
% Date: 23/10/2017
% Author: Sean Anderson

% IMU data acquisition
clear all % clear all variables from the workspace
delete(instrfindall) % delete all communication objects

%% set up serial port to read the IMU
% Define a serial communication object, s, in Matlab
% NOTE: enter your IMU COM port number here!!!
s = serial('com5', 'BaudRate', 57600);
fopen(s);
disp('Serial port initiated...');

% start IMU streaming
fprintf(s, '#o1');                      % this starts the IMU streaming data
pause(2);                               % pause for 2 seconds for transmission
disp('Serial port streaming...');       % display instruction at the command line
fprintf(s, '#ot');                      % this ensures the data is in text format
disp('Serial port streaming text...');  % display instruction at the command line
pause(2);                               % pause for 2 seconds
flushinput(s);                          % flush the serial port buffer

% setup experiment details
maxT = 30;          % time length of data collection experiment
Fs = 50;            % sample rate of accelerometer - about 50 Hz
dt = 1/Fs;          % sample time
N = Fs*maxT + 100;  % number of data samples to record
t = [0:N-1]'*dt;    % time index for plotting
yaw = zeros(N,1);   % x-axis acceleration
pitch = zeros(N,1); % y-axis acceleration
roll = zeros(N,1);  % z-axis acceleration
imuLabels =  '  acc_x  acc_y  acc_z  mag_x  mag_y  mag_z  gyro_x  gyro_y  gyro_z';

%% Setup IMU visualisation
clf;
figure(1);
% Use hold on and hold off to plot multiple cubes
hold on;
% Call the function to plot a cube with dimension of X, Y, Z, at point [x,y,z].
%cube_plot([1,1,1],1,1,1,'r');

origin = [0,0,0];
X = 1;
Y = 1;
Z = 1;
color = 'r';

% cube plot
ver = [1 2 0;
    0 2 0;
    0 2 0.5;
    1 2 0.5;
    0 0 0.5;
    1 0 0.5;
    1 0 0;
    0 0 0];

%  Define the faces of the unit cubic
fac = [1 2 3 4;
    4 3 5 6;
    6 7 8 5;
    1 2 8 7;
    6 7 1 4;
    2 3 5 8];
cube = [ver(:,1)*X+origin(1),ver(:,2)*Y+origin(2),ver(:,3)*Z+origin(3)];
patch('Faces',fac,'Vertices',cube,'FaceColor',color);


% Figure configurations

% Define the range of x-axis, y-axis, and z-axis in form of
% [xmin,xmax,ymin,ymax,zmin,zmax].
% axis([0,1,0,1,0,1]);
% Set the axis with equal unit.
axis equal;

% Show grids on the plot
grid on;

% Set the lable and the font size
xlabel('X','FontSize',18);
ylabel('Y','FontSize',18)
zlabel('Z','FontSize',18)

% Control the ticks on the axises
h = gca; % Get the handle of the figure

% Set the color as transparient
material metal
alpha('color');
alphamap('rampup');

% Set the view point
view(30,30);
h.XLim = [-3 3];
h.YLim = [-3 3];
h.ZLim = [-3 3];


%%


% loop and read data until maxT seconds elapse
disp('Serial Port Recording NOW!!!');
tic; % start a timer
k=1; % initialise sample counter
flushinput(s); % flush the serial port buffer
while (toc < maxT)
       
    % only try reading data if data is available in the port
    if(s.bytesavailable>10)
        
        % get data from serial object (USB port)
        chk = fgetl(s);
        
        % only read data if first character is a # and has 11 spaces in total
        if chk(1) == '#' && length(strfind(chk,'.')) == 3
            
            % define IMU output
            out = chk;
            
            % identify spaces in the string and use as an index
            idx = strfind(out,',');
            
            % read in IMU data and convert from string to a number
            yaw(k) = str2double(out(6:idx(1)-1));
            pitch(k) = str2double(out(idx(1)+1:idx(2)-1));
            roll(k) = str2double(out(idx(2)+1:end-1));
            
            % display IMU output at the command line
            display(out);
            
            % clear axis
            cla;
            
            % rotation matrix
            dcm = angle2dcm(deg2rad(yaw(k)), deg2rad(roll(k)), deg2rad(pitch(k)));
            P = ver*dcm;
            
            % plot cube
            cube = [P(:,1)*X+origin(1),P(:,2)*Y+origin(2),P(:,3)*Z+origin(3)];
            patch('Faces',fac,'Vertices',cube,'FaceColor',color);
            
            % Set the view point
            view(30,30);
            
            % update figure
            drawnow;
            
            % increment counter
            k = k+1;
        end
    end
end

% close the COM port when finished - IMPORTANT!
fclose(s);
disp('ending');

% plot time-series data
figure;
plot(t,yaw,'b'); hold on;
plot(t,roll,'.-r');
plot(t,pitch,'g');
legend('Yaw','Pitch','Roll')
xlabel('Time (s)');
ylabel('IMU Output');
title('IMU Signals on 3-Axes')
%

% the end



