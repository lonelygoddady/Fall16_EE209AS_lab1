%This lab makes use of matlab rvcrobot toolkit by peter corke
%Please make sure you have install the rvcrobot tookit before excute 
%this program
format short;

%% Modified DH Parameter Initilization 
a = [0,-0.3,-3,-2];
alpha = [0,pi/2,0,0];
d = [1,0,0,0];
Theta = [0;-pi/4;pi/4;0]; %(-5,0,3)

%% Define robot link components
L0 = Link([0,d(1),a(1),alpha(1)],'modified');
L1 = Link([0,d(2),a(2),alpha(2)],'modified');
L2 = Link([0,d(3),a(3),alpha(3)],'modified');
L3 = Link([0,d(4),a(4),alpha(4)],'modified');

%Create the robot for later drawing
bot = SerialLink([L0 L1 L2 L3], 'name', 'my robot');

%% Forward Kinematics
%define transform matrix for each joint
T0 = transfer_m(a(1),alpha(1),d(1),Theta(1));
T1 = transfer_m(a(2),alpha(2),d(2),Theta(2));
T2 = transfer_m(a(3),alpha(3),d(3),Theta(3));
T3 = transfer_m(a(4),alpha(4),d(4),Theta(4));

%find the state (transform matrix) of the end effector 
T_EF2base = T0*T1*T2*T3;
EF = T_EF2base*[-0.5;0;0;1;]; %calcluate the end effector coordinate in world frame

%% Inverse Kinematics via Pseudo Inverse Jacobian method 
P_d = [-0.5; 0; 5; 1]; %desired point coordinate
P_initial = EF; %initial guess coordinate
theta_initial = Theta; %q values 
limits = 10; %iteration count varable 

P_pred = zeros(limits,3); %predication coordinates array
Theta_path = zeros(limits,4); %predication q values array

for i = 1:limits
    %Redefine transform matrix each time
    T_0 = transfer_m(a(1),alpha(1),d(1),theta_initial(1)); 
    T_1 = transfer_m(a(2),alpha(2),d(2),theta_initial(2));
    T_2 = transfer_m(a(3),alpha(3),d(3),theta_initial(3));
    T_3 = transfer_m(a(4),alpha(4),d(4),theta_initial(4));
    
    %recalculate the predicated coordinates using FK
    P_temp = (T_0*T_1*T_2*T_3) * [-0.5;0;0;1;]; 
    P_pred(i,:) = P_temp(1:3);
    
    %error calculation
    P_er = [P_d - P_temp; 0; 0]; 
    
    %Jacobian matrix and pseudo-inverse Jacobian matrix calculation
    T_jacb = bot.jacob0(theta_initial); 
    T_jacb_pin = pinv(T_jacb); 
    
    %delta theta(q) calculation and theta(q) value increment
    theta_delta = T_jacb_pin*P_er; 
    Theta_path(i,:) = theta_initial;
    theta_initial = theta_initial + theta_delta; 
end 

round_thr = 1e-04;

for i = 1:numel(P_pred(:,1)) %%Loop to round inifinitesmall value to 0 
   if abs(P_pred(i,1)) < round_thr;
       P_pred(i,1) = 0;
   end 
   if abs(P_pred(i,2)) < round_thr;
       P_pred(i,2) = 0;
   end 
   if abs(P_pred(i,2)) < round_thr;
       P_pred(i,2) = 0;
   end 
end

%plot the IK trajectory from (-5,0,3) to (-0.5,0,5)
plot3(P_pred(:,1),P_pred(:,2),P_pred(:,3),'--'); 
title('IK trajectory');
xlabel('X');
ylabel('Y');
zlabel('Z');

%% Modified method for trajectory 
num_points = 10;  %straight line points
P_start = [-4.9213, 0, 3.1213];  %the exact coordinates when theta is [0;-pi/4;pi/4;0]
P_end = [-0.5, 0 ,5]; 

ST_line = [linspace(P_start(1), P_end(1), num_points); ... %breaking straight line into 10 points
    linspace(P_start(2), P_end(2), num_points); ...
    linspace(P_start(3), P_end(3), num_points);]';
ST_line_homo = [ST_line, ones(1,num_points)'];
point_temp = zeros(1,4);

it = 3;  %iteration times
Guess = [0;-pi/4;pi/4;0]; %initial guess
counter = 1; %counting how mant prediction points generated
error_thres = 5e-04; %error threshold to stop iteration
Err = 1; %a value larger than error_thres

for j = 1:num_points
    while abs(sum(Err)) > error_thres
        %update transform function of each joint
        Tm_0 = transfer_m(a(1),alpha(1),d(1),Guess(1)); 
        Tm_1 = transfer_m(a(2),alpha(2),d(2),Guess(2));
        Tm_2 = transfer_m(a(3),alpha(3),d(3),Guess(3));
        Tm_3 = transfer_m(a(4),alpha(4),d(4),Guess(4));
        
        %FK world frame coordinate calculation 
        point_temp = (Tm_0*Tm_1*Tm_2*Tm_3)*[-0.5;0;0;1];
        
        %coordinate error calculation
        Err = [(ST_line_homo(j,:)-point_temp'), 0, 0];
        
        %jacobian and pesudo-jacobian
        Jacb_m = bot.jacob0(Guess); 
        Jacb_m_pin = pinv(Jacb_m);
        
        %theta delta and update theta angles
        Guess_delta = Jacb_m_pin*Err';
        Guess = Guess + Guess_delta;
        points_predication(counter,:) = point_temp;
        counter = counter + 1; 
    end 
    Err = 1;
end

for i = 1:numel(points_predication(:,1)) %%Loop to round inifinitesmall value to 0 
   if abs(points_predication(i,1)) < round_thr;
       points_predication(i,1) = 0;
   end 
   if abs(points_predication(i,2)) < round_thr;
       points_predication(i,2) = 0;
   end 
   if abs(points_predication(i,2)) < round_thr;
       points_predication(i,2) = 0;
   end 
end

%Plot IK method trajectory before and after modification 
figure;
plot3(P_pred(:,1),P_pred(:,2),P_pred(:,3), 'red', ...
     points_predication(:,1),points_predication(:,2),points_predication(:,3), 'blue');
title('IK trajectory before and after modification');
xlabel('X');
ylabel('Y');
zlabel('Z');