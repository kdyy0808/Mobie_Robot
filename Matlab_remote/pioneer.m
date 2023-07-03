%clear;
vrep=remApi('remoteApi');
vrep.simxFinish(-1);

clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);


%원래 무게 : 1.6 e01
%case3 무게 : 5.12 e02
test_case = 3;
sim_time = 140;
stopcheck = 75;
debug_mode = false;
if test_case==1
    target_v = 5.1282;%load 2kg
elseif test_case==2
    target_v = 5.1282;%0.6
    sim_time = 130;
    sim_time_case2_show = 62; % load 30kg
else
    target_v = 5.1282;
    sim_time = 130;
    sim_time_case3_show = 70;% load 50
end

Lookahead_Distance = 0.2*target_v;
b = 0.281;

size_factor= 2;


% centers_x=[]; centers_y=[];
% centers_x2=[]; centers_y2=[];
% centers_x3=[]; centers_y3=[];

if (clientID>-1)
    disp('Connected')
    %Handle setting
    [~,left_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);
    [~,right_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);
    [~,Pioneer]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_blocking);
    
    [~,Pioneer_front_point]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_connection4',vrep.simx_opmode_blocking);
    [~,Pioneer_center_point]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_connection1',vrep.simx_opmode_blocking);
    [~,L_point]=vrep.simxGetObjectHandle(clientID,'L_point',vrep.simx_opmode_blocking);

    [~,position]=vrep.simxGetObjectPosition(clientID, left_Motor,-1,vrep.simx_opmode_blocking);
    [~,position2]=vrep.simxGetObjectPosition(clientID, Pioneer,-1,vrep.simx_opmode_blocking);

    %Other Code
    [~]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,0.1,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,0.1,vrep.simx_opmode_blocking);

    %sensor cammer 
    %%[~,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor,vrep.simx_opmode_streaming);
    %%[~,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,1,vrep.simx_opmode_streaming);
    
    sim_time=10000;
    for i=1:sim_time
        
       %tic
       %%[~,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor,vrep.simx_opmode_buffer);
       %%[~,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,1,vrep.simx_opmode_buffer);
       
       %[~,position2]=vrep.simxGetObjectPosition(clientID, Pioneer,-1,vrep.simx_opmode_blocking);
       
       [~,position_front_point]=vrep.simxGetObjectPosition(clientID, Pioneer_front_point,-1,vrep.simx_opmode_blocking);
       [~,position_center_point]=vrep.simxGetObjectPosition(clientID, Pioneer_center_point,-1,vrep.simx_opmode_blocking);
       
       [~,left_motor_force]=vrep.simxGetJointForce(clientID,left_Motor,vrep.simx_opmode_blocking);
       [~,right_motor_force]=vrep.simxGetJointForce(clientID,right_Motor,vrep.simx_opmode_blocking);
       
       Vec = position_front_point - position_center_point;
       
       %%figure(2)
       %%imshow(image)

%        disp("center x = ")
%        disp(position_center_point(1))
%        disp("center y = ")
%        disp( position_center_point(2))

       [x1,y1,x2,y2,x3,y3,x4,y4,count] = calc_Lpoint(position_center_point(1), position_center_point(2),Lookahead_Distance);

       Vec1 = [x1 y1 0]-position_center_point;
       Vec2 = [x2 y2 0]-position_center_point;
       if count>2
            Vec3 = [x3 y3 0]-position_center_point;
            Vec4 = [x4 y4 0]-position_center_point;
       end


       C1 = dot(Vec,Vec1);
       C2 = dot(Vec,Vec2);
       if count>2
           C3 = dot(Vec,Vec3);
           C4 = dot(Vec,Vec4);
           if debug_mode == true
               disp(C1)
               disp(C2)
               disp(C3)
               disp(C4)
           end
           [~,I] = max([C1,C2,C3,C4]);
       else
            if C1>C2
                I=1;
            else
                I=2;
            end
           %[M,I] = max(C1,C2);
       end

        
      
       %pause(0.1);
        if I==1
            [~]=vrep.simxSetObjectPosition(clientID,L_point,-1,[x1 y1 0],vrep.simx_opmode_blocking);
            L_point_x = x1;
            L_point_y = y1;
        elseif I==2
            [~]=vrep.simxSetObjectPosition(clientID,L_point,-1,[x2 y2 0],vrep.simx_opmode_blocking);
            L_point_x = x2;
            L_point_y = y2;
        elseif I==3
            [~]=vrep.simxSetObjectPosition(clientID,L_point,-1,[x3 y3 0],vrep.simx_opmode_blocking);
            L_point_x = x3;
            L_point_y = y3;
            
        else %%%%%%%4
            [~]=vrep.simxSetObjectPosition(clientID,L_point,-1,[x4 y4 0],vrep.simx_opmode_blocking);
            L_point_x = x4;
            L_point_y = y4;
        end

       [~,reletive_point]=vrep.simxGetObjectPosition(clientID, L_point,Pioneer_center_point,vrep.simx_opmode_blocking);

%        [~,test_point] = vrep.simxGetObjectPosition(clientID,Pioneer_front_point,Pioneer_center_point, vrep.simx_opmode_blocking);
%        disp(["test:", test_point(1), test_point(2)])

       % 잘못 계산되는줄 알았으나 아니었음
%        gradient = Vec(2)/Vec(1);
%        bias=L_point_y - gradient*L_point_x;
% 
%        gradient2 = -1/gradient;
%        bias2 = position_center_point(2) - gradient2*position_center_point(1);
% 
%        inter_point_x = -(bias - bias2)/(gradient-gradient2);
%        inter_point_y = gradient * inter_point_x + bias;
% 
%        yl = sqrt((inter_point_x - position_center_point(1))^2 + (inter_point_y - position_center_point(2))^2);

       yl = reletive_point(2);
       if debug_mode == true
           disp(["YL", yl])
       end

       R_track = Lookahead_Distance^2/(2*yl);
       v_l = target_v*(1-b/(2*R_track));
       v_r = target_v*(1+b/(2*R_track));

       if abs(v_l-v_r)>3
            disp("errrrrrr")
       end
        
       if debug_mode == true
        disp(["left",v_l])
        disp(["right",v_r])
       end

       [~]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,v_l,vrep.simx_opmode_blocking);
       [~]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,v_r,vrep.simx_opmode_blocking);
       
%        [~]=vrep.simxSetJointForce(clientID,left_Motor,v_l*100000,vrep.simx_opmode_blocking);
%        [~]=vrep.simxSetJointForce(clientID,right_Motor,v_r*100000,vrep.simx_opmode_blocking);
       velocity(i) = v_l;

       if test_case==1
            centers_x(i) = position_center_point(1);
            centers_y(i) = position_center_point(2);
            left_motor_forces1(i) = left_motor_force;
            right_motor_forces1(i) = right_motor_force;
       elseif test_case==2
            centers_x2(i) = position_center_point(1);
            centers_y2(i) = position_center_point(2);
            left_motor_forces2(i) = left_motor_force;
            right_motor_forces2(i) = right_motor_force;
       else
           centers_x3(i) = position_center_point(1);
           centers_y3(i) = position_center_point(2);
           left_motor_forces3(i) = left_motor_force;
           right_motor_forces3(i) = right_motor_force;
       end

       error=calc_error(position_center_point(1),position_center_point(2));
       if test_case == 1
           errors(i) = error;
       elseif test_case == 2
           errors2(i) = error;
       else
           errors3(i) = error;
       end
       
       if i>stopcheck
           if position_center_point(1) < 0 && position_center_point(2) >3.5
               break
           end
       end
       
       

       %[~, linearVelocity, angularVelocity]=vrep.simxGetObjectVelocity(clientID,left_Motor,vrep.simx_opmode_blocking);
       %toc
       if debug_mode == true
        disp(i)
       end
    end
    sim_time = i;
    
    [~]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,0,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,0,vrep.simx_opmode_blocking);
    hold off
    figure(1)
    plot([1:sim_time],velocity(1:sim_time))


    title('Motor Velocity')
    xlabel('Time(s)')
    ylabel('')

    figure(3)
    hold on
    grid on
    if ~isempty(centers_x)
        plot( centers_x(1:length(centers_x)), centers_y(1:length(centers_x)),'b-')
    end

    if ~isempty(centers_x2)
        plot( centers_x2(1:length(centers_x2)), centers_y2(1:length(centers_x2)),'r-.')
    end

    if ~isempty(centers_x3)
        plot( centers_x3(1:length(centers_x3)), centers_y3(1:length(centers_x3)),'g-')
    end

   

    %plot(x,y,'k--')
    %%%%MPC result Plot
    %plot(q1_data,q2_data,'m-')

    theta = linspace(0,2*pi);
    t = linspace(0,130,131);
    plot(size_factor*sin(0.1*t), size_factor*2*sin(0.05*t),'k--')

    axis equal
    title('Trajectory')
    xlabel('X position(m)')
    ylabel('Y position(m)')
    xlim([-5 5]);
    ylim([-5 5]);
    h=legend("Pure Pursuit(Load = 2kg)", ...
        "Pure Pursuit(Load = 30kg)", ...
        "Pure Pursuit(Load = 50kg)", ...
        "Reference");    set(h,'fontsize',10);

    %"MPC", ...
    hold off
    
    figure(4)
    hold on
    grid on

    

    plot([1:length(left_motor_forces1)],(abs(left_motor_forces1(1:length(left_motor_forces1)))+abs(right_motor_forces1(1:length(left_motor_forces1))))/2,'b-')
    if ~isempty(centers_x2)
        plot([1:length(left_motor_forces2)],(abs(left_motor_forces2(1:length(left_motor_forces2)))+abs(right_motor_forces2(1:length(left_motor_forces2))))/2,'r-.')
    end
    if ~isempty(centers_x3)
        plot([1:length(left_motor_forces3)],(abs(left_motor_forces3(1:length(left_motor_forces3)))+abs(right_motor_forces3(1:length(left_motor_forces3))))/2,'g-')
    end
    h=legend("Pure Pursuit(Load = 2kg)", ...
        "Pure Pursuit(Load = 30kg)", ...
        "Pure Pursuit(Load = 50kg)");
    title('Average Motor Force')
    xlabel('Time(s)')
    ylabel('Nm')
    ylim([0 3.0]);
    
    figure(5)
    hold on
    grid on

    plot([1:length(errors)],errors,'b-')
    if ~isempty(centers_x2)
        plot([1:length(errors2)],errors2,'r-.')
    end
    if ~isempty(centers_x3)
        plot([1:length(errors3)],errors3,'g-')
    end
    h=legend("Pure Pursuit(Load = 2kg)", ...
        "Pure Pursuit(Load = 30kg)", ...
        "Pure Pursuit(Load = 50kg)");
    title('Tracking error')
    xlabel('Time(s)')
    ylabel('Error(m)')

    [~]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,0,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,0,vrep.simx_opmode_blocking);
    
    vrep.simxFinish(-1);
end

vrep.delete();
    