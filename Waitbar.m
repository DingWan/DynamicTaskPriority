hwait = waitbar(0,'waitbar'); % set waitbar
q_n;
q_r;
q_s;
for i = 1:n  
    i
  str=['Total steps:',num2str(n),'.Current Proseccing step:',num2str(i)]; % waitbar parameters
  waitbar(i/n,hwait,str);
  %q_start = q0;
  %============ Get random start point for finish the loop ============
  %In general, you can generate N random numbers in the interval [a,b] 
  %with the formula r = a + (b-a).*rand(N,1).
  %q_start(2) = qmin(1) + (qmax(1)-qmin(1)).*rand(1,1);  
  %===== Problem: calculation time is unstable ========
    if i == 1 %|| i == 7*n %|| i == 7*n %|| i == 4*n
        q = q_s; % start point of robot arm
        FTstart = PR10.fkine(q); % get the parameters of TCP
        Pstart = [transpose(FTstart(1:3,4)),tr2eul(FTstart)];
        Pend =  [transpose(traj_1(1:3,4,i)),tr2eul(traj_1(:,:,i))];
        e = Pend - Pstart; % set e
    else
        Pstart =  [transpose(traj_1(1:3,4,i-1)), tr2eul(traj_1(:,:,i-1))];
        Pend =  [transpose(traj_1(1:3,4,i)), tr2eul(traj_1(:,:,i))];
        e = Pend - Pstart; % set e      
    end
% get the first jocobian matrix   
    Jacob = PR10.jacob0(q,'eul');
        aa=100+i;
    while norm(e)>0.0001 %when the TCP doesnt reach the end point yet
        aa=aa+100
        %%obs ball
        d0=handles.obstacle_position_R;
        x0=handles.obstacle_position_X;
        y0=handles.obstacle_position_Y;
        z0=handles.obstacle_position_Z; 
%compute the min distance between obstacle and each link,real time data are substituted into the equations   
    q1=q(1);q2=q(2);q3=q(3);q4=q(4);q5=q(5);q6=q(6);q7=q(7);q8=q(8);q9=q(9);q10=q(10);
    position1_q=eval(transpose(position1)); %first_joint_position
    position2_q=eval(transpose(position2)); %second_joint_position
    position3_q=eval(transpose(position3)); %third_joint_position
    position4_q=eval(transpose(position4)); %shoulder_joint_position
    position5_q=eval(transpose(position5)); %elbow_joint_position
    position6_q=eval(transpose(position6)); %wrist_joint_position
    d1=DistBetween2Segment(obs,obs,ini_point,position1_q);
    d2=DistBetween2Segment(obs,obs,position1_q,position2_q);
    d3=DistBetween2Segment(obs,obs,position2_q,position3_q);
    d4=DistBetween2Segment(obs,obs,position4_q,position5_q);
    d5=DistBetween2Segment(obs,obs,position5_q,position6_q);
    alld=[d1,d2,d3,d4,d5];
    [mins index]=min(alld); %get the minimum of diatance
%find the right case: which joint or rod is the nearest to the obstalce, vide supra
    switch index
      case{1}     
            if mins==DistBetween2Segment(obs,obs,ini_point,ini_point)
                num=1; % initial point
            else if mins==DistBetween2Segment(obs,obs,transpose(position1),transpose(position1))
                num=2; % position1
                else
                    num=3; %first rob of portal3
                end
            end
      case{2}
            if mins==DistBetween2Segment(obs,obs,transpose(position2),transpose(position2))
                num=4; % position2
            else if mins==DistBetween2Segment(obs,obs,transpose(position1),transpose(position1))
                num=2; % position1
                else
                    num=5; %second rob of portal3
                end
            end
      case{3}
            if mins==DistBetween2Segment(obs,obs,transpose(position2),transpose(position2))
                num=4; % position2
            else if mins==DistBetween2Segment(obs,obs,transpose(position3),transpose(position3))
                num=6; % position3
                else
                    num=7; %third rob of portal3
                end
            end
       case{4}
            if mins==DistBetween2Segment(obs,obs,transpose(position4),transpose(position4))
                num=6; % position4
            else if mins==DistBetween2Segment(obs,obs,transpose(position5),transpose(position5))
                num=8; % position5
                else
                    num=9; % rob(shoulder and elbow)
                end
            end
        case{5}
            if mins==DistBetween2Segment(obs,obs,transpose(position5),transpose(position5))
                num=8; % position5
            else if mins==DistBetween2Segment(obs,obs,transpose(position6),transpose(position6))
                num=10;% position6
                else
                    num=11; % rob(wrist and elbow)
                end
            end
        otherwise
            break;
    end
%get the funtion of the min limitation according to different cases;
    switch num
      case{1}
           q1=q(1);q2=q(2);q3=q(3);q4=q(4);q5=q(5);q6=q(6);q7=q(7);q8=q(8);q9=q(9);q10=q(10);
           gradient_distance=eval(gradient1);
           set(handles.closest_Link,'string','Origin Point');
           set(handles.closest_distance,'string',num2str(mins));
           mindis(n)=mins;
      case{2}
          q1=q(1);q2=q(2);q3=q(3);q4=q(4);q5=q(5);q6=q(6);q7=q(7);q8=q(8);q9=q(9);q10=q(10);
           gradient_distance=eval(gradient2);
           set(handles.closest_Link,'string','Portal3_Z_axis');
           set(handles.closest_distance,'string',num2str(mins));
           mindis(n)=mins;
      case{3}
           q1=q(1);q2=q(2);q3=q(3);q4=q(4);q5=q(5);q6=q(6);q7=q(7);q8=q(8);q9=q(9);q10=q(10);
           gradient_distance=eval(gradient3);
           set(handles.closest_Link,'string','Z_axis_and_X_axis_intersectionPoint');
           set(handles.closest_distance,'string',num2str(mins));
           mindis(n)=mins;
       case{4}
          q1=q(1);q2=q(2);q3=q(3);q4=q(4);q5=q(5);q6=q(6);q7=q(7);q8=q(8);q9=q(9);q10=q(10);
           gradient_distance=eval(gradient4);
           set(handles.closest_Link,'string','Portal3_X_axis');
           set(handles.closest_distance,'string',num2str(mins));
           mindis(n)=mins;
       case{5}
            q1=q(1);q2=q(2);q3=q(3);q4=q(4);q5=q(5);q6=q(6);q7=q(7);q8=q(8);q9=q(9);q10=q(10);
           gradient_distance=eval(gradient5);
           set(handles.closest_Link,'string','X_axis_and_Y_axis_intersectionPoint');
           set(handles.closest_distance,'string',num2str(mins));
           mindis(n)=mins;
       case{6}
            q1=q(1);q2=q(2);q3=q(3);q4=q(4);q5=q(5);q6=q(6);q7=q(7);q8=q(8);q9=q(9);q10=q(10);
           gradient_distance=eval(gradient6);
           set(handles.closest_Link,'string','Portal3_Y_axis');
           set(handles.closest_distance,'string',num2str(mins));
           mindis(n)=mins;
      case{7}
            q1=q(1);q2=q(2);q3=q(3);q4=q(4);q5=q(5);q6=q(6);q7=q(7);q8=q(8);q9=q(9);q10=q(10);
           gradient_distance=eval(gradient7);
           set(handles.closest_Link,'string','PA10_shoulder');
           set(handles.closest_distance,'string',num2str(mins));
           mindis(n)=mins;
      case{8}
            q1=q(1);q2=q(2);q3=q(3);q4=q(4);q5=q(5);q6=q(6);q7=q(7);q8=q(8);q9=q(9);q10=q(10);
           gradient_distance=eval(gradient8);
           set(handles.closest_Link,'string','PA10_first_link');
           set(handles.closest_distance,'string',num2str(mins));
           mindis(n)=mins;
       case{9}
           q1=q(1);q2=q(2);q3=q(3);q4=q(4);q5=q(5);q6=q(6);q7=q(7);q8=q(8);q9=q(9);q10=q(10);
           gradient_distance=eval(gradient9);
           set(handles.closest_Link,'string','PA10_elbow');
           set(handles.closest_distance,'string',num2str(mins));
           mindis(n)=mins;
       case{10}
           q1=q(1);q2=q(2);q3=q(3);q4=q(4);q5=q(5);q6=q(6);q7=q(7);q8=q(8);q9=q(9);q10=q(10);
           gradient_distance=eval(gradient10);
           set(handles.closest_Link,'string','PA10_second_link');
           set(handles.closest_distance,'string',num2str(mins));
           mindis(n)=mins;
       case{11}
           q1=q(1);q2=q(2);q3=q(3);q4=q(4);q5=q(5);q6=q(6);q7=q(7);q8=q(8);q9=q(9);q10=q(10);
           gradient_distance=eval(gradient11); 
           set(handles.closest_Link,'string','PA10_wrist');
           set(handles.closest_distance,'string',num2str(mins));
           set(handles.closest_distance,'string',num2str(mins));
           mindis(n)=mins;
        otherwise
            break;
    end
        %% Gradient Projection Method
        %-------------- Gradient function of joint limit functionm Hq----------
        for num_q = 1:length(q)
          gradHq(num_q,1) =  ((qmax(num_q) - qmin(num_q))^2*(2*q(num_q)- qmax(num_q)- qmin(num_q)))/((qmax(num_q) -q(num_q))^2*(q(num_q) - qmin(num_q))^2)/4;
        end
        %-----PA10----singularity---gradient
        gradM=transpose([0,0,0,0,-sin(2*q2),0,-sin(2*q4),0,0,0]);
        %-----PA10----obtacle distance---gradient, vide supra
        %----------------------------------------------------------------------
        if i == 1 && norm(e) > 0.01
           factor = 0.1;
        else
           factor = 1;
        end
      %% norm of 3 gradients
       gradHqnorm=gradHq/norm(gradHq);
       gradMnorm=gradM/norm(gradM);
       %gradWnorm=gradW/norm(gradW);
       if norm(gradient_distance)==0
          gradient_distancenorm=0;
       else
          gradient_distancenorm=gradient_distance/norm(gradient_distance);
       end;      
      %% GPM(+task priority)
       %----first task obstacle avoidance-----
       Jacob1=Jacob;
        if norm((eye(numel(q)) - pinv(Jacob1) *Jacob1 ) * transpose(gradient_distancenorm)) ~= 0
            k = alpha * sqrt(norm (pinv(Jacob1) * transpose(e)))/ ...
            (sqrt(norm (pinv(Jacob1) * transpose(e)))+sqrt(norm((eye(numel(q)) - pinv(Jacob1) *Jacob1 ) * transpose(gradient_distancenorm))));
        else
            k =0;
        end
        qhdot1 = k*(eye(numel(q)) - pinv(Jacob1) *Jacob1 ) * transpose(gradient_distancenorm);
        qsdot1 = pinv(Jacob1) * transpose(e);
        delta_q1 =  qsdot1 + qhdot1;
        q1 = q + factor * transpose(delta_q1);
        Jacob2 = PR10.jacob0(q1,'eul');  
        %--second--joint-limitation------
        FTstart = PR10.fkine(q1);
        Pstart = [transpose(FTstart(1:3,4)),tr2eul(FTstart)];
        Pend =  [transpose(traj_1(1:3,4,i)),tr2eul(traj_1(:,:,i))];
        e2 = Pend - Pstart; 
        if norm((eye(numel(q1)) - pinv(Jacob2) *Jacob2 ) * gradHqnorm) ~= 0
            k = alpha * sqrt(norm (pinv(Jacob2) * transpose(e2)))/ ...
            (sqrt(norm (pinv(Jacob2) * transpose(e2)))+sqrt(norm((eye(numel(q1)) - pinv(Jacob2) *Jacob2 ) * gradHqnorm)));
        else
            k =0;
        end
        P=eye(numel(q1)) - pinv(Jacob1) *Jacob1;
        qhdot2 = k*(eye(numel(q1)) - pinv(Jacob2) *Jacob2 ) * gradHqnorm;
        qsdot2 = pinv(Jacob2) * transpose(e);
        delta_q2 =  qsdot2 + qhdot2;
        q2 = q1 + factor * transpose(delta_q2);
        Jacob3 = PR10.jacob0(q2,'eul') ; 
       %--third--joint-limitation------
        FTstart = PR10.fkine(q2);
        Pstart = [transpose(FTstart(1:3,4)),tr2eul(FTstart)];
        Pend =  [transpose(traj_1(1:3,4,i)),tr2eul(traj_1(:,:,i))];
        e3 = Pend - Pstart;     
        if norm((eye(numel(q2)) - pinv(Jacob3) *Jacob3) * gradMnorm) ~= 0
            k = alpha * sqrt(norm (pinv(Jacob3) * transpose(e3)))/ ...
            (sqrt(norm (pinv(Jacob3) * transpose(e3)))+sqrt(norm((eye(numel(q2)) - pinv(Jacob3) *Jacob3 ) * gradMnorm)));
        else
            k =0;
        end
        qhdot3 = k*(eye(numel(q2)) - pinv(Jacob3) *Jacob3 ) * gradHqnorm;
        qsdot3 = pinv(Jacob3) * transpose(e3);
        delta_q3 =  qsdot3 + qhdot3;
        q3 = q2 + factor * transpose(delta_q3) ;      
        Jacob3 = PR10.jacob0(q3,'eul');         
        q=q3;
        FTP = PR10.fkine(q);
        P =  [transpose(FTP(1:3,4)),tr2eul(FTP)];
        e = Pend - P;
        %============ Initial orientation might have P(i)=-P(i-1)= 3.1416 ============
        %============ Because of the error might equal to -/+0.00001 make ============
        %============ the calculation of "tr2eul(FTP)" cause P(i)=-P(i-1)= 3.1416 ============
        for j = 4:6
           if abs(e(j)-2*pi)<1e03
               e(j)=0;
           end
        end
       % ================================================================

        % plot the connection between the approaching line
%         xP = [Pstart(1) Pend(1) P(1)]; yP = [Pstart(2) Pend(2) P(2)]; zP = [Pstart(3) Pend(3) P(3)];
%         plot3(xP(1:2),yP(1:2),zP(1:2),'g-.','MarkerSize',30); hold on;
%         plot3(xP(2:3),yP(2:3),zP(2:3),'r-.','MarkerSize',30); 
%         if i == 1
%              PR9.plot(q,'workspace',[-2 2 -2 2 -0.5 2],'scale',0.5);
%         end
%         norm(e)
        q1_data = q(1);
        q2_data = q(2);
        q3_data = q(3);
        q4_data = q(4);
        q5_data = q(5);
        q6_data = q(6);
        q7_data = q(7);
        q8_data = q(8);
        q9_data = q(9);
        q10_data = q(10);
%       display
        set(handles.q1_data_output,'string',num2str(q1_data));
        set(handles.q2_data_output,'string',num2str(q2_data));
        set(handles.q3_data_output,'string',num2str(q3_data));
        set(handles.q4_data_output,'string',num2str(q4_data));
        set(handles.q5_data_output,'string',num2str(q5_data));
        set(handles.q6_data_output,'string',num2str(q6_data));
        set(handles.q7_data_output,'string',num2str(q7_data));
        set(handles.q8_data_output,'string',num2str(q8_data));
        set(handles.q9_data_output,'string',num2str(q9_data));
        set(handles.q10_data_output,'string',num2str(q10_data));
    end 
    q_plot(i,:) = q;
    if i==1
       P = Pend; 
    end
    p_current(i,:) = P;% plot the error compared with the trajectory
    err(i,:) = e;% plot the error
    if q(1) > qmax(1) || q(2) > qmax(2) || q(3) > qmax(3) ||...
            q(1) < qmin(1) || q(2) < qmin(2) || q(3) < qmin(3) ||...
            q(4) > qmax(4) || q(5) > qmax(5) || q(6) > qmax(6) ||...
            q(4) < qmin(4) || q(5) < qmin(5) || q(6) < qmin(6) ||...
            q(7) > qmax(7) || q(8) > qmax(8) || q(9) > qmax(9) ||...
            q(7) < qmin(7) || q(8) < qmin(8) || q(9) < qmin(9) ||...
            q(10) > qmax(10) || q(10) < qmin(10)  ...
            || det(Jacob*Jacob') == 0||d1 < d0||d2 < d0||d3 < d0||d4 < d0||d5 < d0
        display('Exceed the boundary, Calculation process stopped');
        error = 1; break;        
    end 
    if i == n %|| i == 2*n || i == 3*n || i == 4*n
    error = 0;
    end
end
close(hwait);