axis([0 1000 0 1000]);            % To set the window size to 1000
DELAY=0.5;
DELAY1=0.45;
[x1,y1] = ginput(2);              % To take input from user for line 1
line(x1,y1,'color','black');      % draw the line (Road 1)
hold on ; 
[x2,y2] = ginput(2);              % To take input from user for line 2
line(x2,y2,'color','black');      % draw the line (Road 2)
rnd_speed1=randi([10,20],1,1);
dist_first_line=sqrt((x1(2)-x1(1))^2+(y1(2)-y1(1))^2);     % Distance between 2 points is calculated
speed_new1=round(dist_first_line/rnd_speed1);               % This is the new speed for 1st vehicle
rnd_speed2=randi([10,20],1,1);
dist_second_line=sqrt((x2(2)-x2(1))^2+(y2(2)-y2(1))^2);     % Distance between 2 points is calculated
speed_new2=round(dist_first_line/rnd_speed1);               % This is the new speed for 2nd vehicle
point1=linspace(x1(1),x1(2),speed_new1);  % Random speed is given by taking new speed number of points in a linspace
point2=linspace(y1(1),y1(2),speed_new1);
point3=linspace(x2(1),x2(2),speed_new2);
point4=linspace(y2(1),y2(2),speed_new2);
first_vehicle=plot(point1,point2,'s','MarkerFaceColor','red');   % Plot first vehicle on road 1
second_vehicle=plot(point3,point4,'o','MarkerFaceColor','blue'); % Plot second vehicle on road 2
title('V2V connectivity simulation');                            % Title is given to the figure
for i=1:1000
    
    for k = 1:speed_new1                   % for all the values in linspace
         first_vehicle.XData = point1(k);  %first vehicle's x co-ordinate
         first_vehicle.YData = point2(k);  %first vehicle's y co-ordinate
         second_vehicle.XData = point3(k); %second vehicle's x co-ordinate
         second_vehicle.YData = point4(k); %second vehicle's y co-ordinate
         plot(point1(k),point2(k),point3(k),point4(k));
         vehicle_dist=[point1(k),point2(k);point3(k),point4(k)]; % Calculate the Euclidian distance between two vehicle's positions
         distance1= pdist(vehicle_dist,'euclidean'); 
          if distance1<=100                % If the distance is within 200 m
            line1=plot([point1(k),point3(k)],[point2(k),point4(k)],'--','color','green');  % Show connectivity between two vehicles
            pause(0.3);                  % Delay of 0.45
            set(line1,'Visible','off');     % Visibility property of line is set to 'off'
          end
          pause(DELAY1);                     % Delay of 0.5
         
    end
        set(first_vehicle,'Visible','off'); % Visibility property of first vehicle's position is set to 'off'
        set(second_vehicle,'Visible','off'); %Visibility property of second vehicle's position is set to 'off'
   
end

hold off;

