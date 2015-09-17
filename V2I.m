axis([0 1000 0 1000]);                            % To set the window size to 1000
DELAY=0.5;
DELAY1=0.45;
[a,b]=ginput(5);                                  % Take input from user
line(a,b,'color','black'); hold on;               % Draw a line connecting the points taken as input from user
rnd_speed=randi([10,20],1,1);                     % Random speed between 10-20 m/s is given to the vehicle
dist_whole=sqrt((a(2)-a(1))^2+(b(2)-b(1))^2);     % Distance between 2 points is calculated
speed_new=round(dist_whole/rnd_speed);           
x=linspace(a(1),a(2),speed_new);                  % Random speed is given by taking speed_new points in linspace
y=linspace(b(1),b(2),speed_new);
u=linspace(a(2),a(3),speed_new);
v=linspace(b(2),b(3),speed_new);
w=linspace(a(3),a(4),speed_new);
z=linspace(b(3),b(4),speed_new);
q=linspace(a(4),a(5),speed_new);
r=linspace(b(4),b(5),speed_new);
arr1=[x u w q];                                   % Array of all 'x' co-ordinates of 4 lines
arr2=[y v z r];                                   % Array of all 'y' co-ordinates of 4 lines
[xcord1,ycord1]=ginput(1);                        % Select a point for RSU 1
text(xcord1,ycord1,'RSU 1 ','HorizontalAlignment','right');
[xcord2,ycord2]=ginput(1);                        % Select a point for RSU 2
text(xcord2,ycord2,'RSU 2 ','HorizontalAlignment','right');
plot(xcord1,ycord1,'o','MarkerFaceColor','blue'); % RSU 1 plotted on the figure window
plot(xcord2,ycord2,'o','MarkerFaceColor','blue'); % RSU 2 plotted on the figure window
p=plot(x,y,'square','MarkerFaceColor','green','MarkerSize',5);   % Vehicle moving along the road
title('V2I connectivity');                        % Title is given to the figure
for i=1:1000
    for j=1:length(arr1)-1
     
           p.XData = arr1(j);               % X co-ordinate for that particular road segment
           p.YData = arr2(j);               % Y co-ordinate for that particular road segment
           first_dist=[xcord1,ycord1;arr1(j),arr2(j)]; % Take euclidian distance between vehicle's position on the road and RSU 1
           distance1=pdist(first_dist,'euclidean');
           second_dist=[xcord2,ycord2;arr1(j),arr2(j)]; % Take euclidian distance between vehicle's position on the road and RSU 2
           distance2=pdist(second_dist,'euclidean');
           
           pause(DELAY1);
           if distance1<=100                           % if distance between RSU 1 and vehicle's position < 100 m
                   line1=plot([xcord1,arr1(j)],[ycord1,arr2(j)],'--','color','green'); % Show connectivity to RSU 1
                   range1=plot([arr1(j),arr1(j+1)],[arr2(j),arr2(j+1)],'color','green');          % plot the points for given line space. Hence moving vehicle effect
                   pause(0.3);                     
                   set(line1,'Visible','off');                 % Visibility property is set to ='off'
                   set(range1,'Visible','on');                 % Visibility property is set to ='off'
                
           elseif distance2<=100
                  line2=plot([xcord2,arr1(j)],[ycord2,arr2(j)],'--','color','green');
                  range1=plot([arr1(j),arr1(j+1)],[arr2(j),arr2(j+1)],'color','green');          % plot the points for given line space. Hence moving vehicle effect
                  pause(0.3);                     
                  set(line2,'Visible','off');                 % Visibility property is set to ='off'
                  set(range1,'Visible','on');                 % Visibility property is set to ='off'
                
           else
                      first=plot([arr1(j),arr1(j+1)],[arr2(j),arr2(j+1)],'color','red');
                      set(first,'Visible','on');
           end
           
         
   end
end

hold off;
