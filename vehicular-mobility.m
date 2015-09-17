%SIMULATION OF VANET MOBILITY AND V2V SIMULATION

entry=[1500 2500 4000]; %ycoordinates of entry ramps
exit=[1550 2550 4050];  %ycoordinates of exit ramps
trafficdensity=[100 150 200 250 300 350 400 450 500];
v2v=[0 0 0 0 0 0 0 0 0]; %list having the connectivity entries for different traffice densities
target=41; %car numbered 41 is taken as target
neighbours=[];
duration=[0 0 0 0 0 0 0 0 0];
avg_cont_neighbours=[0 0 0 0 0 0 0 0 0];
for cars=100:50:500
    connectivity=[0 0 0 0 0];
    times=[];
    same_neighbours=[];
    ycoord1=transpose(randperm(5000,cars)); %the array has ycoordinate positions of cars on lane 1
    ycoord2=transpose(randperm(5000,cars)); %the array has ycoordinate positions of cars on lane 2
    ycoord3=transpose(randperm(5000,cars)); %the array has ycoordinate positions of cars on lane 3
    ycoord4=transpose(randperm(5000,cars)); %the array has ycoordinate positions of cars on lane 4
    ycoord1=sort(ycoord1,1); %sorting to get cars in increasing order of positions
    ycoord2=sort(ycoord2,1);
    ycoord3=sort(ycoord3,1);
    ycoord4=sort(ycoord4,1);
    positions=zeros(5000,4);
    speed=zeros(5000,4);
    oldcoord=0;
    %placing cars numbered uniquely in increasing in a 5000x4 matrix named
    %positions and their respective speeds in a mtrix called speed (wherever no car is present, positions and speed value is zero)
    for j=1:cars
        index=ycoord1(j);   
        positions(index,1)=j;   %place car at the position pointed by the ycoord1 array
        speed(index,1)=(31-22).*rand(1,1) + 22; %speed values (50 & 70 miles /hour) converted to m/s
    end
    %for cars in lane 2:
    maximum=max(max(positions));    %for car labels to be in continuity
    for j=1:cars
        index=ycoord2(j);
        positions(index,2)=maximum+j;   %for car labels to be in continuity
        speed(index,2)=(31-22).*rand(1,1) + 22; %speed values (50 & 70 miles /hour) converted to m/s
    end
    %for cars in lane 3:
    maximum=max(max(positions));
    for j=1:cars
        index=ycoord3(j);
        positions(index,3)=maximum+j;
        speed(index,3)=(31-22).*rand(1,1) + 22; %speed values (50 & 70 miles /hour) converted to m/s
    end
    maximum=max(max(positions));
    %for cars in lane 4:
    for j=1:cars
        index=ycoord4(j);
        positions(index,4)=maximum+j;
        speed(index,4)=(31-22).*rand(1,1) + 22; %speed values (50 & 70 miles /hour) converted to m/s
    end
    for i=1:5 %five iterations for computing average 5
        timeflag=0;
        neighbours=[];
        for j=1:22 %10 minutes in all=600 secs, value of j is the seconds value
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % For Lane 1:
            %safety application messages will be exchanged here (every 1 sec)
            rand1=rand(1,1);
            rand2=rand(1,1);
            if rand1>0.833
                entryramp=randi(3,[1,1]);   %to chose an entry ramp randomly
                if ~(ismember(entry(entryramp),ycoord1)) %checking if no vehicle is already present there
                    %add a vehicle here
                    ycoord1(length(ycoord1)+1)=entry(entryramp);
                    positions(entry(entryramp),1)=max(max(positions))+1;    %label the new car
                    speed(entry(entryramp),1)=(31-22).*rand(1,1) + 22;  %assign it a random speed
                end
            end
            if rand2>0.833
                exitramp=randi(3,[1,1]);    %to chose an exit ramp randomly
                if ismember(exit(exitramp),ycoord1) %remove if there is car at that exit ramp that can exit
                    if positions(exit(exitramp),1)~=target %target car should not be removed
                        ycoord1=ycoord1(find(ycoord1~=exit(exitramp))); %car location removed from ycoord array
                        positions(exit(exitramp),1)=0;  %car removed
                        speed(exit(exitramp),1)=0;  %corresponding speed entry cleared
                    end
                end
            end
            ycoord1=sort(ycoord1,1); %sort the vehicles in increasing order of positions
            for k=1:1    %1 loop assumed to be of 100ms for required granularity
                for l=1:length(ycoord1) % for each car in the lane
                    if(l<=length(ycoord1))  %to avoid indexing error when no. of cars reduce during lane changing, in short recalculating the length
                        acc=-5+10*rand(1,1);    %random acceleration value within -5 and +5 m/s^2
                        r=speed(ycoord1(l),1);
                        newspeed=abs(r+acc);
                        if(l~=length(ycoord1))
                            if(ycoord1(l+1)-ycoord1(l)<=10)
                                flag=1;
                                %look if lane change is poosible :-
                                m=ycoord1(l);
                                for g=m-10:m+10 %checking if any car is present in the parallel lane near that position
                                    if  ismember(g,ycoord2)
                                        flag=0; %if so,lane change is not possible in lane2
                                    end
                                end
                                if flag==1  %if there is space for changing lanes
                                    %change lane here:
                                    ycoord2(length(ycoord2)+1)=m;
                                    positions(m,2)=positions(m,1);  %move the car to next lane
                                    positions(m,1)=0;   %clear the car label from current lane
                                    speed(m,2)=speed(m,1);  %move the corresponding speed entry to adjacent lane
                                    speed(m,1)=0;   %clear the speed entry from current lane
                                    ycoord1(l)=0;   %remove the position entry from ycoord1 array
                                    ycoord2=sort(ycoord2,1);    %sort the array pointing to next lane to get them back in increasing order after adding car 
                                elseif flag==0 %if lane change was not possible
                                    dist=ycoord1(l+1)-ycoord1(l);   
                                    factor=(-0.75+sqrt(0.5625+0.02804*dist))/0.01408;    
                                    newspeed=min(factor,speed(ycoord1(l+1),1)); %reduce the speed as given in the project desciption
                                end
                            end
                        end
                        if(ycoord1(l)~=0)
                            oldcoord=ycoord1(l);
                            ycoord1(l)=round(ycoord1(l)+newspeed*0.01); %position update per 100ms
                            ycoord1(l)=mod(ycoord1(l),5000)+1;  %rollback position
                            if(positions(ycoord1(l),1)==0)
                                positions(ycoord1(l),1)=positions(oldcoord,1);  %move the car forward
                                positions(oldcoord,1)=0;    %clear previous position
                                speed(ycoord1(l),1)=newspeed;   %similarly with speed matrix
                                speed(oldcoord,1)=0;
                            end
                        end 
                    end
                end
                ycoord1=ycoord1(find(ycoord1~=0));  %remove the cleared entries from ycoord1 array, car is no more present there as it changed lanes
            end
             
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
            %similar computation for each lanes(refer to detailed comments above). Lane 2:
            %entry and exit here
            rand1=rand(1,1);
            rand2=rand(1,1);
            if rand1>0.833
                entryramp=randi(3,[1,1]);
                if ~(ismember(entry(entryramp),ycoord2)) %checking if no vehicle is already present there
                    %add a vehicle here
                    ycoord2(length(ycoord2)+1)=entry(entryramp);
                    positions(entry(entryramp),2)=max(max(positions))+1;
                    speed(entry(entryramp),2)=(31-22).*rand(1,1) + 22;
                end
            end
            if rand2>0.833
                exitramp=randi(3,[1,1]);
                if ismember(exit(exitramp),ycoord2)
                    if positions(exit(exitramp),2)~=target
                        ycoord2=ycoord2(find(ycoord2~=exit(exitramp)));
                        positions(exit(exitramp),2)=0;
                        speed(exit(exitramp),2)=0;
                    end
                end
            end
            ycoord2=sort(ycoord2,1);
            for k=1:1
                for l=1:length(ycoord2) % for each car in a lane
                    if(l<=length(ycoord2))
                        acc=-5+10*rand(1,1);
                        newspeed=abs(speed(ycoord2(l),2)+acc);
                        if(l~=length(ycoord2))
                            if(ycoord2(l+1)-ycoord2(l)<=10)
                                flag=1;
                                %look if lane change is poosible in lane 1:-
                                m=ycoord2(l);
                                for g=m-10:m+10
                                    if  ismember(g,ycoord1)
                                        flag=0; %lane change not possible in lane 1
                                    end
                                end
                                if flag==1
                                    %change lane here to lane 1:
                                    ycoord1(length(ycoord1)+1)=m;
                                    positions(m,1)=positions(m,2);
                                    positions(m,2)=0;
                                    speed(m,1)=speed(m,2);
                                    speed(m,2)=0;
                                    ycoord2(l)=0;
                                    ycoord1=sort(ycoord1,1);
                                elseif flag==0
                                    %look if lane change is poosible in lane 3:-
                                    for g=m-10:m+10
                                        if  ismember(g,ycoord3)
                                            flag=0; %lane change not possible in lane 3
                                        end
                                    end
                                    if flag==1
                                        %change lane here to lane 3:
                                        ycoord3(length(ycoord3)+1)=m;
                                        positions(m,3)=positions(m,2);
                                        positions(m,2)=0;
                                        speed(m,3)=speed(m,2);
                                        speed(m,2)=0;
                                        ycoord2(l)=0;
                                        ycoord3=sort(ycoord3,1);
                                    elseif flag==0 %if lane change was not possible at all
                                        dist=ycoord2(l+1)-ycoord2(l);
                                        factor=(-0.75+sqrt(0.5625+0.02804*dist))/0.01408;
                                        newspeed=min(factor,speed(ycoord2(l+1),2));
                                    end
                                end
                            end
                        end
                        if(ycoord2(l)~=0)
                            oldcoord=ycoord2(l);
                            ycoord2(l)=round(ycoord2(l)+newspeed*0.01); %position update per 100ms
                            ycoord2(l)=mod(ycoord2(l),5000)+1;%rollback position
                            if(positions(ycoord2(l),2)==0)
                                positions(ycoord2(l),2)=positions(oldcoord,2);
                                positions(oldcoord,2)=0;
                                speed(ycoord2(l),2)=newspeed;
                                speed(oldcoord,2)=0;
                            end
                        end
                    end
                end
                ycoord2=ycoord2(find(ycoord2~=0));
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %Lane 3:
            %entry and exit here
            rand1=rand(1,1);
            rand2=rand(1,1);
            if rand1>0.833
                entryramp=randi(3,[1,1]);
                if ~(ismember(entry(entryramp),ycoord3)) %checking if no vehicle is already present there
                    %add a vehicle here
                    ycoord3(length(ycoord3)+1)=entry(entryramp);
                    positions(entry(entryramp),3)=max(max(positions));
                    speed(entry(entryramp),3)=(31-22).*rand(1,1) + 22;
                end
            end
            if rand2>0.833
                exitramp=randi(3,[1,1]);
                if ismember(exit(exitramp),ycoord3)
                    if positions(exit(exitramp),3)~=target
                        ycoord3=ycoord3(find(ycoord3~=exit(exitramp)));
                        positions(exit(exitramp),3)=0;
                        speed(exit(exitramp),3)=0;
                    end
                end
            end
            ycoord3=sort(ycoord3,1);
            for k=1:1
                for l=1:length(ycoord3) % for each car in the lane
                    if(l<=length(ycoord3))
                        acc=-5+10*rand(1,1);
                        newspeed=abs(speed(ycoord3(l),3)+acc);
                        if(l~=length(ycoord3))
                            if(ycoord3(l+1)-ycoord3(l)<=10)
                                flag=1;
                                %look if lane change is poosible in lane 2:-
                                m=ycoord3(l);
                                for g=m-10:m+10
                                    if  ismember(g,ycoord2)
                                        flag=0; %lane change not possible in lane 2
                                    end
                                end
                                if flag==1
                                    %change lane here to lane 2:
                                    ycoord2(length(ycoord2)+1)=m;
                                    positions(m,2)=positions(m,3);
                                    positions(m,3)=0;
                                    speed(m,2)=speed(m,3);
                                    speed(m,3)=0;
                                    ycoord3(l)=0;
                                    ycoord2=sort(ycoord2,1);  
                                elseif flag==0
                                    %look if lane change is poosible in lane 4:-
                                    for g=m-10:m+10
                                        if  ismember(g,ycoord4)
                                            flag=0; %lane change not possible in lane 4
                                        end
                                    end
                                    if flag==1
                                        %change lane here to lane 4:
                                        ycoord4(length(ycoord4)+1)=m;
                                        positions(m,4)=positions(m,3);
                                        positions(m,3)=0;
                                        speed(m,4)=speed(m,3);
                                        speed(m,3)=0;
                                        ycoord3(l)=0;
                                        ycoord4=sort(ycoord4,1);
                                    elseif flag==0 %if lane change was not possible at all
                                        dist=ycoord3(l+1)-ycoord3(l);
                                        factor=(-0.75+sqrt(0.5625+0.02804*dist))/0.01408;
                                        newspeed=min(factor,speed(ycoord3(l+1),3));
                                    end
                                end
                            end
                        end
                        if(ycoord3(l)~=0)
                            oldcoord=ycoord3(l);
                            ycoord3(l)=round(ycoord3(l)+newspeed*0.01); %position update per 100ms
                            ycoord3(l)=mod(ycoord3(l),5000)+1;%rollback position
                            if(positions(ycoord3(l),3)==0)
                                positions(ycoord3(l),3)=positions(oldcoord,3);
                                positions(oldcoord,3)=0;
                                speed(oldcoord,3)=0;
                                speed(ycoord3(l),3)=newspeed;
                            end
                        end
                    end
                end
                ycoord3=ycoord3(find(ycoord3~=0));
            end
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %for lane 4:
            %entry and exit here
            rand1=rand(1,1);
            rand2=rand(1,1);
            if rand1>0.833
                entryramp=randi(3,[1,1]);%3 or 4???
                if ~(ismember(entry(entryramp),ycoord4)) %checking if no vehicle is already present there
                    %add a vehicle here
                    ycoord4(length(ycoord4)+1)=entry(entryramp);
                    positions(entry(entryramp),4)=max(max(positions));
                    speed(entry(entryramp),4)=(31-22).*rand(1,1) + 22;
                end
            end
            if rand2>0.833
                exitramp=randi(3,[1,1]);%3 or 4???
                if ismember(exit(exitramp),ycoord4)
                    if positions(exit(exitramp),4)~=target
                        ycoord4=ycoord4(find(ycoord4~=exit(exitramp)));
                        positions(exit(exitramp),4)=0;
                        speed(exit(exitramp),4)=0;
                    end
                end
            end
            ycoord4=sort(ycoord4,1);
            for k=1:1
                for l=1:length(ycoord4) % for each car in a lane
                    if(l<=length(ycoord4))
                        acc=-5+10*rand(1,1);
                        newspeed=abs(speed(ycoord4(l),4)+acc);
                        if(l~=length(ycoord4))
                            if(ycoord4(l+1)-ycoord4(l)<=10)
                                flag=1;
                                %look if lane change is poosible in lane 3:-
                                m=ycoord4(l);
                                for g=m-10:m+10
                                    if  ismember(g,ycoord3)
                                        flag=0; %lane change not possible
                                    end
                                end
                                if flag==1
                                    %change to lane 3 here:
                                    ycoord3(length(ycoord3)+1)=m;
                                    positions(m,3)=positions(m,4);
                                    positions(m,4)=0;
                                    speed(m,3)=speed(m,4);
                                    speed(m,4)=0;
                                    ycoord4(l)=0;
                                    ycoord3=sort(ycoord3,1);
                                elseif flag==0 %if lane change was not possible
                                    dist=ycoord4(l+1)-ycoord4(l);
                                    factor=(-0.75+sqrt(0.5625+0.02804*dist))/0.01408;
                                    newspeed=min(factor,speed(ycoord4(l+1),4));
                                end
                            end
                        end
                        if(ycoord4(l)~=0)
                            oldcoord=ycoord4(l);
                            ycoord4(l)=round(ycoord4(l)+(newspeed)); %position update per 100ms
                            ycoord4(l)=mod(ycoord4(l),5000)+1;%rollback position
                            if(positions(ycoord4(l),4)==0)
                                positions(ycoord4(l),4)=positions(oldcoord,4);
                                positions(oldcoord,4)=0;
                                speed(oldcoord,4)=0;
                                speed(ycoord4(l),4)=newspeed;
                            end
                        end
                    end
                end
                ycoord4=ycoord4(find(ycoord4~=0));
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %answering the questions:
            new=[];
            [row,col]=find(positions==target);  %store the coordinates of target car by finding it using its label in the positions matrix
            for o=1:4
                for p=1:5000
                    X=[p,500+(o-1)*3;row,500+(col-1)*3];    %factor of 3 multiplied to take into account the 3m lane separation
                    if positions(p,o)~=0
                        if pdist(X,'euclidean')<50  %communication range=50, has to be change to run for a different range
                            if j==1
                                neighbours(length(neighbours)+1)=positions(p,o); %add all communication neighbors to an array for the 1st second
                            end
                            new(length(new)+1)=positions(p,o);  %similar array for every other second which will be overwritten every seconds iteration
                        end
                    end
                end
            end
            if j==1
                common=intersect(neighbours,new);
                common_cont=intersect(neighbours,new);
            else
                common_cont=intersect(common,new);  %this array will store the continuos neighbours every second
                common=intersect(neighbours,new);   %recalculate common neighbours for consecutive seconds iteration
            end
            if timeflag==0
                if length(common)>=3
                    times(i)=j;
                elseif length(common)<3
                    timeflag=1; %set if less than 3 cars are common to avoid further iteration
                end
            end
            if j==20 %continuous duration of time for question 3
                same_neighbours(i)=length(common_cont); %number of continuos neighbors after 10 seconds
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        [row,col]=find(positions==target);
        for o=1:4
            for p=1:5000
                X=[p,500+(o-1)*3;row,500+(col-1)*3]; %factor of 3 multiplied to take into account the 3m lane separation
                if positions(p,o)~=0
                    if pdist(X,'euclidean')<50  %communication range has to be changed here to get outputs for a different range
                        connectivity(i)=connectivity(i)+1; %increment for every neighbor in communication range
                    end
                end
            end
        end
    end
    avg_cont_neighbours(cars/50-1)=mean(same_neighbours);
    duration(cars/50-1)=mean(times);
    v2v(cars/50-1)=mean(connectivity);
end
%Question 1:
figure(1);
plot(trafficdensity,v2v);
title('Connectivity Plot for communication range = 50 meters');
xlabel('Traffic Density');
ylabel('Average number of nodes');
%Question 2:
figure(2);
plot(trafficdensity,duration);
title('Connectivity Duration Plot (3 neighbours) for communication range = 50 meters');
xlabel('Traffic Density');
ylabel('Average duration');
%Question 3
figure(3);
plot(trafficdensity,avg_cont_neighbours);
title('Average number of same neighboursfor a continuous period of 10s, range = 50 meters');
xlabel('Traffic Density');
ylabel('Average no. of same neighbours');
%Outputs for Question 4 taken by running this code for communication
%range=100m
