%%%%%% Purpose: Robot simulator 
%%%%%% Author : Arthur Klezovichs

clf;        %clears figures
clc;        %clears console
clear;      %clears workspace

num_par=500;
num_pos=100;
par_per_pos=5;
map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];

hold on;
axis equal;

%%%%%%%%%%%%%%%% Initialization %%%%%%%%%%%%%%%%%%%%%%
 
for i = 1:num_pos
    particles(i)=BotSim(map);
    particles(i).randomPose(5); 
    particles(i).drawBot(3);
end

j=num_pos+1;
for i = 1:num_pos

part_pos=particles(i).getBotPos();
part_orient=particles(i).getBotAng();
        for k = 1:par_per_pos - 1
        particles(j)=BotSim(map);
        particles(j).setBotPos(part_pos);
        particles(j).setBotAng(part_orient + j*2*pi/5);
        particles(j).drawBot(3);
        j=j+1;
        end
end
particles(1).drawMap(); 

%%%%%%%%%%%%%%%%% Movement %%%%%%%%%%%%%%%%%%
age=input('I love drugs ')
%hold off
clf;
axis equal
particles(1).drawMap();
hold on
for i = 1:num_par
    particles(i).move(10);
    particles(i).drawBot(3);
end
age=input('I love drugs ')

%hold off
clf;
particles(1).drawMap();
axis equal
hold on
for i = 1:num_par
    particles(i).move(-10);
    particles(i).drawBot(3);
end
