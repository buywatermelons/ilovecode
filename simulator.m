%%%%%% Purpose: Robot simulator 
%%%%%% Author : Arthur Klezovichs

clf;        %clears figures
clc;        %clears console
clear;      %clears workspace

variance=1; % the variance of the measurement
num_par=50;
num_pos=10;
par_per_pos=5;
map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];

hold on;
axis equal;

%%%%%%%%%%%%%%%% Initialization %%%%%%%%%%%%%%%%%%%%%%
robot=BotSim(map);
robot.randomPose(3);
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
robot.drawBot(10);


%%%%%%%%%%%%%%%%% Movement %%%%%%%%%%%%%%%%%%
age=input('Press any key to continue...  ')
%hold off
clf;
axis equal
particles(1).drawMap();
hold on
for i = 1:num_par
    particles(i).move(10);
    particles(i).drawBot(3);
end
robot.move(10);
robot.drawBot(5);
age=input('Press any key to continue...  ')

%hold off
clf;
particles(1).drawMap();
axis equal
hold on
for i = 1:num_par
    particles(i).move(-10);
    particles(i).drawBot(3);
end
robot.move(-10)
robot.drawBot(5)

%%%%%%%%%%%%%%% Scanning %%%%%%%%%%%%%%
botSim.sensorNoise = variance;
[distance crossingPoint]  = robot.ultraScan();
for i = 1:num_par
scan_results(i)=particles(i).ultraScan();
end

% Computing the weight of the particle
for i=1:num_par
weight(i)=1*sqrt(2*pi*variance^2)*exp(-(scan_results(i)-distance)^2/(2*variance^2));
end

% Computing the summ of the weights
weight_sum=0;
for i=1:num_par
weight_sum= weight_sum+ weight(i);
end

% Normalisation
for i=1:num_par
probability(i)=weight(i)/weight_sum;
end

% Resampling
% Ruley-wheel selection to choose 50 new particles
for i=1:num_par
rand_part=rand; % Choose a random value from 1 to 0
j=1;
while rand_part >= probability(j) 
rand_part=rand_part-probability(j);
j=j+1;
end
chosen_particle(i)=j;
end

for i=1:num_par
pos=particles(chosen_particle(i)).getBotPos();
ang=particles(chosen_particle(i)).getBotAng();
particles(i).setBotPos(pos);
particles(i).setBotAng(ang);
end

clf;
particles(1).drawMap();
axis equal
hold on

for i=1:num_par
particles(i).drawBot(3)
end
robot.drawBot(5)