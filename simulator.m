%%%%%% Purpose: Robot simulator 
%%%%%% Author : Arthur Klezovichs

clf;        %clears figures
clc;        %clears console
clear;      %clears workspace
t = cputime;
damping_factor=0;
sn=4.9; % sensor noise
rot_factor = pi; % random rotation angle 
turning_noise=0.01;
motion_noise=0.3; % around optimal
scatter = 1; % number of bad scans before rescattering the particles
low_weight= 1.7; % summ of the weight of an unsuccessful scan
num_scans=20;
num_par=100; % number of particles
num_pos=20; % number of initial positions
par_per_pos= num_par/num_pos; % number of particles per position
map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];
num_steps=10;
sq_size = 5; % square size we want to converge to
test_sample = 1;  
%hold on;
%axis equal;
for test = 1: test_sample
%%%%%%%%%%%%%%%% Initialization %%%%%%%%%%%%%%%%%%%%%%
robot=BotSim(map);
cm=BotSim(map); % center of mass of particles
robot.randomPose(5);
robot.setScanConfig(robot.generateScanConfig(num_scans));
robot.sensorNoise = sn;
robot.motionNoise = motion_noise;
robot.turningNoise = turning_noise;
for i = 1:num_pos
    particles(i)=BotSim(map);
    particles(i).randomPose(5); 
    %particles(i).drawBot(3);
    particles(i).sensorNoise = sn;
    particles(i).motionNoise = motion_noise;
    particles(i).turningNoise = turning_noise;
    particles(i).setScanConfig(particles(i).generateScanConfig(num_scans));
end

j=num_pos+1;
for i = 1:num_pos

part_pos=particles(i).getBotPos();
part_orient=particles(i).getBotAng();
        for k = 1:par_per_pos - 1
        particles(j)=BotSim(map);
        particles(j).setBotPos(part_pos);
        particles(j).setBotAng(part_orient + j*2*pi/5);
       % particles(j).drawBot(3);
        j=j+1;
        end
end



%%%%%%%%%%%%%%%%% Action  %%%%%%%%%%%%%%%%%%
bad_scans=0; % number of bad scans in a row
for step=1:num_steps+1

% Checking if we need to rescatter 
if(bad_scans == scatter)
for i = 1:num_par
particles(i).randomPose(2);
particles(i).setBotAng(2*pi*rand);
end
bad_scans=0;
%display('Rescatered');
%clf;
%particles(1).drawMap();
%axis equal
%hold on
%for i=1:num_par
%particles(i).drawBot(3)
%end
%robot.drawRBot(5)
%input('Press any key to continue...  ');
end


%Do an initial scan
[distance crossingPoint]  = robot.ultraScan();
for i = 1:num_par
[scan_results{i}, cross]=particles(i).ultraScan();
end

%%%%%%%%%%%%%%%%% Weighting %%%%%%%%%%%%%%%%%%%%%%%

% Computing the weight of the particle
for i=1:num_par
  insideMap = particles(i).insideMap();
  current_vector=scan_results{i};
  best_vector=current_vector;
  err_v= distance - current_vector;
  best_error=err_v'*err_v;
  best_orient=particles(i).getBotAng();
  opt_rot=0;
  for j=1:num_scans
    current_vector=circshift(current_vector,-1);
    err_v=distance-current_vector;
    if(err_v'*err_v < best_error)
       best_vector=current_vector;
       best_error=err_v'*err_v;
       opt_rot=j;
    end
  end
  scan_results{i}=best_vector;
  particles(i).turn(opt_rot*2*pi/num_scans);
  
  if(insideMap == 1)
  % Sophisticated weighting function goes here
  x=best_vector;
  y=distance;
  v(1:num_scans)=sn^2;
  sigma=diag(v);
  weight(i)=damping_factor+(1/sqrt(det(2*pi*sigma)))*exp((-1/2)*(x-y)'*inv(sigma)*(x-y));
  %  weight(i)=1*sqrt(2*pi*sn^2)*exp(-(current_vector(1)-distance(1))^2/(2*sn^2)) + damping_factor;
  else 
    weight(i)=0;
  end

end
% Computing the summ of the weights
weight_sum=sum(weight);
%for i=1:num_par
%weight_sum= weight_sum+ weight(i);
%end

% Checking if this scan is "bad" 
if(weight_sum < low_weight)
bad_scans=bad_scans+1;
else
bad_scans=0;
end

% Resampling
% Ruley-wheel selection to choose 50 new particles
for i=1:num_par
rand_part=weight_sum*rand; % Choose a random value from 1 to 0
j=1;
while rand_part >= weight(j) 
rand_part=rand_part-weight(j);
j=j+1;
end
chosen_particle(i)=j;
end

%Creating the new particles and deleting the old ones.
for i=1:num_par
pos=particles(chosen_particle(i)).getBotPos();
ang=particles(chosen_particle(i)).getBotAng();
particles(i).setBotPos(pos);
particles(i).setBotAng(ang);
end


%%%%%%%%%%%%%%%%%% Movement %%%%%%%%%%%%%%%%%%%%%%%%%
%Prevents the robot from bumping into walls
distance_s=distance(1);
%if(distance < 10)
if(distance_s < 10)
%If too close to a wall - move back, but not to far
%so we don't bump into another wall
rotation = pi;
[distance crossingPoint]  = robot.ultraScan();
distance_s=distance(1);
movement=(2*distance_s)*rand;
robot.turn(rotation);
robot.move(movement);
for i = 1:num_par
    particles(i).turn(rotation);
    particles(i).move(movement); 
   % particles(i).drawBot(3);
end
else 
rotation=rot_factor*rand;
%movement=(distance_s/6)*rand;
movement=9*rand;
robot.move(movement);
robot.turn(rotation);
for i = 1:num_par
    particles(i).move(movement); 
    particles(i).turn(rotation);
    %particles(i).drawBot(3);
end
end

%%%%%%%%%%%%%%% Convergence testing %%%%%%%%%%%%%%%%%%%%%%%%%
% Computing the position of the center of mass
sum_orient=0;
sum_pos=[0,0];
for i=1:num_par
sum_orient=sum_orient + particles(i).getBotAng();
sum_pos=sum_pos + particles(i).getBotPos();
end
final_pos =sum_pos/num_par;
final_orient=sum_orient/num_par ;
cm.setBotPos(final_pos);
cm.setBotAng(final_orient);

% Computing convergence region
middle=cm.getBotPos();
xv = [middle(1) + sq_size, middle(1) + sq_size, middle(1) - sq_size, middle(1) - sq_size,middle(1) + sq_size]; 
yv = [middle(2) + sq_size  , middle(2) - sq_size, middle(2) - sq_size, middle(2) + sq_size,middle(2) + sq_size];
for i=1:num_par
pos=particles(i).getBotPos();
x(i)=pos(1);
y(i)=pos(2);
end
in = inpolygon(x,y,xv,yv);



%%%%%%%%%%%%%%%%%% Test display %%%%%%%%%%%%%%%%%%%%%%%%%
clf;
particles(1).drawMap();
axis equal
hold on
cm.drawGBot(3);
robot.drawRBot(5);
plot(xv,yv);
for i=1:num_par
particles(i).drawBot(3);
end
pause(2);
fName = 'weights.txt';         %# A file name
fid = fopen(fName,'a');            %# Open the file
if fid ~= -1
  fprintf(fid,'Weights@Step %d: min/max/mean \n',step,mean(weight),max(weight),min(weight));       %# Print the string
  fclose(fid);                     %# Close the file
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if((nnz(in) > 0.9*num_par)) %&& (step == num_steps))


real_pos=robot.getBotPos();
real_orient=robot.getBotAng();

while(final_orient > 2*pi)
final_orient = final_orient - 2*pi;
end
while(real_orient > 2*pi)
real_orient = real_orient - 2*pi;
end



%if(nnz(in) > 0.99*num_par)
break;
%else
%num_scans=1;
%end
else 
bad_scans = scatter;
step = 1;
end
end
%%%%%%%%%%%%%%%%%%%% Final result %%%%%%%%%%%%%%%%%%%%%%%%%%%

% Making the robot and particles face north.

for i=1:num_par
particles(i).turn(pi/2 - final_orient);
end
robot.turn(pi/2-real_orient);



%Drawing the final configuration and outputing the results
clf;
particles(1).drawMap();
axis equal
hold on

for i=1:num_par
particles(i).drawBot(3);
end

cm.drawGBot(3);
robot.drawRBot(5);

plot(xv,yv,x(~in),y(~in),'bo');
e = cputime - t;
fprintf('Real pos %.1f %.1f orient %.1f \n',real_pos(1),real_pos(2),real_orient);
fprintf('Final pos %.1f %.1f orient %.1f \n',final_pos(1),final_pos(2),final_orient);
fprintf('Distance %.1f  oriernt %.1f \n',sqrt((real_pos(1)-final_pos(1))^2+(real_pos(2)-final_pos(2))^2),real_orient-final_orient);
fprintf('Elapsed %.2f(s)/%d steps/%d scans \n',e,step,num_scans*step);
end