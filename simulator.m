%%%%%% Purpose: Robot simulator 
%%%%%% Author : Arthur Klezovichs


clf;        %clears figures
clc;        %clears console
clear;      %clears workspace
t = cputime;
t0 = cputime;
%fName = 'weights.txt'; 
%fName1 = 'results.txt';        %# A file name
%fid = fopen(fName,'w');  
%fid1 = fopen(fName1,'w');          %# Open the file
damping_factor=0;
sn=4.9; % sensor noise
rot_factor = pi; % random rotation angle 
turning_noise=0.01;
motion_noise=0.1; % around optimal
scatter = 3; % number of bad scans before rescattering the particles
low_weight= 1.2e-11; % summ of the weight of an unsuccessful scan

num_par=100; % number of particles
num_pos=100; % number of initial positions
par_per_pos= num_par/num_pos; % number of particles per position
map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];
num_steps=10;
sq_size = 5; % square size we want to converge to
test_sample = 10;  
dist_acc = 0; % Used to compute the average distance, when averaging over many runs
ok_c =0; % OK count
cerr_c =0; % Critical error count
in_box=0;
num_scans=15;
conv_time=10; % For how many iteratiodn do we have a particle cluster
%hold on;
%axis equal;
for test = 1: test_sample
in_box=0;
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
  for i = 1:num_pos
    particles(i).randomPose(1); 
  end

  j=num_pos+1;
  for i = 1:num_pos
    part_pos=particles(i).getBotPos(); 
    part_orient=particles(i).getBotAng();
  for k = 1:par_per_pos - 1
    particles(j).setBotPos(part_pos);
    particles(j).setBotAng(part_orient + j*2*pi/5);
    j=j+1;
  end
  end
  bad_scans=0;

%%%%% Draws the scatered particles
clf;
particles(1).drawMap();
axis equal
hold on   
cm.drawGBot(3);
robot.drawRBot(5); 
for i=1:num_par
particles(i).drawBot(3);
end

end


%Do an initial scan
[distance crossingPoint]  = robot.ultraScan();
for i = 1:num_par
if(particles(i).insideMap == 0)
particles(i).randomPose(3);
end
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
  weight(i)=damping_factor+exp((-1/2)*(x-y)'*inv(sigma)*(x-y));
  % Real one below
  % weight(i)=damping_factor+(1/sqrt(det(2*pi*sigma)))*exp((-1/2)*(x-y)'*inv(sigma)*(x-y));
  %  weight(i)=1*sqrt(2*pi*sn^2)*exp(-(current_vector(1)-distance(1))^2/(2*sn^2)) + damping_factor;
  else 
    weight(i)=0;
    particles(i).randomPose(3);
    
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
if(distance_s < 15)
  rotation = pi/4 + pi*rand/4;
  movement=(-1*distance_s/3);
end

if(distance_s > 15 )
  rotation=rot_factor*rand;
if( rem(step,2) == 0 )
  movement=5*rand + 7;
else
movement=10*rand;
end

if (rem(step,4) == 0 )
  movement=distance_s-10;
end
  
end

robot.move(movement);
robot.turn(rotation);
for i = 1:num_par
    particles(i).move(movement); 
    particles(i).turn(rotation);
    %particles(i).drawBot(3);
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

% RANDOM particles around center of mass %
% Adding random particles %
%for rp= 1:(num_par/20)
%dp=final_pos + [2*rand,2*rand];
%rind=randi(num_par);
%particles(rind).setBotPos(dp);
%particles(rind).setBotAng()
%end
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
%if fid ~= -1
  %fprintf(fid,'Weights@Step %d:mean|max|summ %.1e|%.1e|%.1e \n',step,mean(weight),max(weight),sum(weight));
%end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if((nnz(in) > 0.9*num_par)) %&& (rem(step,3) == 0))

in_box=in_box+1;
real_pos=robot.getBotPos();
real_orient=robot.getBotAng();

while(final_orient > 2*pi)
final_orient = final_orient - 2*pi;
end
while(real_orient > 2*pi)
real_orient = real_orient - 2*pi;
end



%if(nnz(in) > 0.99*num_par)
if(in_box == conv_time)
break;
end
%else
%num_scans=1;
%end
%else 
%bad_scans = scatter;
%step = 1;
end
end
%%%%%%%%%%%%%%%%%%%% Final result %%%%%%%%%%%%%%%%%%%%%%%%%%%

% Making the robot and particles face north.

for i=1:num_par
particles(i).turn(pi/2 - final_orient);
end
robot.turn(pi/2-real_orient);
cm.turn(pi/2-final_orient);


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
dist=sqrt((real_pos(1)-final_pos(1))^2+(real_pos(2)-final_pos(2))^2);
dist_acc=dist_acc + dist;

% Distance estimation error check
if( dist < 5 )
 fprintf('[OK]');
 ok_c = ok_c+1;
 end

%if fid1 ~= -1
  fprintf('Dist:%.1f DAng:%.1f %.2f(s)',dist,real_orient-final_orient,e);
  fprintf('Time:%.2f(s) /%d steps\n',e,step);
%end
%fprintf('Test case %d\n',test);
t = cputime;
end

%fprintf(fid1,'ADist:%.1f %.2f(s)\n',dist_acc/test_sample,t-t0);
%fprintf('OKS%d',ok_c);
%fclose(fid);
%fclose(fid1);
fp=cm.getBotPos();
fo=cm.getBotAng();