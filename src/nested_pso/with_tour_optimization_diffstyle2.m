% Create a 10x2 cell array
tic
rng(1)
%% Drop Location pick up location
X=[2 -4 -5.5 4.5 -0.50];
Y=[3 3 -6.5 -6.5 -8];
W = length(X);
%% Pick up location of Shaft A , Shaft B and Screw 
xp = [-9, 0, 7];
yp = [9, 9, 9];

%% Initial Robot Location
rotbot_initial_pos=[-9, 0]; % rotot intial pos[x,y];

Demand_matrix=[10 18 20 50 50 % number of Shaft A needed by assembley stations
            35 3 1 5 40   % number of Shaft B needed by assembley stations
            15 48 7 8 25];
Demand_total = sum(sum(Demand_matrix));


%%  distance matrix, distance between each nodes
dis_matx = [0	60	157.639610306789	108.870057685089	153.124891681028
60	0	109.355339059327	157.740115370178	160.296464556282
157.639610306789	109.355339059327	0	108.627416997969	57.3847763108502
108.870057685089	157.740115370178	108.627416997970	0	56.2132034355964
153.124891681028	160.296464556282	57.3847763108502	56.2132034355964	0];

%% Calculate distance between all pick up points
pick_matrix = [0	112.367532368147	182.367532368147
112.367532368147	0	97.6396103067892
182.367532368147	97.6396103067892	0]; 

%% Calculate distance from all pickup points to all delivery points (assembley points)
Del_matrix = [155.941125496954	95.9411254969543	169.497474683059	253.681240867132	220.438600180013
126.911688245431	138.468037431535	212.024386617640	195.740115370178	240.823376490863
96.5269119345812	156.526911934581	254.166522241371	165.355339059328	210.438600180013];

%% cost for transpoting each material for a unit distance
trans_cost=[5 4 2]; % cost for transpotation for a unit distance (shaft A, Shaft B, Screws)
ntravalcost=1;

%% Calculate distance from Robot intial position to all other pickup points
Rob_pick_matrix = [90	146.610173055266	216.610173055267];

%% Calculate distance from Robot intial position to all other delivery points (Assembley stations)

Rob_dis_mat = [122.426406871193	62.4264068711928	79.4974746830583	168.953318805774	130.438600180013];

%% Maxmimum robot limit
robot_limit = 170; 

%% Maximum tour limit
tour_limit = 5;
u= numel(Demand_matrix)*tour_limit;
%%
%%  testatsatsartsdatsdatsdata 
tparticles=100;

tit=0; % counting itteration
tc1 = 1; % Magnitude
tc2 = 1; % Magnitude

tg_best = zeros(1,u+2); % Initiating an array for global best parent, with the last slot holding the best acheived value. 
tg_best(1,u+1 : u+2)= inf; % initially setting the global best acheived result (distance) to be +ve infinity as we are about minimise the distance
tparticles_best = zeros(tparticles,u+2); % initiating a matrix for particle best, with size W+1, where the last coloumn consists of the total distance or cost
tparticles_best(1:tparticles,u+1:u+2)=inf;
tvelocity = randi([0, max(max(Demand_matrix))], tparticles, u); % initiating a velocity vector with random number
tinitialWeight =0.9 ; % inital weight for updating the vector
tfinalWeight = 0.4 ;  % final weight
tmaxit = 150; % maximum number of itteration to run
tweight = 0.5; % weight used to update the vector
tbest_pick_seq=zeros(1,length(xp)*tour_limit);
tdropsequence=zeros(1,length(X)*tour_limit);
tTP = zeros(tparticles, u+2);
best_value_tour=zeros(1,tmaxit);


% maxval=[];
% for o =1:tour_limit
%     maxval= [ maxval, reshape(Demand_matrix, 1, [])];
% end

for tj=1:tparticles 
%   tTP(tj,1:u)=randi([0, round(mean(Demand_matrix, 'all'))], 1, u ); % each row of TP is a random-key number between (0,1)
    tTP(tj, 1:u) = randi([0, round(mean(mean(Demand_matrix)))], 1, u);

%    tTP(tj, 1:u) = randi([0, max(max(Demand_matrix))], 1, u);
   
end




bestgg=zeros(1,tmaxit);

%% Encoding and Decoding
% outputMatrix = reshape(TP, 2, 3, 2);

for o=1:tparticles
    dup_demand_mat=Demand_matrix;
    bb=reshape(tTP(o,1:u), length(xp), length(X), tour_limit);
    tour=0;
    totcost=0;
    rotbot_initial_pos=rotbot_initial_pos;
    total_parts_to_deliver=Demand_total;
    triplastpos=0;
    for t=1:tour_limit
        box_matrix=bb(:,:,t);
        sum(sum(box_matrix));
        if any(any(box_matrix > Demand_matrix)) || any(any(box_matrix < 0))
           mask = box_matrix > dup_demand_mat;
           box_matrix(mask) = dup_demand_mat(mask);
           box_matrix(box_matrix < 0) = 0;
           tTP(o,numel(Demand_matrix)*t - numel(Demand_matrix) +1:numel(Demand_matrix)*t)= reshape(box_matrix, 1, []);
        end
        box_matrix;
    
        if total_parts_to_deliver > 0
           if sum(sum(box_matrix))> robot_limit
              totcost = inf;
              tour = inf;
           elseif sum(sum(box_matrix)) > 0 && tour~= inf 
                  dup_demand_mat = dup_demand_mat - box_matrix;
                  total_parts_to_deliver=total_parts_to_deliver-sum(sum(box_matrix));
                  tour = tour+1;
                  [tcost,picksequence,tdropsequence]=Seq_dis_function2(box_matrix,X,xp,ntravalcost,trans_cost,dis_matx,pick_matrix,Del_matrix,Rob_pick_matrix,Rob_dis_mat,W,triplastpos);
                  totcost=totcost + tcost;
                  triplastpos=tdropsequence(end);
           end  
            
           if tour == tour_limit && (total_parts_to_deliver ~=0)
               totcost = inf;
               tour = inf;
            end
        end
    end
    total_parts_to_deliver;
    if total_parts_to_deliver ~= 0
         totcost = inf;
         tour = inf;
    end
    sum(sum(dup_demand_mat));
    tour;
    if tour > 1
        tourcost=1000*tour;
    else
        tourcost=0;
    end
    totcost;
    tTP(o,u+1)=tour  ;
    tTP(o,u+2)=totcost+ tourcost + Rob_dis_mat(tdropsequence(1,end));

    if tTP(o,u+1) < tparticles_best(o,u+1)  % if value (distance or cost) acheived is better than the particle best, then the current particle is the particle best 
       tparticles_best(o,1:u+2) = tTP(o,1:u+2);
    end

    if tTP(o,u+1) < tg_best(1,u+1) % if the value (distance or cost) acheived is better than the global best value, then the current particle is the global best particle
        tg_best(1,1:u+2) =  tTP(o,1:u+2);
    end
end



%% Main part of Swarm optimisation
 while tit < tmaxit 
     
       tit = tit + 1
       tweight = tinitialWeight - (tinitialWeight - tfinalWeight) * (tit / tmaxit);
%      round(tweight);

     % update velocity
      for p = 1:tparticles
          tvelocity(p,1:u) = tweight*tvelocity(p,1:u)+tc1*rand(1,u).*(tparticles_best(p,1:u)-tTP(p,1:u))+ tc2*rand(1,u).*(tg_best(1:u)-tTP(p,1:u));
          %tvelocity=round(tvelocity);
          tTP(p,1:u) = tTP(p,1:u) + tvelocity(p,1:u);
      end
      tTP(:,1:u)=round(abs(tTP(:,1:u)));

    for o=1:tparticles
        
        dup_demand_mat=Demand_matrix;
        bb=reshape(tTP(o,1:u), length(xp), length(X), tour_limit);
        tour=0;
        totcost=0;
        rotbot_initial_pos=rotbot_initial_pos;
        total_parts_to_deliver=Demand_total;
        triplastpos=0;
        for t=1:tour_limit
            box_matrix=bb(:,:,t);
            sum(sum(box_matrix));
            if any(any(box_matrix > Demand_matrix)) || any(any(box_matrix < 0))
               mask = box_matrix > dup_demand_mat;
               box_matrix(mask) = dup_demand_mat(mask);
               box_matrix(box_matrix < 0) = 0;
               tTP(o,numel(Demand_matrix)*t - numel(Demand_matrix) +1:numel(Demand_matrix)*t)= reshape(box_matrix, 1, []);
            end
            box_matrix;
        
            if total_parts_to_deliver > 0
               if sum(sum(box_matrix))> robot_limit
                  totcost = inf;
                  tour = inf;
               elseif sum(sum(box_matrix)) > 0 && tour~= inf 
                      dup_demand_mat = dup_demand_mat - box_matrix;
                      total_parts_to_deliver=total_parts_to_deliver-sum(sum(box_matrix));
                      tour = tour+1;
                      [tcost,picksequence,tdropsequence]=Seq_dis_function2(box_matrix,X,xp,ntravalcost,trans_cost,dis_matx,pick_matrix,Del_matrix,Rob_pick_matrix,Rob_dis_mat,W,triplastpos);
                      totcost=totcost + tcost;
                      triplastpos=tdropsequence(end);
               end  
                
               if tour == tour_limit && (total_parts_to_deliver ~=0)
                   totcost = inf;
                   tour = inf;
                end
            end
        end
        total_parts_to_deliver;
        if total_parts_to_deliver ~= 0
             totcost = inf;
             tour = inf;
        end
        sum(sum(dup_demand_mat));
        tour;
        if tour > 1
            tourcost=1000*tour;
        else
            tourcost=0;
        end
        totcost;
        tTP(o,u+1)=tour  ;
        tTP(o,u+2)=totcost+ tourcost + Rob_dis_mat(tdropsequence(1,end));
    
        if tTP(o,u+1) < tparticles_best(o,u+1)  % if value (distance or cost) acheived is better than the particle best, then the current particle is the particle best 
           tparticles_best(o,1:u+2) = tTP(o,1:u+2);
        end
    
        if tTP(o,u+1) < tg_best(1,u+1) % if the value (distance or cost) acheived is better than the global best value, then the current particle is the global best particle
            tg_best(1,1:u+2) =  tTP(o,1:u+2);
        end
    end
    best_value_tour(tit) =  tg_best(1,end-1);
end
tg_best(end)
tg_best(end-1)
toc
reshape(tg_best(1,1:u), length(xp), length(X), tour_limit)


figure
plot(best_value_tour)




% 




















