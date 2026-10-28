rng(3)
tic
actualmap = map;
occGrid=copy(map);

inflate(occGrid,0.5)
figure;
show(actualmap)
%% Drop Location pick up location
X=[2 -4 -5.5 4.5 -0.50];
Y=[3 3 -6.5 -6.5 -8];
W = length(X);
%% Pick up location of Shaft A , Shaft B and Screw 
xp = [-9, 0, 7];
yp = [9, 9, 9];

%% Initial Robot Location
rotbot_initial_pos=[-9, 0  ]; % rotot intial pos[x,y];

%% ploting the coordinates on the occupancy grid
hold on
r = 0.5;
h1 = plot(rotbot_initial_pos(1), rotbot_initial_pos(2), 'go', 'DisplayName', 'Robot Initial Position');
plot([rotbot_initial_pos(1), rotbot_initial_pos(1) + r*cos(pi/2)], [rotbot_initial_pos(2), rotbot_initial_pos(2) + r*sin(pi/2)], 'g-');
h2 = []; 

for p = 1:length(X)
    h2(p) = plot(X(p), Y(p), 'o', 'DisplayName', ['Assembley ', num2str(p)]);
    plot([X(p), X(p) + r*cos(pi/2)], [Y(p), Y(p) + r*sin(pi/2)], '-');
end

for p = 1:length(xp)
    randomColor = rand(1, 3);  % Generate a random RGB color
    h3(p) = plot(xp(p), yp(p), 'o', 'MarkerFaceColor', randomColor, 'DisplayName', ['Pickup ', num2str(p)]);
end


legend([h1, h2, h3])
hold off



%% distance matrix, distance between each nodes
dis_matx = zeros(W,W);
for m = 1:W
    for n = 1:W
        start = [X(m), Y(m), pi/2];
        goal = [X(n), Y(n), pi/2];
        
        %Start and goal position in griid coordinates
        grid_start=world2grid(occGrid,start(1:2));
        grid_goal=world2grid(occGrid,goal(1:2));
        
        % planning the path  - consider only location, not otientation     
        planner = plannerAStarGrid(occGrid);
        route = plan(planner,grid_start,grid_goal);

        %finding distance of the route
        total_distance=0;
        if length(route)>2
            for h = 1:length(route)-1
                P = [route(h,2), route(h)];
                Q = [route(h+1,2), route(h+1)];
                distance = norm(P - Q);
                total_distance= total_distance + distance;
            end
        end
                
        dis_matx(m,n)=total_distance;

    end
end

%% Calculate distance from Robot intial position to all other delivery points (Assembley stations)

Rob_dis_mat = zeros(1,W);

for n = 1:W
        start = [rotbot_initial_pos(1), rotbot_initial_pos(2), pi/2];
        goal = [X(n), Y(n), pi/2];
        
        %Start and goal position in griid coordinates
        grid_start=world2grid(occGrid,start(1:2));
        grid_goal=world2grid(occGrid,goal(1:2));
        
        % planning the path  - consider only location, not otientation     
        planner = plannerAStarGrid(occGrid);
        route = plan(planner,grid_start,grid_goal);

        %finding distance of the route
        total_distance=0;
        if length(route)>2
            for h = 1:length(route)-1
                P = [route(h,2), route(h)];
                Q = [route(h+1,2), route(h+1)];
                distance = norm(P - Q);
                total_distance= total_distance + distance;
            end
        end
        Rob_dis_mat(n)=total_distance;
end

%% Calculate distance from Robot intial position to all other pickup points
Rob_pick_matrix = zeros(1,length(xp));

for n = 1:length(xp)
        start = [rotbot_initial_pos(1), rotbot_initial_pos(2), pi/2];
        goal = [xp(n), yp(n), pi/2];
        
        %Start and goal position in griid coordinates
        grid_start=world2grid(occGrid,start(1:2));
        grid_goal=world2grid(occGrid,goal(1:2));
        
        % planning the path  - consider only location, not otientation     
        planner = plannerAStarGrid(occGrid);
        route = plan(planner,grid_start,grid_goal);

        %finding distance of the route
        total_distance=0;
        if length(route)>2
            for h = 1:length(route)-1
                P = [route(h,2), route(h)];
                Q = [route(h+1,2), route(h+1)];
                distance = norm(P - Q);
                total_distance= total_distance + distance;
            end
        end
        Rob_pick_matrix(n)=total_distance;
end



%% Calculate distance from all pickup points to all delivery points (assembley points)
Del_matrix = zeros(length(xp),n);

for m = 1:length(xp)
    for n = 1:W
        start = [xp(m), yp(m), pi/2];
        goal = [X(n), Y(n), pi/2];
        
        %Start and goal position in griid coordinates
        grid_start=world2grid(occGrid,start(1:2));
        grid_goal=world2grid(occGrid,goal(1:2));
        
        % planning the path  - consider only location, not otientation     
        planner = plannerAStarGrid(occGrid);
        route = plan(planner,grid_start,grid_goal);

        %finding distance of the route
        total_distance=0;
        if length(route)>2
            for h = 1:length(route)-1
                P = [route(h,2), route(h)];
                Q = [route(h+1,2), route(h+1)];
                distance = norm(P - Q);
                total_distance= total_distance + distance;
            end
        end
                
        Del_matrix(m,n)=total_distance;


    end
end

%% Calculate distance between all pick up points
pick_matrix = zeros(length(xp),length(xp)); 

for m = 1:length(xp)
    for n = 1:length(xp)
        start = [xp(m), yp(m), pi/2];
        goal = [xp(n), yp(n), pi/2];
        
        %Start and goal position in griid coordinates
        grid_start=world2grid(occGrid,start(1:2));
        grid_goal=world2grid(occGrid,goal(1:2));
        
        % planning the path  - consider only location, not otientation     
        planner = plannerAStarGrid(occGrid);
        route = plan(planner,grid_start,grid_goal);

        %finding distance of the route
        total_distance=0;
        if length(route)>2
            for h = 1:length(route)-1
                P = [route(h,2), route(h)];
                Q = [route(h+1,2), route(h+1)];
                distance = norm(P - Q);
                total_distance= total_distance + distance;
            end
        end
                
        pick_matrix(m,n)=total_distance;

    end
end


%% Materials needed by each assembley section
box_matrix=[10 18 20 50 50 % number of Shaft A needed by assembley stations
            35 3 1 5 40   % number of Shaft B needed by assembley stations
            15 48 7 8 25];  % numbre of screws needed "...................." 

% tsa = sum(box_matrix(1,:)) %125
% tsb = sum(box_matrix(2,:)) % 84
% ts = sum(box_matrix(3,:)) % 87

%% cost for transpoting each material for a unit distance
trans_cost=[5 4 2]; % cost for transpotation for a unit distance (shaft A, Shaft B, Screws)
ntravalcost=1;

% ucsa = 5 % cost for transpoting Shaft A for a unit distance
% ucsb = 4 % cost for transpoting shaft B for a unit distance
% ucs  = 2 % cost for transpoting screws for a unit distance











%% Parameters for Differential evolution
n=length(X)+length(xp) ; % number of locations to visit
popsize=100; % number of particles
F = 0.8; % Scaling factor
CR = 0.9; % Crossover rate
% n_el=popsize*r_el; % Number of solutions/chromosomes copied to the next generation as Elites
G=0; % initialize the generation counter (G)
Max_G=50; % set the maximum generation (Max_G) as the stopping condition/criteria 
% best_value=zeros(1,Max_G);
daNo_of_best_population=zeros(1,Max_G+1);
dabest_value=zeros(1,Max_G+1);
% No_of_best_population=[];

TP=zeros(popsize,n+1); % Initilaize the total population , popsize*n+1, where column n+1 will be updated as the fitness function later
for j=1:popsize 
    TP(j,1:n)= rand(1,n) ; % each row of TP is a random-key number between (0,1) 
end


%% Encoding , Decoding and Fitness Function
for j = 1:popsize
    [~,pick_seq]=sort(TP(j,(1:length(xp))));
    [~,drop_sec]=sort(TP(j,(4:n)));
    
   
    % Calculating cost from robot inital location to one of the pick  up
    % point
    tcost = Rob_pick_matrix(pick_seq(1)) * ntravalcost;
    totweight=zeros(1,length(xp));
    totweight(pick_seq(1)) = sum(box_matrix(pick_seq(1),:));
    % calculating cost of travell to other pickup points considering the
    % weight
    for i = 1: length(xp)-1
        tcost =  tcost + pick_matrix(pick_seq(i),pick_seq(i+1)) * sum(totweight.*trans_cost);
        totweight(pick_seq(i+1)) = sum(box_matrix(pick_seq(i+1),:));
    end
    tcost = tcost + Del_matrix(pick_seq(length(xp)), drop_sec(1)) * sum(totweight.*trans_cost);
    totweight = totweight - box_matrix(:,drop_sec(1))';
    
    for i = 1:W-1
        tcost= tcost + dis_matx(drop_sec(i),drop_sec(i+1))* sum(totweight.*trans_cost);
        totweight = totweight - box_matrix(:,drop_sec(i+1))';
    end
    totweight;
    tcost;
    tcost = tcost + Rob_dis_mat(drop_sec(W)) * ntravalcost;
    TP(j,n+1) = tcost; % setting the last coloumn of the population to be the total cost
end


dummy=sortrows(TP,n+1);
best_solution=dummy(1,:);
dabest_value(1,1) = min(TP(:, end));
daNo_of_best_population(1,1) = sum(TP(:, end) == min(TP(:, end)));

%% main part of Evolutionary computation
while G< Max_G
      G=G+1;
      Trailvec= zeros(popsize,n+1);
      n_TP= zeros(popsize, n+1);
      % main part of the algorithm
      for i = 1:popsize
          var = randperm(popsize, 3);
          while any(var == i)
              var = randperm(popsize, 3);
          end
         
          % Mutation
          V = best_solution(1,1:n) + F * TP(var(2),1:n) - TP(var(3),1:n);

          % Cross over
          U = zeros(1,n+1);
          for m = 1:n
              r = rand();  dt= randi(n);
              if r <= CR || dt==m
                 % If the crossover probability is met, swap the bits to that
                 % of donnar vector 
                 U(m) = V(m);
              elseif r > CR && dt ~= m
                  U(m) = TP(i,m);
              end              
              
          end

          % Decoding and fitness calculation
          [~,pick_seq]=sort(U(1,(1:length(xp))));
          [~,drop_sec]=sort(U(1,(4:n)));
    
          
          % Calculating cost from robot inital location to one of the pick  up
          % point
          tcost = Rob_pick_matrix(pick_seq(1)) * ntravalcost;
          totweight=zeros(1,length(xp));
          totweight(pick_seq(1)) = sum(box_matrix(pick_seq(1),:));
          % calculating cost of travell to other pickup points considering the
          % weight
          for j = 1: length(xp)-1
              tcost =  tcost + pick_matrix(pick_seq(j),pick_seq(j+1)) * sum(totweight.*trans_cost);
              totweight(pick_seq(j+1)) = sum(box_matrix(pick_seq(j+1),:));
          end
          tcost = tcost + Del_matrix(pick_seq(length(xp)), drop_sec(1)) * sum(totweight.*trans_cost);
          totweight = totweight - box_matrix(:,drop_sec(1))';
    
          for j = 1:W-1
              tcost= tcost + dis_matx(drop_sec(j),drop_sec(j+1))* sum(totweight.*trans_cost);
              totweight = totweight - box_matrix(:,drop_sec(j+1))';
          end
          totweight;
          tcost;
          tcost = tcost + Rob_dis_mat(drop_sec(W)) * ntravalcost;
          U(1,n+1) = tcost; % setting the last coloumn of the population to be the total cost
     
          Trailvec(i,:) = U ;           

      end
      % Fitness evaluation
      for i=1:popsize
          if  Trailvec(i,n+1) <= TP(i,n+1)
              n_TP(i,:) = Trailvec(i,:);
          else
              n_TP(i,:) = TP(i,:);
          end
      end
      dabest_value(1,G+1) = min(n_TP(:, end));
      daNo_of_best_population(1,G+1) = sum(n_TP(:, end) == min(n_TP(:, end)));
      TP = n_TP;
      dummy=sortrows(n_TP,n+1);
      best_solution=dummy(1,:);
end

finalTP=sortrows(n_TP,n+1);%report the finalTP sorted by column n+1
finalTP(1,end)
%% Final, Best path acheived (Global Best particle)
[~,pick_seq]=sort(finalTP(1,(1:length(xp))))
[~,drop_sec]=sort(finalTP(1,(4:n)))
Final_path=[];


%% Final achieved Optimal Route 
% Route Robot initial locaiton to one of the pickup location
start = [rotbot_initial_pos(1), rotbot_initial_pos(2), pi/2];
goal = [xp(pick_seq(1)), yp(pick_seq(1)), pi/2];
grid_start=world2grid(occGrid,start(1:2));
grid_goal=world2grid(occGrid,goal(1:2));
planner = plannerAStarGrid(occGrid);
route = plan(planner,grid_start,grid_goal);

Final_path=[Final_path;route];
% figure;
% show(planner)


% Route for every pickup location
for g= 1:length(xp)-1
    start = [xp(pick_seq(g)), yp(pick_seq(g)), pi/2];
    goal = [xp(pick_seq(g+1)), yp(pick_seq(g+1)), pi/2];
    grid_start=world2grid(occGrid,start(1:2));
    grid_goal=world2grid(occGrid,goal(1:2));

    % planning the path  - consider only location, not otientation
    planner = plannerAStarGrid(occGrid);
    route = plan(planner,grid_start,grid_goal);
    Final_path = [Final_path; route];
%     figure
%     show(planner)    
end
% Route for final pickup location to one of the assembley section
start = [xp(pick_seq(length(xp))), yp(pick_seq(length(xp))), pi/2];
goal = [X(drop_sec(1)), Y(drop_sec(1)), pi/2];
grid_start=world2grid(occGrid,start(1:2));
grid_goal=world2grid(occGrid,goal(1:2));
planner = plannerAStarGrid(occGrid);
route = plan(planner,grid_start,grid_goal);
Final_path=[Final_path;route];
% figure;
% show(planner)

% Route to deliver for all assembley stations
for g= 1:W-1
    start = [X(drop_sec(g)), Y(drop_sec(g)), pi/2];
    goal = [X(drop_sec(g+1)), Y(drop_sec(g+1)), pi/2];
    grid_start=world2grid(occGrid,start(1:2));
    grid_goal=world2grid(occGrid,goal(1:2));

    % planning the path  - consider only location, not otientation
    planner = plannerAStarGrid(occGrid);
    route = plan(planner,grid_start,grid_goal);
    Final_path = [Final_path; route];
%     figure
%     show(planner)    
end
 
% Route after last assembley station to robot initial location

start = [X(drop_sec(W)), Y(drop_sec(W)), pi/2];
goal = [rotbot_initial_pos(1), rotbot_initial_pos(2), pi/2];
grid_start=world2grid(occGrid,start(1:2));
grid_goal=world2grid(occGrid,goal(1:2));

% planning the path  - consider only location, not otientation
planner = plannerAStarGrid(occGrid);
route = plan(planner,grid_start,grid_goal);
Final_path = [Final_path; route];

%Final_path=[Final_path(1:end,2)* (0.1) - 20.1, Final_path(1:end,1) * (-0.1) + 19.3]; % converting the values to coordinates in the senario map
Final_path = grid2world(actualmap,Final_path);
lastx=Final_path(end,1)
lasty=Final_path(end,2)
%% Ploting Final Route

figure
show(actualmap)
hold on
r = 0.5;
h1 = plot(rotbot_initial_pos(1), rotbot_initial_pos(2), 'go', 'DisplayName', 'Robot Initial Position');
plot([rotbot_initial_pos(1), rotbot_initial_pos(1) + r*cos(pi/2)], [rotbot_initial_pos(2), rotbot_initial_pos(2) + r*sin(pi/2)], 'g-');
h2 = []; 

for p = 1:length(X)
    h2(p) = plot(X(p), Y(p), 'o', 'DisplayName', ['Assembley ', num2str(p)]);
    plot([X(p), X(p) + r*cos(pi/2)], [Y(p), Y(p) + r*sin(pi/2)], '-');
end

for p = 1:length(xp)
    randomColor = rand(1, 3);  % Generate a random RGB color
    h3(p) = plot(xp(p), yp(p), 'o', 'MarkerFaceColor', randomColor, 'DisplayName', ['Pickup ', num2str(p)]);
end

for q = 1:length(Final_path(1:end,1))-1
    plot([Final_path(q,1), Final_path(q+1,1)], [Final_path(q,2), Final_path(q+1,2)], '--', 'Color', 'k');
end

legend([h1, h2, h3])
title('Final Path')
hold off

figure;
plot(dabest_value)
hold on
plot(sabest_value)
plot(gabest_value)
title('Best value in each generation / iteration')
xlabel("Generation / iteration")
ylabel("Cost")
legend("Differential Evolution (DE)","Particle Swarm Optimization (PSO)", "Genetic Algorithm (GA)")
figure
plot(daNo_of_best_population)
title('No. of best populaiton in each generation')


     