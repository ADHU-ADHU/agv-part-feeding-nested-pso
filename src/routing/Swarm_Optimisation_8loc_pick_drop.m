rng(1)
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
rotbot_initial_pos=[-9, 0]; % rotot intial pos[x,y];

%% ploting the coordinates on the occupancy grid
hold on
r = 0.5;
h1 = plot(rotbot_initial_pos(1), rotbot_initial_pos(2), 'go', 'DisplayName', 'Robot Initial Position');
plot([rotbot_initial_pos(1), rotbot_initial_pos(1) + r*cos(pi/2)], [rotbot_initial_pos(2), rotbot_initial_pos(2) + r*sin(pi/2)], 'g-');
h2 = []; 

for p = 1:length(X)
    h2(p) = plot(X(p), Y(p), 'o', 'DisplayName', ['Assembly ', num2str(p)]);
    plot([X(p), X(p) + r*cos(pi/2)], [Y(p), Y(p) + r*sin(pi/2)], '-');
end

colors = rand(3, 3);

% Loop through the points and cycle through the three colors
for p = 1:length(xp)
    % Use one of the three colors in a cyclic manner
    currentColor = colors(mod(p-1, size(colors, 1)) + 1, :);

    h3(p) = plot(xp(p), yp(p), 'o', 'MarkerFaceColor', currentColor, 'DisplayName', ['Pickup ', num2str(p)]);
end


legend([h1, h2, h3])
hold off
title("Assembley, PickUp and initial Robot Locations")



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
% box_matrix=[10 18 20 50 50 % number of Shaft A needed by assembley stations
%            35 3 1 5 40   % number of Shaft B needed by assembley stations
 %           15 48 7 8 25];  % numbre of screws needed "...................." 
% 
box_matrix=[0	18	20	50	50
0	3	0	0	0
0	48	7	0	0];

% tsa = sum(box_matrix(1,:)) %125
% tsb = sum(box_matrix(2,:)) % 84
% ts = sum(box_matrix(3,:)) % 87

%% cost for transpoting each material for a unit distance
trans_cost=[5 4 2]; % cost for transpotation for a unit distance (shaft A, Shaft B, Screws)
ntravalcost=1;

% ucsa = 5 % cost for transpoting Shaft A for a unit distance
% ucsb = 4 % cost for transpoting shaft B for a unit distance
% ucs  = 2 % cost for transpoting screws for a unit distance


%% Swarm Optimization parameters and population intialisation
n=length(X)+length(xp) ; % number of locations to visit
it=0; % counting itteration
c1 = 1; % Magnitude
c2 = 1; % Magnitude
particles=100; % number of particles
g_best = zeros(1,n+1); % Initiating an array for global best parent, with the last slot holding the best acheived value. 
g_best(1,n+1)= inf; % initially setting the global best acheived result (distance) to be +ve infinity as we are about minimise the distance
particles_best = zeros(particles,n+1); % initiating a matrix for particle best, with size W+1, where the last coloumn consists of the total distance or cost
particles_best(1:particles,n+1)=inf;
velocity = rand(particles, n); % initiating a velocity vector with random number
initialWeight = 0.9; % inital weight for updating the vector
finalWeight = 0.4 ;  % final weight
maxit = 50; % maximum number of itteration to run
weight = 0.5; % weight used to update the vector
TP=zeros(particles,n+1); % Initilaize the total population , popsize*n+1, where column n+1 will be updated as the fitness function later
saNo_of_best_population=zeros(1, maxit+1);
sabest_value=zeros(1,maxit+1);
for j=1:particles 
    TP(j,1:n)=rand(1,n); % each row of TP is a random-key number between (0,1) 
end

%% Encoding , Decoding and Fitness Function
for j = 1:particles
    
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
    tcost = tcost ;%+ Rob_dis_mat(drop_sec(W)) * ntravalcost;
    TP(j,n+1) = tcost; % setting the last coloumn of the population to be the total cost


    if TP(j,n+1) < particles_best(j,n+1)  % if value (distance or cost) acheived is better than the particle best, then the current particle is the particle best 
        particles_best(j,1:n+1) = TP(j,1:n+1);
    end

    if TP(j,n+1) < g_best(1,n+1) % if the value (distance or cost) acheived is better than the global best value, then the current particle is the global best particle
        g_best(1,1:n+1) =  TP(j,1:n+1);
    end
end
sabest_value(1,1) = min(g_best(:, end));
saNo_of_best_population(1,1) = sum(particles_best(:, end) == min(particles_best(:, end)));


%% Main part of Swarm optimisation
while it < maxit 
    it = it + 1;
    weight = initialWeight - (initialWeight - finalWeight) * (it / maxit);

    % update velocity
    for p = 1:particles
        velocity(p,1:n) = weight*velocity(p,1:n)+c1*rand(1,n).*(particles_best(p,1:n)-TP(p,1:n))+ c2*rand(1,n).*(g_best(1:n)-TP(p,1:n));
        TP(p,1:n) = TP(p,1:n) + velocity(p,1:n);
    end
    
    % Again, Encoding , decoding and calculating the fitness function after updating the population


    for j = 1:particles
        
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
        tcost = tcost ;%+ Rob_dis_mat(drop_sec(W)) * ntravalcost;
        TP(j,n+1) = tcost; % setting the last coloumn of the population to be the total cost
    
    
        if TP(j,n+1) < particles_best(j,n+1)  % if value (distance or cost) acheived is better than the particle best, then the current particle is the particle best 
            particles_best(j,1:n+1) = TP(j,1:n+1);
        end
    
        if TP(j,n+1) < g_best(1,n+1) % if the value (distance or cost) acheived is better than the global best value, then the current particle is the global best particle
            g_best(1,1:n+1) =  TP(j,1:n+1);
        end
    end
        
   
    sabest_value(1,it+1) = min(g_best(:, end));
    saNo_of_best_population(1,it+1) = sum(particles_best(:, end) == min(particles_best(:, end)));

end

% TP;
% g_best;
g_best
[~,pick_seq]=sort(g_best(1,(1:length(xp))));
[~,drop_sec]=sort(g_best(1,(4:n)));
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
for g= 1:length(pick_seq)-1
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
start = [xp(pick_seq(end)), yp(pick_seq(end)), pi/2];
goal = [X(drop_sec(1)), Y(drop_sec(1)), pi/2];
grid_start=world2grid(occGrid,start(1:2));
grid_goal=world2grid(occGrid,goal(1:2));
planner = plannerAStarGrid(occGrid);
route = plan(planner,grid_start,grid_goal);
Final_path=[Final_path;route];
% figure;
% show(planner)

% Route to deliver for all assembley stations
for g= 1:length(drop_sec)-1
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
 
% % Route after last assembley station to robot initial location
% 
% start = [X(drop_sec(end)), Y(drop_sec(end)), pi/2];
% goal = [rotbot_initial_pos(1), rotbot_initial_pos(2), pi/2];
% grid_start=world2grid(occGrid,start(1:2));
% grid_goal=world2grid(occGrid,goal(1:2));
% % 
% % % planning the path  - consider only location, not otientation
% planner = plannerAStarGrid(occGrid);
% route = plan(planner,grid_start,grid_goal);
% Final_path = [Final_path; route];


%Final_path=[Final_path(1:end,2)* (0.1) - 20.1, Final_path(1:end,1) * (-0.1) + 19.3]; % converting the values to coordinates in the senario map
Final_path = grid2world(actualmap,Final_path);
% Final_path(:,1)= Final_path(:,1)+1
lastx=Final_path(end,1)
lasty=Final_path(end,2)
% Final_path(:,1) = Final_path(:,1) +0.5; 
%% Ploting Final Route

figure
show(actualmap)
hold on
r = 0.5;
h1 = plot(rotbot_initial_pos(1), rotbot_initial_pos(2), 'go', 'DisplayName', 'Robot Charging Station');

plot([rotbot_initial_pos(1), rotbot_initial_pos(1) + r*cos(pi/2)], [rotbot_initial_pos(2), rotbot_initial_pos(2) + r*sin(pi/2)], 'g-');
h2 = []; 

for p = 1:length(X)
    h2(p) = plot(X(p), Y(p), 'o', 'DisplayName', ['Assembly ', num2str(p)]);
    plot([X(p), X(p) + r*cos(pi/2)], [Y(p), Y(p) + r*sin(pi/2)], '-');
end

for p = 1:length(xp)
    % Use one of the three colors in a cyclic manner
    currentColor = colors(mod(p-1, size(colors, 1)) + 1, :);

    h3(p) = plot(xp(p), yp(p), 'o', 'MarkerFaceColor', currentColor, 'DisplayName', ['Pickup ', num2str(p)]);
end

for q = 1:length(Final_path(1:end,1))-1
    plot([Final_path(q,1), Final_path(q+1,1)], [Final_path(q,2), Final_path(q+1,2)], '--', 'Color', 'k');
end

legend([h1, h2, h3])
title('Tour 5')
hold off

figure;
plot(sabest_value)
title('Best value in each iteration with Particle Swarm Optimisation (PSO)')
xlabel("Iteration")
ylabel("Cost")
figure
plot(saNo_of_best_population)
title('No. of best populaiton in each iteration')
toc