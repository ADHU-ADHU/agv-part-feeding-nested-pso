function [tcost,picksequence,dropsequence] = Seq_dis_function2(box_matrix,X,xp,ntravalcost,trans_cost,dis_matx,pick_matrix,Del_matrix,Rob_pick_matrix,Rob_dis_mat,W,triplastpos)
rng(5)
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
maxit = 10; % maximum number of itteration to run
weight = 0.5; % weight used to update the vector
TP=zeros(particles,n+1); % Initilaize the total population , popsize*n+1, where column n+1 will be updated as the fitness function later
No_of_best_population=zeros(1, maxit);
best_value=zeros(1,maxit);
for j=1:particles 
    TP(j,1:n)=rand(1,n); % each row of TP is a random-key number between (0,1) 
end
%% Encoding , Decoding and Fitness Function
for j = 1:particles
    [~,pick_seq]=sort(TP(j,(1:length(xp))));
    [~,drop_sec]=sort(TP(j,(4:n)));
    totweight=zeros(1,length(xp));
   
    % Calculating cost from robot inital location to one of the pick  up
    % point

    for e=1:length(pick_seq)
        if sum(box_matrix(pick_seq(e),:)) < 1
            pick_seq(e) = 0;
        end        
    end

    pick_seq=pick_seq(pick_seq~= 0);
      
    % Calculating cost to travell from robot inital positon / last assemply station from of previous trim to pick from the
    % previous 
    if triplastpos ==0
        tcost = Rob_pick_matrix(pick_seq(1)) * ntravalcost;
        totweight(pick_seq(1)) = sum(box_matrix(pick_seq(1),:));
    elseif triplastpos >0
        tcost = Del_matrix(pick_seq(1),triplastpos) * ntravalcost;
        totweight(pick_seq(1)) = sum(box_matrix(pick_seq(1),:));
    end


    % calculating cost of travell to other pickup points considering the
    % weight
    for i = 1: length(pick_seq)-1
        tcost =  tcost + pick_matrix(pick_seq(i),pick_seq(i+1)) * sum(totweight.*trans_cost);
        totweight(pick_seq(i+1)) = sum(box_matrix(pick_seq(i+1),:));
    end

    for e=1:length(drop_sec)
        if sum(box_matrix(:,drop_sec(e))) < 1
            drop_sec(e) = 0;
        end        
    end
    drop_sec = drop_sec(drop_sec~= 0);
   
    tcost = tcost + Del_matrix(pick_seq(end), drop_sec(1)) * sum(totweight.*trans_cost);
    totweight = totweight - box_matrix(:,drop_sec(1))';
    
    for i = 1:length(drop_sec)-1
        tcost= tcost + dis_matx(drop_sec(i),drop_sec(i+1))* sum(totweight.*trans_cost);
        totweight = totweight - box_matrix(:,drop_sec(i+1))';
    end

    TP(j,n+1) = tcost; % setting the last coloumn of the population to be the total cost


    if TP(j,n+1) < particles_best(j,n+1)  % if value (distance or cost) acheived is better than the particle best, then the current particle is the particle best 
        particles_best(j,1:n+1) = TP(j,1:n+1);
    end

    if TP(j,n+1) < g_best(1,n+1) % if the value (distance or cost) acheived is better than the global best value, then the current particle is the global best particle
        g_best(1,1:n+1) =  TP(j,1:n+1);
    end
end


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
        totweight=zeros(1,length(xp));
       
        % Calculating cost from robot inital location to one of the pick  up
        % point

        for e=1:length(pick_seq)
            if sum(box_matrix(pick_seq(e),:)) < 1
                pick_seq(e) = 0;
            end        
        end

        pick_seq=pick_seq(pick_seq~= 0);
          
        % Calculating cost to travell from robot inital positon / last assemply station from of previous trim to pick from the
        % previous 
        if triplastpos ==0
            tcost = Rob_pick_matrix(pick_seq(1)) * ntravalcost;
            totweight(pick_seq(1)) = sum(box_matrix(pick_seq(1),:));
        elseif triplastpos >0
            tcost = Del_matrix(pick_seq(1),triplastpos) * ntravalcost;
            totweight(pick_seq(1)) = sum(box_matrix(pick_seq(1),:));
        end
    
    
        % calculating cost of travell to other pickup points considering the
        % weight
        for i = 1: length(pick_seq)-1
            tcost =  tcost + pick_matrix(pick_seq(i),pick_seq(i+1)) * sum(totweight.*trans_cost);
            totweight(pick_seq(i+1)) = sum(box_matrix(pick_seq(i+1),:));
        end

        for e=1:length(drop_sec)
            if sum(box_matrix(:,drop_sec(e))) < 1
                drop_sec(e) = 0;
            end        
        end
        drop_sec = drop_sec(drop_sec~= 0);
       
        tcost = tcost + Del_matrix(pick_seq(end), drop_sec(1)) * sum(totweight.*trans_cost);
        totweight = totweight - box_matrix(:,drop_sec(1))';
        
        for i = 1:length(drop_sec)-1
            tcost= tcost + dis_matx(drop_sec(i),drop_sec(i+1))* sum(totweight.*trans_cost);
            totweight = totweight - box_matrix(:,drop_sec(i+1))';
        end
    
        TP(j,n+1) = tcost; % setting the last coloumn of the population to be the total cost
    
    
        if TP(j,n+1) < particles_best(j,n+1)  % if value (distance or cost) acheived is better than the particle best, then the current particle is the particle best 
            particles_best(j,1:n+1) = TP(j,1:n+1);
        end
    
        if TP(j,n+1) < g_best(1,n+1) % if the value (distance or cost) acheived is better than the global best value, then the current particle is the global best particle
            g_best(1,1:n+1) =  TP(j,1:n+1);
        end
    end
end
tcost=g_best(n+1);
[~,picksequence]=sort(g_best(1,(1:length(xp))));
[~,dropsequence]=sort(g_best(1,(length(xp)+1:n)));
picksequence=picksequence;
dropsequence=dropsequence;
tcost;
end









