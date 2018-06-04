% CS 5335 Robotics
% Final Project
% Implementing PRM on Puma 560 using Robotics Toolbox
% Created by Deepak Bansal
%        and Sugandha Kher

% This is main file which should be run to execute the program
% Requirements to run this program:
%       1. MATLAB
%       2. Peter Corke's Robotics Toolbox 9.10
%       3. Bioinformatics Toolbox
%
% How to run:
%       1. Run command: startup_rvc
%       2. Run: main()

% main function
function main()

    % create robot
    rob = createRobot();
    
    % start position
    qStart = [0 0 0 0 0 0];
    
    % target position
	xGoal = [0.5;0.0;-0.5];
    distance = [0.25 0.25 0.25 0.25 0.25 0.25];
    rob.plot(qStart);
    hold on;
    createObstacles();
    
    % set range
    min = -pi;
    max = pi;
    
    % number of sample nodes
    total_nodes = 1000;
    
    % initialize vertices
    vertices = [];
    
    % initialize edges
    edgesX = [];
    edgesY = [];
    
    % weights
    weights = [];
    
    % goal matrix
    goal = rob.ikine(transl(xGoal),zeros(1,6),[1,1,1,0,0,0]);
    
    % adding vertices
    times = 0;  % try it 10 times before exiting in case can't get any vertices
    while isempty(vertices)
        times = times + 1;
        % run for as many samples defined in total_nodes
        for i=1:total_nodes
            sample = (max-min).*rand(1,6) + min; % take sample
            if clearNode(rob,sample) == 0 % check for collision
                vertices = [vertices;sample]; % add to vertices set
                [r,c] = size(vertices);
                nq = []; % a set of nodes in vertices that are close to sample
                for j = 1:1:r-1
                    dist = sample - vertices(j,1:6);
                    if dist < distance
                        nq = [nq; j,r];
                    end
                end
                sortrows(nq);
                [a,b] = size(nq);
                % add edges
                for k = 1:1:a
                    if linkNode(rob,vertices(nq(k,2),1:6),vertices(nq(k,1),1:6)) == 0 % check for collision
                        edgesX = [edgesX;round(nq(k,2))];
                        edgesY = [edgesY;round(nq(k,1))];
                        weights = [weights;vertices(nq(k,1))-vertices(nq(k,2))];
                    end
                end
            end        
        end
        if times == 50
            break
        end
    end
    
    if isempty(vertices)
        message = "Could not create clear vertices in these samples. Try running again."
        return
    end
    
    % add source
    vertices = [vertices;qStart];
    [r,c] = size(vertices);
    nq = [];
    within = distance - 0.05;
    times = 0;
    while isempty(nq) 
        times = times + 1;
        within = within + 0.05;
        for j = 1:1:r-1
            dist = qStart - vertices(j,1:6);
            if dist < within
                nq = [nq; j,r];
            end
        end
        if times == 10
            break
        end
    end
    if isempty(nq)
        message = "Could not find vertices close to source"
        return
    else
        sourceX = [];
        sourceY = [];
        sourceW = [];
        sortrows(nq);
        [a,b] = size(nq);
        for k = 1:1:a
            if linkNode(rob,vertices(nq(k,2),1:6),vertices(nq(k,1),1:6)) == 0
                sourceX = [sourceX;round(nq(k,2))];
                sourceY = [sourceY;round(nq(k,1))];
                sourceW = [sourceW;vertices(nq(k,1))-vertices(nq(k,2))];
            end
        end
        if isempty(sourceX) || isempty(sourceY) || isempty(sourceW)
            message = "Could not find a clear edge from source to any close vertex."
            return
        else 
            edgesX = [edgesX;sourceX];
            edgesY = [edgesY;sourceY];
            weights = [weights;sourceW];
        end
    end
    
    % add destination
    vertices = [vertices;goal];
    [r,c] = size(vertices);
    nq = [];
    within = distance - 0.05;
    times = 0;
    while isempty(nq)
        times = times + 1;
        within = within + 0.05;
        for j = 1:1:r-1
            dist = vertices(j,1:6) - goal;
            if dist < within
                nq = [nq; j,r];
            end
        end
        if times == 10
            break
        end
    end
    if isempty(nq)
        message = "Could not find vertices close to destination"
        return
    else
        destX = [];
        destY = [];
        destW = [];
        sortrows(nq);
        [a,b] = size(nq);
        for k = 1:1:a
            if linkNode(rob,vertices(nq(k,2),1:6),vertices(nq(k,1),1:6)) == 0
                destX = [destX;fix(nq(k,1))];
                destY = [destY;fix(nq(k,2))];
                destW = [destW;vertices(nq(k,1))-vertices(nq(k,2))];
            end
        end
        if isempty(destX) || isempty(destY) || isempty(destW)
            message = "Could not find a clear edge from destination to any close vertex."
            return
        else 
            edgesX = [edgesX;destX];
            edgesY = [edgesY;destY];
            weights = [weights;destW];
        end
    end
    
    weights = abs(weights);
    
    [r,c] = size(vertices);
    G = graph(edgesX',edgesY',weights');
    
    [P,d] = shortestpath(G,r-1,r);
    
    if isempty(P)
        message = "Could not find a path from source to destination."
        return
    end
    
    [u,v] = size(P);

    milestones = [];
    for t = 1:1:v
        milestones = [milestones;vertices(P(t),1:6)];
    end
    qTraj = interpMilestones(milestones);
    rob.plot(qTraj);
    
    message = "Reached Destination Successfully"
    
end

function createObstacles()
    sphereCenter1 = [0.8;0.4;0.75];
    sphereRadius1 = 0.3;

    sphereCenter2 = [-0.8;-0.4;-0.8];
    sphereRadius2 = 0.2;
    
    sphereCenter3 = [-0.25;-0.4;0];
    sphereRadius3 = 0.15;
    
    sphereCenter4 = [0.8;-0.9;-0.75];
    sphereRadius4 = 0.25;
    
    sphereCenter5 = [0.8;0.2;-0.75];
    sphereRadius5 = 0.25;
    	
    drawSphere(sphereCenter1,sphereRadius1);
    drawSphere(sphereCenter2,sphereRadius2);
    drawSphere(sphereCenter3,sphereRadius3);
    drawSphere(sphereCenter4,sphereRadius4);
    drawSphere(sphereCenter5,sphereRadius5);
end

function drawSphere(position,diameter)

    % diameter = 0.1;
    [X,Y,Z] = sphere;
    X=X*diameter;
    Y=Y*diameter;
    Z=Z*diameter;
    X=X+position(1);
    Y=Y+position(2);
    Z=Z+position(3);
    surf(X,Y,Z);

    %~ shading flat

end

function traj = interpMilestones(qMilestones)

    d = 0.05;
%     traj = qMilestones(1,:);
    traj = [];
    for i=2:size(qMilestones,1)

        delta = qMilestones(i,:) - qMilestones(i-1,:);
        m = max(floor(norm(delta) / d),1);
        vec = linspace(0,1,m);
        leg = repmat(delta',1,m) .* repmat(vec,size(delta,2),1) + repmat(qMilestones(i-1,:)',1,m);
        traj = [traj;leg'];

    end
end