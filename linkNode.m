% 
% This function takes two joint configurations and the parameters of the
% obstacle as input and calculates whether a collision free path exists
% between them.
% 
% input: q1, q2 -> start and end configuration, respectively. Both are 1x4
%                  vectors.
%        sphereCenter -> 3x1 position of center of sphere
%        r -> radius of sphere
%        rob -> SerialLink class that implements the robot
% output: collision -> binary number that denotes whether this
%                      configuration is in collision or not.
function collision = linkNode(rob,q1,q2)
    
    % initialize
    c = clearNode(rob, q1);
    
    % number of points between q1 and q2
    points = 10;
    
    % create points and check for collision
    for i= 1 : points
        qa = (q1(1) * i + q2(1) * (points-i+1))/(points+1);
        qb = (q1(2) * i + q2(2) * (points-i+1))/(points+1);
        qc = (q1(3) * i + q2(3) * (points-i+1))/(points+1);
        qd = (q1(4) * i + q2(4) * (points-i+1))/(points+1);
        qe = (q1(5) * i + q2(5) * (points-i+1))/(points+1);
        qf = (q1(6) * i + q2(6) * (points-i+1))/(points+1);
        q = [qa qb qc qd qe qf];
        
        c = c | clearNode(rob, q);
        collision = c;
    end
    
end

