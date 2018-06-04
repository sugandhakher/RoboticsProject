% 
% Evaluate whether the configuration <q> is in collision with a spherical
% obstacle centered at <sphereCenter> with radius <r>.
% 
% input: q -> 1x6 vector of joint angles 
% output: collision -> binary number that denotes whether this
%                      configuration is in collision or not.
function collision = clearNode(rob,q)

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
    
    % get points on elbow and at EE
    x1 = [0;0;0];
    T2 = rob.A(1,q) * rob.A(2,q) * rob.A(3,q);
    x2 = T2(1:3,4);
    T3 = rob.A(1,q) * rob.A(2,q) * rob.A(3,q) * rob.A(4,q);
    x3 = T3(1:3,4);
        
    vec = 0:0.1:1;
    m = size(vec,2);
    
    x12 = repmat(x2-x1,1,m) .* repmat(vec,3,1) + repmat(x1,1,m);
    x23 = repmat(x3-x2,1,m) .* repmat(vec,3,1) + repmat(x2,1,m);
    x = [x12 x23];
    
    flag1 = 0;
    flag2 = 0;
    flag3 = 0;
    flag4 = 0;
    flag5 = 0;
    
    if sum(sum((x - repmat(sphereCenter1,1,size(x,2))).^2,1) < sphereRadius1^2) > 0
        flag1 = 1;
    end
    
    if sum(sum((x - repmat(sphereCenter2,1,size(x,2))).^2,1) < sphereRadius2^2) > 0
        flag2 = 1;
    end
    
    if sum(sum((x - repmat(sphereCenter3,1,size(x,2))).^2,1) < sphereRadius3^2) > 0
        flag3 = 1;
    end
    
    if sum(sum((x - repmat(sphereCenter4,1,size(x,2))).^2,1) < sphereRadius4^2) > 0
        flag4 = 1;
    end
    
    if sum(sum((x - repmat(sphereCenter5,1,size(x,2))).^2,1) < sphereRadius5^2) > 0
        flag5 = 1;
    end
    
    if (flag1 == 1) || (flag2 == 1) || (flag3 == 1) || (flag4 == 1) || (flag5 == 1)
        collision = 1;
    else
        collision = 0;
    end

end
