function [ y ] = shadow_cost_function( x )
%SHADOW_COST_FUNCTION Summary of this function goes here
%   Detailed explanation goes here
load('c.mat');

%number of segments
N = size(c,1);

value = c(:,6);
plane_labels = c(:,7);



%we expend x aka labels with an -1, and then set all -1 neighbour indices
%to N+1, as a result, when we perform x(c(:,9:18),1), i.e. when we
%calculate neighbour labels, invalid neighbours will be labeled -1.
x = [x; -1];
c(c == -1) = N+1;

%neighbours is a n*10 matrix, i'th row means the labels of 10 nearest
%neighbours of i'th segment
neighbours = reshape(x(c(:,9:18),1),size(c(:,9:18)));
neighbour_idx = c(:,9:18);

% delete the augmented element
x = x(1:end-1);

% pair(i) is the average label difference between a segment and its neighbour
% labels.
pair = zeros(N,1);
for i = 1:N
    for j = 1:10
        if( neighbours(i,j) >= 0)
            pair(i) = pair(i) + abs( x(i) - neighbours(i,j));
        end
    end
end

has_non_plane_neighbour = zeros(N,1);
non_plane_constraint = 0;
for i = 1:N
    for j = 1:10
        if neighbour_idx (i,j) <= N
            if( plane_labels ( neighbour_idx (i,j)) == 0)
                has_non_plane_neighbour(i) = 1;
            end
        end
    end
    if (has_non_plane_neighbour(i) == 1)
        non_plane_constraint = non_plane_constraint + x(i)*x(i);
	else 
        non_plane_constraint = non_plane_constraint + (1-x(i))*(1-x(i))/50;
    end
end


%(1) darker segments has higher possibility to be shadow
% t1 = 300*norm(x - value - 0.33*ones(size(value)));
t1 = 200*norm(x - value);
%(2) regularizer to push labels towards 0 or 1
t2 = 10*norm( x.*(ones(length(x),1) - x));
%(3) avg. difference betw. the label of a segment and its neighbours.
t3 = 80*norm(plane_labels.*pair);
%(4) if a segment has a non plane neighbour, it has higher possibility to be a shadow
t4 = 500*non_plane_constraint;
  
 y = t1 + t3 + t4;
 % y=t4;
 if( randi(10,1,1,'uint32') == 1)
 str = sprintf('%.2f + %.2f + %.2f + %.2f = %.2f',t1, t2, t3, t4, y);
 disp(str);
end

