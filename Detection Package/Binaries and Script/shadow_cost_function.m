function [ y ] = shadow_cost_function( x )
%SHADOW_COST_FUNCTION Summary of this function goes here
%   Detailed explanation goes here
load('c.mat');
value = c(:,6);
plane_labels = c(:,7);
neighbours = reshape(x(c(:,9:18),1),size(c(:,9:18)));

pair =  sum( abs(repmat(x,1,10) - neighbours ), 2);
object = 0;

 y = 300*norm(x - value) + 10*norm( x.*(ones(length(x),1) - x)) + 50*norm(plane_labels.*pair) ;
 % y = norm(  x.*(ones(length(x),1) - x)   );

 if( randi(100,1,1,'uint32') == 1)
 disp(y);
 end
end

