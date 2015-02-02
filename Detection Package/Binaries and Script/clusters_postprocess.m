clear all;
%% preprocessing cluster data
c = load('clusters_statistics.txt',' ');
N = size(c,1);
% temp is storing connection information
temp = c(:,9:18);
% -1 was used to represent no neighbour. setting them to -2 then add temp
% by 1 will add an offset to the neighbour indices. That's because in C++
% the indices starts from zero, and we must offset by one here.
temp(temp == -1) = -2;
temp = temp +1;
c(:,9:18)= temp;
% for i=1:1:size(c,1)
%     temp = c(i,:);
%     temp(temp==-1) = i;
%     c(i,:) = temp;
% end

% switch coordinates
c = [c(:,1) c(:,3) c(:,2) c(:,4:18)];
save('c.mat','c');

% structure of c: [ x y z h s v plane_label size neighbours]

% colormap('prism');
% scatter3(c(:,1),c(:,2),c(:,3),max(c(:,8)/50, 10),c(:,7),'filled');
% for i = 1:N
%     for j = 9:18
%         if c(i,j) > 0 && c(i,7) == 0
%             idx = c(i,j);
%             x = [ c(i,1) c(idx,1)];
%             y = [ c(i,2) c(idx,2)];
%             z = [ c(i,3) c(idx,3)];
%             hold on;
%             plot3(x,y,z);
%         end
%         
%     end
% end
 
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for each cluster, we check cdf wrt its cluster center color value, 
% this is saying, if the value is small, the success rate of picking a 1 is
% small. so we use bernoulli distribution to pick a value with this success
% rate.
value = c(:,6);
pdf = hist(value,100);
cdf(1) = pdf(1);
for i=2:1:length(pdf);
    cdf(i) = cdf(i-1)+pdf(i);
end
cdf  = cdf/max(cdf);

value = round(c(:,6)*100);
value = max(value,1);

p = cdf(value);
labels  = binornd(1,p)';
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure;colormap('prism');
% scatter3(c(:,1),c(:,2),c(:,3),c(:,8)/50,255*labels,'filled');   
% figure;colormap('prism');
% scatter3(c(:,1),c(:,2),c(:,3),c(:,8)/50,255*c(:,7),'filled');   
%%
options = optimset('MaxIter',10);
% x = fmincon(@shadow_cost_function,labels,[],[],[],[],...
%     zeros(length(labels),1),1.1*ones(length(labels),1),[],options);
x = fminunc(@shadow_cost_function,labels,options);
xx = round(x);

for i = 1:1:length(xx)
    if c(i,7) == 0
        xx(i) = 2;
    end
end

%%
colormap('prism');
scatter3(c(:,1),c(:,2),c(:,3),c(:,8)/50,xx,'filled');
labels = xx;
fid = fopen('labels.txt', 'wt');
for i = 1:length(labels)
    fprintf(fid,'%g',labels(i));
    if i~=length(labels)
        fprintf(fid,'\n');
    end
end
fclose(fid);
%note: 1 means bright and not shadow, 0 means shadow. 2 means its part of
%the object

quit()



