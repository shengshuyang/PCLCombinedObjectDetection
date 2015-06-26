% clear all;
% Define the root folder for the images
HOMEIMAGES = '.\USC_SHADOW\Images'; % you can set here your default folder
HOMEANNOTATIONS = '.\USC_SHADOW\Annotations'; % you can set here your default folder
folderlist = {'users/shengshuyang//usc_table_top'};
LMinstall (folderlist, HOMEIMAGES, HOMEANNOTATIONS);
D = LMdatabase(HOMEANNOTATIONS);

% Now you can visualize the images
LMplot(D, 1, HOMEIMAGES);

% Or read an image
[annotation, img] = LMread(D, 25, HOMEIMAGES);
[mask, class] = LMobjectmask(annotation, HOMEIMAGES);
image = double(img);
mask = sum(mask,3);
image(:,:,1) =  image(:,:,1)  + 50*double(mask);
imagesc(uint8(image));