clear all;
% Define the root folder for the images
HOMEIMAGES = '.\IGNORE\Images'; % you can set here your default folder
HOMEANNOTATIONS = '.\IGNORE\Annotations'; % you can set here your default folder
folderlist = {'users/shengshuyang//usc_table_top'};
LMinstall (folderlist, HOMEIMAGES, HOMEANNOTATIONS);
D = LMdatabase(HOMEANNOTATIONS);

% Now you can visualize the images
LMplot(D, 1, HOMEIMAGES);

% Or read an image
[annotation, img] = LMread(D, 1, HOMEIMAGES);
[mask, class] = LMobjectmask(annotation, HOMEIMAGES);
image = double(img);
image(:,:,1) =  image(:,:,1)  + 100*double(mask);
imagesc(uint8(image));