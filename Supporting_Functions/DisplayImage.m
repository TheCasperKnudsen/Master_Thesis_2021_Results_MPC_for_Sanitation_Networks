function DisplayImage(string)
%DISPLAYIMAGE simply displays image without axis - Input should be a string
img = imread(string);
image(img);
set(gca,'visible','off');
clear img
end

