% This example will create a surface describing the function:
% z = f(x,y) = x^2-y^2, over the range -5<x<5, -5<y<5 
% Define the data: 
% First define the ranges and resolution of the grid space:
xmin = 0;
xmax = 25;
ymin = 0;
ymax = 25;
gridresolution = 50; 


% Use linspace to define a vector of points representing
% the ranges of the x- and y-dimensions:
x = linspace(xmin, xmax, gridresolution);
y = linspace(ymin, ymax, gridresolution); 


% Use meshgrid to calculate the grid matrices:
% for the x- and y-coordinates
[X,Y] = meshgrid(x,y); 


% Now calculate the function using array operations to manipulate
% the matrices element by element. The result is a matrix of Z-data,
% the same size as the X and Y grid matrices:
Z = X.*1 + Y.*0; 


% Plot the results by drawing the surface:
figure
surf(X,Y,Z)
xlabel X; ylabel Y; zlabel Z;
colormap()
