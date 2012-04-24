function varargout = MAPPING(varargin)
% MAPPING MATLAB code for MAPPING.fig
%      MAPPING, by itself, creates a new MAPPING or raises the existing
%      singleton*.
%
%      H = MAPPING returns the handle to a new MAPPING or the handle to
%      the existing singleton*.
%
%      MAPPING('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MAPPING.M with the given input arguments.
%
%      MAPPING('Property','Value',...) creates a new MAPPING or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before MAPPING_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to MAPPING_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help MAPPING

% Last Modified by GUIDE v2.5 19-Apr-2012 11:27:18

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @MAPPING_OpeningFcn, ...
                   'gui_OutputFcn',  @MAPPING_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before MAPPING is made visible.
function MAPPING_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to MAPPING (see VARARGIN)

% Choose default command line output for MAPPING
handles.output = hObject;
% Update handles structure
% --- Executes on button press in push_sensors.
% hObject    handle to push_sensors (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get robot object to query sensors and output to command window
obj= get(handles.text_title,'UserData');
[rad rIR rSon rLid angRLid numPtsLid]= getConstants(obj);

% Bump sensors
bump= genBump(obj);
bumpR= bump(1);
bumpF= bump(2);
bumpL= bump(3);
fprintf('Bump Sensors:\n')
fprintf('\tRight: %.0f\n',bumpR)
fprintf('\tFront: %.0f\n',bumpF)
fprintf('\tLeft: %.0f\n\n',bumpL)

% Cliff sensors
cliffR= 0;
cliffFR= 0;
cliffFL= 0;
cliffL= 0;
cliff= genCliff(obj);
cliffRstr= cliff(1);
cliffFRstr= cliff(2);
cliffFLstr= cliff(3);
cliffLstr= cliff(4);
fprintf('Cliff Sensors:\n')
fprintf('\tRight: State %.0f Strength %%%.3f\n',cliffR,cliffRstr)
fprintf('\tFront-Right: State %.0f Strength %%%.3f\n',cliffFR,cliffFRstr)
fprintf('\tFront-Left: State %.0f Strength %%%.3f\n',cliffFL,cliffFLstr)
fprintf('\tLeft: State %.0f Strength %%%.3f\n\n',cliffL,cliffLstr)

% Infrared wall sensor
wall= genIR(obj);
fprintf('Wall Sensor: %.0f\n\n',wall)

% Virtual Wall sensor
vwall= genVWall(obj);
fprintf('Virtual Wall Sensor: %.0f\n\n',vwall)

% Sonar sensors
distSonar= genSonar(obj);
fprintf('Sonar Sensors:\n')
fprintf('\tFront: %.3f m\n',distSonar(1))
fprintf('\tLeft: %.3f m\n',distSonar(2))
fprintf('\tRear: %.3f m\n',distSonar(3))
fprintf('\tRight: %.3f m\n\n',distSonar(4))

% LIDAR sensor
handleGUI= gcf;     % Get GUI figure handle to switch back to
distLidar= genLidar(obj);
angleData= linspace(pi/2-angRLid/2,pi/2+angRLid/2,numPtsLid);
figure
hold on
polar(linspace(0,2*pi,11),rad*ones(1,11),'b-')
polar([0 pi/2],[0 1.5*rad],'b-')
polar([angleData(1) 0 angleData(end)],[distLidar(1) 0 distLidar(end)],'k-')
polar(angleData,distLidar,'k.')
axis([-10 10 -10 10])
title('LIDAR Data')
xlabel('Distance (m)')
ylabel('Distance (m)')
hold off
figure(handleGUI)   % Switch back to GUI figure
fprintf('LIDAR Sensor: see figure\n\n')

% Odometry
distOdom= genOdomDist(obj);
angOdom= genOdomAng(obj);
fprintf('Odometry Data (since last call):\n')
fprintf('\tDistance: %.3f m\n',distOdom)
fprintf('\tAngle: %.3f m\n\n',angOdom)

% Overhead localization system
[x y th]= genOverhead(obj);
fprintf('Overhead Localization System output:\n')
fprintf('\tX-Coordinate: %.3f m\n',x)
fprintf('\tY-Coordinate: %.3f m\n',y)
fprintf('\tAngle relative to horizontal: %.3f rad\n\n',th)

% Camera
[angCam distCam colorCam]= genCamera(obj);
fprintf('Camera Data:\n')
for i= 1:length(angCam)
    fprintf('Beacon at %.0f deg, %.3f m, with color [%.2f %.2f %.2f]\n',...
        rad2deg(angCam(i)),distCam(i),colorCam(i,1),colorCam(i,2),...
        colorCam(i,3))
end
fprintf('\n')
guidata(hObject, handles);

% UIWAIT makes MAPPING wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = MAPPING_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in mapper.
function mapper_Callback(hObject, eventdata, handles)
% hObject    handle to mapper (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
