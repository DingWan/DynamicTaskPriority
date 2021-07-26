function varargout = GUI_10(varargin)
%GUI_10 MATLAB code file for GUI_10.fig
%      GUI_10, by itself, creates a new GUI_10 or raises the existing
%      singleton*.
%
%      H = GUI_10 returns the handle to a new GUI_10 or the handle to
%      the existing singleton*.
%
%      GUI_10('Property','Value',...) creates a new GUI_10 using the
%      given property value pairs. Unrecognized properties are passed via
%      varargin to GUI_10_OpeningFcn.  This calling syntax produces a
%      warning when there is an existing singleton*.
%
%      GUI_10('CALLBACK') and GUI_10('CALLBACK',hObject,...) call the
%      local function named CALLBACK in GUI_10.M with the given input
%      arguments.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUI_10

% Last Modified by GUIDE v2.5 19-Dec-2016 19:08:42

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUI_10_OpeningFcn, ...
                   'gui_OutputFcn',  @GUI_10_OutputFcn, ...
                   'gui_LayoutFcn',  [], ...
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

% --- Executes just before GUI_10 is made visible.
function GUI_10_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% Choose default command line output for GUI_10
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes GUI_10 wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% --- Outputs from this function are returned to the command line.
function varargout = GUI_10_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes1

% --- Executes during object creation, after setting all properties.
function select_M_CreateFcn(hObject, eventdata, handles)
% hObject    handle to select_M (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% --- Executes during object deletion, before destroying properties.
function select_M_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to select_M (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --------------------------------------------------------------------
function select_M_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to select_M (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes when select_M is resized.
function select_M_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to select_M (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in simulation.
function simulation_Callback(hObject, eventdata, handles)
% hObject    handle to simulation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clc
warning off
%get the mode informtion, here only the "portal3+PA10" mode is available,
%the other two modes are similar
select_M_portal3 = get(handles.select_M_portal3,'Value');
select_M_PA10 = get(handles.select_M_PA10,'Value');
%set the position of figure
axes(handles.axes1);
%global variable:limitation of joint angle range
global qdotlim
%% simulation under different mode
if select_M_portal3 == 1 && select_M_PA10 == 1
%% Create Symbolic Variables
syms q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 real
deg = pi/180;
%% Create redundant robot
L(1) = Link([pi/2 0 0 pi/2 1]); L(1).qlim = [0,30];
L(2) = Link([pi/2 0 0 pi/2 1]); L(2).qlim = [0,30];
L(3) = Link([pi/2 0 0 pi/2 1]);L(3).qlim = [0,30];
L(4)= Revolute('d', 3.15, 'a', 0, 'alpha', -pi/2 , ...
    'qlim', [-160 160]*deg);
L(5) = Revolute('d', 0, 'a', 0, 'alpha', pi/2, ...
    'qlim', [-160 160]*deg);
L(6) =  Revolute('d', 4.5, 'a', 0, 'alpha', -pi/2 , ...
    'qlim', [-160 160]*deg);
L(7) =  Revolute('d', 0, 'a', 0, 'alpha', pi/2, ...
    'qlim', [-160 160]*deg);
L(8) =  Revolute('d', 5, 'a', 0, 'alpha', -pi/2 , ...
    'qlim', [-160 160]*deg);
L(9) =  Revolute('d', 0, 'a', 0, 'alpha', pi/2 , ...
    'qlim', [-160 160]*deg);    
L(10) =  Revolute('d', 0.8, 'a', 0, 'alpha', 0, ...
    'qlim', [-160 160]*deg);

PR10 = SerialLink(L,'name','Protal3-PA10');

%% plot robot
q = [q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 ];
load('E:\\GPM\\q_ie.mat','q_ie');
q_r = q_ie(2,:); % ready pose, arm up
q_n = q_ie(1,:);
load('E:\\GPM\\teach_q_temporarydata.mat','aa');
q_s=aa;
%q_s = [10 5 10  0 -pi/4  pi/4 4*pi/9 -pi/3 0 pi/4 ];
PR10.plot(q_s,'workspace',[-10 20 -20 20 -5 30],'jointdiam', 2,'scale',0.2);
hold on;
%% Forward kinematics
FT0 = PR10.fkine(q_n);
FT1 = PR10.fkine(q_r);
%% Cartesian Trajectory
n = 50;
traj_1 = ctraj(FT0,FT1,n);
for i = 1:n;
xtraj(i) = traj_1(1,4,i);
ytraj(i) = traj_1(2,4,i);
ztraj(i) = traj_1(3,4,i);
end
plot3(xtraj,ytraj,ztraj)
%% get the distance function gradient;Property index: H(q):distance
%obstacle coordinate
 obs=[handles.obstacle_position_X,handles.obstacle_position_Y,handles.obstacle_position_Z];
 ini_point=[0,0,0];
%get the coordinates of each joint
 position1=first_joint_position_compute(q);
 position2=second_joint_position_compute(q);
 position3=third_joint_position_compute(q);
 position4=shoulder_joint_position_compute(q);%4 and 3 same
 position5=elbow_joint_position_compute(q);
 position6=wrist_joint_position_compute(q);
%when the distance between the obstacle and the initial point is min;
        fun_d1=DistBetween2Segment(obs,obs,ini_point,ini_point);
        gradient1=jacobian(fun_d1,q);
%when the distance between the obstacle and the first link is min;
        fun_d3=qiudiandaozhixianjuli(ini_point,transpose(position1),obs);
        gradient3=jacobian(fun_d3,q);
%when the distance between the obstacle and the first joint is min;
        fun_d2=sqrt((position1(1)-obs(1))^2+(position1(2)-obs(2))^2+(position1(3)-obs(3))^2);
        gradient2=jacobian(fun_d2,q);
%when the distance between the obstacle and the second link is min; 
        fun_d5=qiudiandaozhixianjuli(position2,transpose(position1),obs);
        gradient5=jacobian(fun_d5,q);
%when the distance between the obstacle and the second joint is min;
        fun_d4=sqrt((position2(1)-obs(1))^2+(position2(2)-obs(2))^2+(position2(3)-obs(3))^2);
        gradient4=jacobian(fun_d4,q);
%when the distance between the obstacle and the third link is min;
        fun_d7=qiudiandaozhixianjuli(transpose(position2),transpose(position3),obs);
        gradient7=jacobian(fun_d7,q);
%when the distance between the obstacle and the third joint(or the shoulder) is min;
        fun_d6=sqrt((position3(1)-obs(1))^2+(position3(2)-obs(2))^2+(position3(3)-obs(3))^2);
        gradient6=jacobian(fun_d6,q);
%when the distance between the obstacle and the forth link(shouler and elbow) is min;
        fun_d9=qiudiandaozhixianjuli(transpose(position5),transpose(position4),obs);
        gradient9=jacobian(fun_d9,q);
%when the distance between the obstacle and the elbow is min;
        fun_d8=sqrt((position5(1)-obs(1))^2+(position5(2)-obs(2))^2+(position5(3)-obs(3))^2);
        gradient8=jacobian(fun_d8,q);
%when the distance between the obstacle and the fifth link(wrist and elbow) is min;
        fun_d11=qiudiandaozhixianjuli(transpose(position5),transpose(position6),obs);
        gradient11=jacobian(fun_d11,q);
%when the distance between the obstacle and the wrist is min;
        fun_d10=sqrt((position6(1)-obs(1))^2+(position6(2)-obs(2))^2+(position6(3)-obs(3))^2);
        gradient10=jacobian(fun_d10,q);
%% Property index: H(q):joint limit, function is in calculation step
qmin = [L(1).qlim(:,1),L(2).qlim(:,1),L(3).qlim(:,1),...
        L(4).qlim(:,1),L(5).qlim(:,1),L(6).qlim(:,1),...
        L(7).qlim(:,1),L(8).qlim(:,1),L(9).qlim(:,1),...
        L(10).qlim(:,1)];
qmax = [L(1).qlim(:,2),L(2).qlim(:,2),L(3).qlim(:,2),...
        L(4).qlim(:,2),L(5).qlim(:,2),L(6).qlim(:,2),...
        L(7).qlim(:,2),L(8).qlim(:,2),L(9).qlim(:,2),...
        L(10).qlim(:,2)];
%% Method II: Gradient Projection Method, main process is in "waitbar"
factor = 1;
k = -0.01;
alpha = -0.5;
error = 1;
j = 0;
%plot the ball obstacle
obs=[handles.obstacle_position_X,handles.obstacle_position_Y,handles.obstacle_position_Z];
d0 = handles.obstacle_position_R;
x0 = obs(1);
y0 = obs(2);
z0 = obs(3);
[x,y,z]=sphere;
mesh(d0*x+x0,d0*y+y0,d0*z+z0);
hold all;
%% general method of getting the singularity/jacobian gradient. due to computing complexity, the ready-made gradient  
%Jacob = PR13.jacob0(q)
%Jacob = PR13.jacob0(q_n,'eul');
%det(Jacob'*Jacob)£»
%det(Jacob*Jacob')£»
%%singularity
%W=1/sqrt(det(Jacob*Jacob'));
%gradient=jacobian(W,q)

tic % time
run Waitbar.m
toc
q_plot1=q_plot;
PR10.plot(q_plot1,'workspace',[-10 20 -20 20 -5 30],'scale',0.5);% plot
% save the joint angle matrix
save('E:\\GPM\\temporarydata.mat','q_plot1');
else if select_M_portal3 == 1 && select_M_PA10 == 0% other modes
    else if select_M_portal3 == 0 && select_M_PA10 == 1
        end
    end
end      
% --- Executes on button press in load.
function load_Callback(hObject, eventdata, handles)
% hObject    handle to load (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

run E:\\GPM\\rvctools\\startup_rvc.m
clc
[pname,adrname]=uigetfile('*mat');
   if exist(strcat(adrname,pname))
       a=strcat(adrname,pname);
       load(a,'q_ie');
   else 
       return;
   end;

axes(handles.axes1);
select_M_portal3 = get(handles.select_M_portal3,'Value');
select_M_PA10 = get(handles.select_M_PA10,'Value');
% Update handles structure
guidata(hObject, handles);
deg = pi/180;      
if select_M_portal3 == 1 && select_M_PA10 == 0
else if select_M_portal3 == 0 && select_M_PA10 == 1        
    else if select_M_portal3 == 1 && select_M_PA10 == 1
            syms q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 real
            L(1) = Link([pi/2 0 0 pi/2 1]); L(1).qlim = [0,30];
            L(2) = Link([pi/2 0 0 pi/2 1]); L(2).qlim = [0,30];
            L(3) = Link([pi/2 0 0 pi/2 1]); L(3).qlim = [0,30];
            L(4)= Revolute('d', 3.15, 'a', 0, 'alpha', -pi/2 , ...
                    'qlim', [-160 160]*deg);
            L(5) = Revolute('d', 0, 'a', 0, 'alpha', pi/2, ...
                    'qlim', [-160 160]*deg);
            L(6) =  Revolute('d', 4.5, 'a', 0, 'alpha', -pi/2 , ...
                    'qlim', [-160 160]*deg);
            L(7) =  Revolute('d', 0, 'a', 0, 'alpha', pi/2, ...
                    'qlim', [-160 160]*deg);
            L(8) =  Revolute('d', 5, 'a', 0, 'alpha', -pi/2 , ...
                    'qlim', [-160 160]*deg);
            L(9) =  Revolute('d', 0, 'a', 0, 'alpha', pi/2 , ...
                    'qlim', [-160 160]*deg);    
            L(10) =  Revolute('d', 0.8, 'a', 0, 'alpha', 0, ...
                    'qlim', [-160 160]*deg);
            PR10 = SerialLink(L,'name','Protal3-PA10');
            q = [q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 ];
            q_r = q_ie(2,:); % ready pose, arm up
            q_n = q_ie(1,:);
            PR10.plot(q_r,'workspace',[-10 20 -20 20 -5 30],'jointdiam', 2,'scale',0.2);
            hold on;
            FT0 = PR10.fkine(q_n);
            FT1 = PR10.fkine(q_r);
            n = 50;
            traj_1 = ctraj(FT0,FT1,n);
            for i = 1:n;
            xtraj(i) = traj_1(1,4,i);
            ytraj(i) = traj_1(2,4,i);
            ztraj(i) = traj_1(3,4,i);
            end
            plot3(xtraj,ytraj,ztraj)
            hold on;
        obs=[handles.obstacle_position_X,handles.obstacle_position_Y,handles.obstacle_position_Z];
        d0 = handles.obstacle_position_R;
        x0 = obs(1);
        y0 = obs(2);
        z0 = obs(3);
        [x,y,z]=sphere;
        mesh(d0*x+x0,d0*y+y0,d0*z+z0)
        hold on;
        %set value
        set(handles.path_origin_point_X_input,'string',num2str(FT0(1,4)));
        set(handles.path_origin_point_Y_input,'string',num2str(FT0(2,4)));
        set(handles.path_origin_point_Z_input,'string',num2str(FT0(3,4)));
        set(handles.path_origin_point_phi_input,'string',num2str(q_n(8)));
        set(handles.path_origin_point_theta_input,'string',num2str(q_n(9)));
        set(handles.path_origin_point_psi_input,'string',num2str(q_n(10))); 
        
        set(handles.path_end_point_X_input,'string',num2str(FT1(1,4)));
        set(handles.path_end_point_Y_input,'string',num2str(FT1(2,4)));
        set(handles.path_end_point_Z_input,'string',num2str(FT1(3,4)));
        set(handles.path_end_point_phi_input,'string',num2str(q_r(8)));
        set(handles.path_end_point_theta_input,'string',num2str(q_r(9)));
        set(handles.path_end_point_psi_input,'string',num2str(q_r(10))); 
        end 
    end
end

% --- Executes during object creation, after setting all properties.
function obstalce_position_X_CreateFcn(hObject, eventdata, handles)
% hObject    handle to obstalce_position_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% --- Executes on button press in select_C_o.
function select_C_o_Callback(hObject, eventdata, handles)
% hObject    handle to select_C_o (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
select_C_o = get(handles.select_C_o,'Value')

if select_C_o == 0

        set(handles.obstacle_position_X_input,'Enable','Off')
        set(handles.obstacle_position_Y_input,'Enable','Off')
        set(handles.obstacle_position_Z_input,'Enable','Off')
        set(handles.obstacle_position_R_input,'Enable','Off')
        %set(handles.chkCrackers,'Value',0)
        %set(handles.chkRelish,'Enable','On')
else   
        set(handles.obstacle_position_X_input,'Enable','On')
        set(handles.obstacle_position_Y_input,'Enable','On')
        set(handles.obstacle_position_Z_input,'Enable','On')
        set(handles.obstacle_position_R_input,'Enable','On')
        set(handles.closest_Link,'Enable','On')
        set(handles.closest_distance,'Enable','On')
end
% Hint: get(hObject,'Value') returns toggle state of select_C_o

% --- Executes on button press in select_M_portal3.
function select_M_portal3_Callback(hObject, eventdata, handles)
% hObject    handle to select_M_portal3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of select_M_portal3

function path_end_point_psi_input_Callback(hObject, eventdata, handles)
% hObject    handle to path_end_point_psi_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(get(hObject,'String'))
    path_end_point_psi = str2double(get(hObject,'String'));
    handles.path_end_point_psi = path_end_point_psi;
end
guidata(hObject, handles);
% Hints: get(hObject,'String') returns contents of path_end_point_psi_input as text
%        str2double(get(hObject,'String')) returns contents of path_end_point_psi_input as a double

% --- Executes during object creation, after setting all properties.
function path_end_point_psi_input_CreateFcn(hObject, eventdata, handles)
% hObject    handle to path_end_point_psi_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function path_end_point_Z_input_Callback(hObject, eventdata, handles)
% hObject    handle to path_end_point_Z_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(get(hObject,'String'))
    path_end_point_Z = str2double(get(hObject,'String'));
    handles.path_end_point_Z = path_end_point_Z;
end
guidata(hObject, handles);
% Hints: get(hObject,'String') returns contents of path_end_point_Z_input as text
%        str2double(get(hObject,'String')) returns contents of path_end_point_Z_input as a double

% --- Executes during object creation, after setting all properties.
function path_end_point_Z_input_CreateFcn(hObject, eventdata, handles)
% hObject    handle to path_end_point_Z_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function path_end_point_theta_input_Callback(hObject, eventdata, handles)
% hObject    handle to path_end_point_theta_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(get(hObject,'String'))
    path_end_point_theta = str2double(get(hObject,'String'));
    handles.path_end_point_theta = path_end_point_theta;
end
guidata(hObject, handles);
% Hints: get(hObject,'String') returns contents of path_end_point_theta_input as text
%        str2double(get(hObject,'String')) returns contents of path_end_point_theta_input as a double

% --- Executes during object creation, after setting all properties.
function path_end_point_theta_input_CreateFcn(hObject, eventdata, handles)
% hObject    handle to path_end_point_theta_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function path_end_point_Y_input_Callback(hObject, eventdata, handles)
% hObject    handle to path_end_point_Y_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(get(hObject,'String'))
    path_end_point_Y = str2double(get(hObject,'String'));
    handles.path_end_point_Y = path_end_point_Y;
end
guidata(hObject, handles);
% Hints: get(hObject,'String') returns contents of path_end_point_Y_input as text
%        str2double(get(hObject,'String')) returns contents of path_end_point_Y_input as a double

% --- Executes during object creation, after setting all properties.
function path_end_point_Y_input_CreateFcn(hObject, eventdata, handles)
% hObject    handle to path_end_point_Y_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function path_end_point_phi_input_Callback(hObject, eventdata, handles)
% hObject    handle to path_end_point_phi_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(get(hObject,'String'))
    path_end_point_phi = str2double(get(hObject,'String'));
    handles.path_end_point_phi = path_end_point_phi;
end
guidata(hObject, handles);
% Hints: get(hObject,'String') returns contents of path_end_point_phi_input as text
%        str2double(get(hObject,'String')) returns contents of path_end_point_phi_input as a double

% --- Executes during object creation, after setting all properties.
function path_end_point_phi_input_CreateFcn(hObject, eventdata, handles)
% hObject    handle to path_end_point_phi_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function path_end_point_X_input_Callback(hObject, eventdata, handles)
% hObject    handle to path_end_point_X_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(get(hObject,'String'))
    path_end_point_X = str2double(get(hObject,'String'));
    handles.path_end_point_X = path_end_point_X;
end
guidata(hObject, handles);
% Hints: get(hObject,'String') returns contents of path_end_point_X_input as text
%        str2double(get(hObject,'String')) returns contents of path_end_point_X_input as a double

% --- Executes during object creation, after setting all properties.
function path_end_point_X_input_CreateFcn(hObject, eventdata, handles)
% hObject    handle to path_end_point_X_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function path_origin_point_X_input_Callback(hObject, eventdata, handles)
% hObject    handle to path_origin_point_X_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(get(hObject,'String'))
    path_origin_point_X = str2double(get(hObject,'String'));
    handles.path_origin_point_X = path_origin_point_X;
end
guidata(hObject, handles);
% Hints: get(hObject,'String') returns contents of path_origin_point_X_input as text
%        str2double(get(hObject,'String')) returns contents of path_origin_point_X_input as a double

% --- Executes during object creation, after setting all properties.
function path_origin_point_X_input_CreateFcn(hObject, eventdata, handles)
% hObject    handle to path_origin_point_X_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function path_origin_point_phi_input_Callback(hObject, eventdata, handles)
% hObject    handle to path_origin_point_phi_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(get(hObject,'String'))
    path_origin_point_phi = str2double(get(hObject,'String'));
    handles.path_origin_point_phi = path_origin_point_phi;
end
guidata(hObject, handles);
% Hints: get(hObject,'String') returns contents of path_origin_point_phi_input as text
%        str2double(get(hObject,'String')) returns contents of path_origin_point_phi_input as a double

% --- Executes during object creation, after setting all properties.
function path_origin_point_phi_input_CreateFcn(hObject, eventdata, handles)
% hObject    handle to path_origin_point_phi_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function path_origin_point_Y_input_Callback(hObject, eventdata, handles)
% hObject    handle to path_origin_point_Y_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(get(hObject,'String'))
    path_origin_point_Y = str2double(get(hObject,'String'));
    handles.path_origin_point_Y = path_origin_point_Y;
end
guidata(hObject, handles);
% Hints: get(hObject,'String') returns contents of path_origin_point_Y_input as text
%        str2double(get(hObject,'String')) returns contents of path_origin_point_Y_input as a double

% --- Executes during object creation, after setting all properties.
function path_origin_point_Y_input_CreateFcn(hObject, eventdata, handles)
% hObject    handle to path_origin_point_Y_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function path_origin_point_theta_input_Callback(hObject, eventdata, handles)
% hObject    handle to path_origin_point_theta_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(get(hObject,'String'))
    path_origin_point_theta = str2double(get(hObject,'String'));
    handles.path_origin_point_theta = path_origin_point_theta;
end
guidata(hObject, handles);
% Hints: get(hObject,'String') returns contents of path_origin_point_theta_input as text
%        str2double(get(hObject,'String')) returns contents of path_origin_point_theta_input as a double

% --- Executes during object creation, after setting all properties.
function path_origin_point_theta_input_CreateFcn(hObject, eventdata, handles)
% hObject    handle to path_origin_point_theta_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function path_origin_point_Z_input_Callback(hObject, eventdata, handles)
% hObject    handle to path_origin_point_Z_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(get(hObject,'String'))
    path_origin_point_Z = str2double(get(hObject,'String'));
    handles.path_origin_point_Z = path_origin_point_Z;
end
guidata(hObject, handles);
% Hints: get(hObject,'String') returns contents of path_origin_point_Z_input as text
%        str2double(get(hObject,'String')) returns contents of path_origin_point_Z_input as a double

% --- Executes during object creation, after setting all properties.
function path_origin_point_Z_input_CreateFcn(hObject, eventdata, handles)
% hObject    handle to path_origin_point_Z_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function path_origin_point_psi_input_Callback(hObject, eventdata, handles)
% hObject    handle to path_origin_point_psi_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(get(hObject,'String'))
    path_origin_point_psi = str2double(get(hObject,'String'));
    handles.path_origin_point_psi = path_origin_point_psi;
end
guidata(hObject, handles);
% Hints: get(hObject,'String') returns contents of path_origin_point_psi_input as text
%        str2double(get(hObject,'String')) returns contents of path_origin_point_psi_input as a double

% --- Executes during object creation, after setting all properties.
function path_origin_point_psi_input_CreateFcn(hObject, eventdata, handles)
% hObject    handle to path_origin_point_psi_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --------------------------------------------------------------------
function Untitled_1_Callback(hObject, ~, handles)
% hObject    handle to Untitled_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

function obstacle_position_X_input_Callback(hObject, eventdata, handles)
% hObject    handle to obstacle_position_X_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(get(hObject,'String'))
    obstacle_position_X = str2double(get(hObject,'String'));
    handles.obstacle_position_X = obstacle_position_X;
end
guidata(hObject, handles);
% Hints: get(hObject,'String') returns contents of obstacle_position_X_input as text
%        str2double(get(hObject,'String')) returns contents of obstacle_position_X_input as a double

function obstacle_position_Y_input_Callback(hObject, eventdata, handles)
% hObject    handle to obstacle_position_Y_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(get(hObject,'String'))
    obstacle_position_Y = str2double(get(hObject,'String'));
    handles.obstacle_position_Y = obstacle_position_Y;
end
guidata(hObject, handles);
% Hints: get(hObject,'String') returns contents of obstacle_position_Y_input as text
%        str2double(get(hObject,'String')) returns contents of obstacle_position_Y_input as a double

function obstacle_position_Z_input_Callback(hObject, ~, handles)
% hObject    handle to obstacle_position_Z_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(get(hObject,'String'))
    obstacle_position_Z = str2double(get(hObject,'String'));
    handles.obstacle_position_Z = obstacle_position_Z;
end
guidata(hObject, handles);
% Hints: get(hObject,'String') returns contents of obstacle_position_Z_input as text
%        str2double(get(hObject,'String')) returns contents of obstacle_position_Z_input as a double

function obstacle_position_R_input_Callback(hObject, eventdata, handles)
% hObject    handle to obstacle_position_R_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(get(hObject,'String'))
    obstacle_position_R = str2double(get(hObject,'String'));
    handles.obstacle_position_R = obstacle_position_R;
end
guidata(hObject, handles);
% Hints: get(hObject,'String') returns contents of obstacle_position_R_input as text
%        str2double(get(hObject,'String')) returns contents of obstacle_position_R_input as a double

% --- Executes on button press in save_joint_angle_data.
function save_joint_angle_data_Callback(hObject, ~, ~)
% hObject    handle to save_joint_angle_data (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
load('E:\\GPM\\temporarydata.mat','q_plot1');
[filename ,pathname]=uiputfile({'*.mat','MAT-files(*.mat)'},'±£´æ');%pathname-save path£¬filename-save file name
str=strcat(pathname,filename);%string
save(char(str), 'q_plot1');%save data a as .mat£»
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in exit.
function exit_Callback(hObject, ~, ~)
% hObject    handle to exit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

button=questdlg('Are you sure to close','Close','Yes','No','Yes');
if strcmp(button,'Yes')    
    close(gcf)
    delete(hObject);
else
    return;
end

% --- Executes on button press in replay.
function replay_Callback(hObject, eventdata, handles)
% hObject    handle to replay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB

syms q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 real
deg = pi/180;
%% Create arm
L(1) = Link([pi/2 0 0 pi/2 1]); L(1).qlim = [0,30];
L(2) = Link([pi/2 0 0 pi/2 1]); L(2).qlim = [0,30];
L(3) = Link([pi/2 0 0 pi/2 1]); L(3).qlim = [0,30];
L(4)= Revolute('d', 3.15, 'a', 0, 'alpha', -pi/2 );
L(5) = Revolute('d', 0, 'a', 0, 'alpha', pi/2);
L(6) =  Revolute('d', 4.5, 'a', 0, 'alpha', -pi/2 );
L(7) =  Revolute('d', 0, 'a', 0, 'alpha', pi/2);
L(8) =  Revolute('d', 5, 'a', 0, 'alpha', -pi/2 );
L(9) =  Revolute('d', 0, 'a', 0, 'alpha', pi/2 );    
L(10) =  Revolute('d', 0.8, 'a', 0, 'alpha', 0);
PR10 = SerialLink(L,'name','Protal3-PA10');
load('E:\\GPM\\temporarydata.mat'); %load ready-made data
for i=1:50 % plot
PR10.plot(q_plot1(i,:),'workspace',[-10 20 -20 20 -5 30],'scale',0.5);
        q=q_plot1(i,:);
        q1_data = q(1);
        q2_data = q(2);
        q3_data = q(3);
        q4_data = q(4);
        q5_data = q(5);
        q6_data = q(6);
        q7_data = q(7);
        q8_data = q(8);
        q9_data = q(9);
        q10_data = q(10);
%       display
        set(handles.q1_data_output,'string',num2str(q1_data));
        set(handles.q2_data_output,'string',num2str(q2_data));
        set(handles.q3_data_output,'string',num2str(q3_data));
        set(handles.q4_data_output,'string',num2str(q4_data));
        set(handles.q5_data_output,'string',num2str(q5_data));
        set(handles.q6_data_output,'string',num2str(q6_data));
        set(handles.q7_data_output,'string',num2str(q7_data));
        set(handles.q8_data_output,'string',num2str(q8_data));
        set(handles.q9_data_output,'string',num2str(q9_data));
        set(handles.q10_data_output,'string',num2str(q10_data));
end
% handles    structure with handles and user data (see GUIDATA)

function save_in_input_Callback(hObject, ~, handles)
% hObject    handle to save_in_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(get(hObject,'String'))
    save_in = str2str(get(hObject,'String'));
    handles.save_in = save_in;
end
guidata(hObject, handles);
% Hints: get(hObject,'String') returns contents of save_in_input as text
%        str2double(get(hObject,'String')) returns contents of save_in_input as a double

% --- Executes during object creation, after setting all properties.
function save_in_input_CreateFcn(hObject, ~, handles)
% hObject    handle to save_in_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
h=errordlg('Exceed the boundary, Calculation process stopped','error');
ha=get(h,'children');
hu=findall(allchild(h),'style','pushbutton');
set(hu,'string','back');
ht=findall(ha,'type','text');

% --- Executes during object creation, after setting all properties.
function obstacle_position_X_input_CreateFcn(hObject, eventdata, handles)
% hObject    handle to obstacle_position_X_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

        %set(handles.obstacle_position_X_input,'Enable','Off')
        %set(handles.chkCrackers,'Value',0)
        %set(handles.chkRelish,'Enable','On')

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% --- Executes on button press in teach.
function teach_Callback(hObject, eventdata, handles)
% hObject    handle to teach (see GCBO)
axes(handles.axes1);
syms q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 real
deg = pi/180;
%% Create Symbolic arm
L(1) = Link([pi/2 0 0 pi/2 1]); L(1).qlim = [0,30];
L(2) = Link([pi/2 0 0 pi/2 1]); L(2).qlim = [0,30];
L(3) = Link([pi/2 0 0 pi/2 1]);L(3).qlim = [0,30];
L(4)= Revolute('d', 3.15, 'a', 0, 'alpha', -pi/2 , ...
    'qlim', [-160 160]*deg);
L(5) = Revolute('d', 0, 'a', 0, 'alpha', pi/2, ...
    'qlim', [-160 160]*deg);
L(6) =  Revolute('d', 4.5, 'a', 0, 'alpha', -pi/2 , ...
    'qlim', [-160 160]*deg);
L(7) =  Revolute('d', 0, 'a', 0, 'alpha', pi/2, ...
    'qlim', [-160 160]*deg);
L(8) =  Revolute('d', 5, 'a', 0, 'alpha', -pi/2 , ...
    'qlim', [-160 160]*deg);
L(9) =  Revolute('d', 0, 'a', 0, 'alpha', pi/2 , ...
    'qlim', [-160 160]*deg);    
L(10) =  Revolute('d', 0.8, 'a', 0, 'alpha', 0, ...
    'qlim', [-160 160]*deg);
PR10 = SerialLink(L,'name','Protal3-PA10');
q = [q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 ];
load('E:\\GPM\\q_ie.mat','q_ie');
q_r = q_ie(2,:); % ready pose, arm up
q_n = q_ie(1,:);
PR10.plot(q_r,'workspace',[-10 20 -20 30 -5 30],'jointdiam', 2,'scale',0.2);
hold on
FT0 = PR10.fkine(q_n);
FT1 = PR10.fkine(q_r);
n = 50;
traj_1 = ctraj(FT0,FT1,n);
%%
for i = 1:n;
xtraj(i) = traj_1(1,4,i);
ytraj(i) = traj_1(2,4,i);
ztraj(i) = traj_1(3,4,i);
end
plot3(xtraj,ytraj,ztraj)
hold on;
obs=[handles.obstacle_position_X,handles.obstacle_position_Y,handles.obstacle_position_Z];
d0 = handles.obstacle_position_R;
x0 = obs(1);
y0 = obs(2);
z0 = obs(3);
[x,y,z]=sphere;
mesh(d0*x+x0,d0*y+y0,d0*z+z0)
hold all;
teach(PR10);
hold on;
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function q2_data_output_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q2_data_output (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function q1_data_output_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q1_data_output (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function q3_data_output_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q3_data_output (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function q4_data_output_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q4_data_output (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function q5_data_output_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q5_data_output (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function q6_data_output_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q6_data_output (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function q7_data_output_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q7_data_output (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function q8_data_output_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q8_data_output (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function q9_data_output_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q9_data_output (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function q10_data_output_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q10_data_output (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function closest_Link_CreateFcn(hObject, eventdata, handles)
% hObject    handle to closest_Link (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function closest_distance_CreateFcn(hObject, eventdata, handles)
% hObject    handle to closest_distance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function select_C_CreateFcn(hObject, eventdata, handles)
% hObject    handle to select_C (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% --- Executes during object creation, after setting all properties.
function select_C_o_CreateFcn(hObject, eventdata, handles)
% hObject    handle to select_C (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to select_C (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to select_C (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% --- Executes during object creation, after setting all properties.
function obstacle_position_CreateFcn(hObject, eventdata, handles)
% hObject    handle to select_C (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% --- Executes during object creation, after setting all properties.
function obstacle_position_R_input_CreateFcn(hObject, eventdata, handles)
% hObject    handle to select_C (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% --- Executes during object creation, after setting all properties.
function text32_CreateFcn(hObject, eventdata, handles)
% hObject    handle to select_C (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% --- Executes on button press in select_C_o.
function select_C_l_Callback(hObject, eventdata, handles)
% hObject    handle to select_C_l (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
select_C_l = get(handles.select_C_l,'Value')
% Hint: get(hObject,'Value') returns toggle state of select_C_l

% --- Executes on button press in select_C_o.
function select_C_s_Callback(hObject, eventdata, handles)
% hObject    handle to select_C_s (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
select_C_s = get(handles.select_C_s,'Value')
% Hint: get(hObject,'Value') returns toggle state of select_C_s


% --- Executes during object creation, after setting all properties.
function select_M_PA10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to select_M_PA10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
