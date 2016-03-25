function varargout = EE553_term_pr_Omer_Cayir(varargin)
% EE553_TERM_PR_OMER_CAYIR MATLAB code for EE553_term_pr_Omer_Cayir.fig
%      EE553_TERM_PR_OMER_CAYIR, by itself, creates a new EE553_TERM_PR_OMER_CAYIR or raises the existing
%      singleton*.
%
%      H = EE553_TERM_PR_OMER_CAYIR returns the handle to a new EE553_TERM_PR_OMER_CAYIR or the handle to
%      the existing singleton*.
%
%      EE553_TERM_PR_OMER_CAYIR('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in EE553_TERM_PR_OMER_CAYIR.M with the given input arguments.
%
%      EE553_TERM_PR_OMER_CAYIR('Property','Value',...) creates a new EE553_TERM_PR_OMER_CAYIR or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before EE553_term_pr_Omer_Cayir_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to EE553_term_pr_Omer_Cayir_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help EE553_term_pr_Omer_Cayir

% Last Modified by GUIDE v2.5 24-Mar-2016 18:20:54

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @EE553_term_pr_Omer_Cayir_OpeningFcn, ...
                   'gui_OutputFcn',  @EE553_term_pr_Omer_Cayir_OutputFcn, ...
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


% --- Executes just before EE553_term_pr_Omer_Cayir is made visible.
function EE553_term_pr_Omer_Cayir_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to EE553_term_pr_Omer_Cayir (see VARARGIN)

% Choose default command line output for EE553_term_pr_Omer_Cayir
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes EE553_term_pr_Omer_Cayir wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = EE553_term_pr_Omer_Cayir_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
% initialization of parameters
handles.cases = 9; % choose Case 5 with H1
handles.ttc = 1e-3; % threshold value to check whether the norm is too small
handles.noi = 150; % the maximum number of iteration for gradient projection
handles.grad = 1; % commpute gradient analytically (2 for numerically)
handles.noiods = 50; % the maximum number of iteration for one-dim search
handles.finr = 1e-5; % the minimum of IOU for one-dim search
handles.ODS = 2; % one-dim search method (1:Dichotomoous, 2:Fibonacci)
handles.mseuni = 10; % maximum of MSE for uniform power allocation
handles.trg = [0;0]; % 2x1 vector for the position of target in meters
axes(handles.axes1);
    inputImageArray = imread('input.png');
    imshow(inputImageArray);
axes(handles.axes2);
    inputImageArray = imread('output.png');
    imshow(inputImageArray);
guidata(hObject, handles);

% --- Executes on button press in viewlayout_pb.
function viewlayout_pb_Callback(hObject, eventdata, handles)
% hObject    handle to viewlayout_pb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clc
global M;
CSang = [1 2 3 4 1 2 3 4 1 2 3 4];
CSrng = [1 1 1 1 1 1 1 1 2 2 2 2];
CSrcs = [2 2 2 2 3 3 3 3 1 1 1 1];
axes(handles.axes1);
[M,N,tx,ty,rx,ry] = viewlayout(CSang(handles.cases),CSrng(handles.cases),handles.trg);
H = getRCS(CSrcs(handles.cases),M,N);
computeAebe(M,N,tx,ty,rx,ry,H,handles.trg);

guidata(hObject, handles);

% --- Executes on button press in start_pb.
function start_pb_Callback(hObject, eventdata, handles)
% hObject    handle to start_pb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clc
viewlayout_pb_Callback(hObject, eventdata, handles)
global Ae be M;
nmax=handles.mseuni;
Uv = ones(M,1);
Pu = (Uv'*be/(Uv'*Ae*Uv)/nmax)*Uv;
% sum(Pu);
sgmu = objfun(Pu);
maxiter = handles.noi;
ttc = handles.ttc;
grad = handles.grad;
odsmaxiter = handles.noiods;
odsfinrange = handles.finr;
if get(handles.dichotomous_rb,'Value')
	handles.ODS=1;
elseif get(handles.fibonacci_rb,'Value')
    handles.ODS=2;
end
ODS = handles.ODS;
[Popt, MSE] = gradientprojection(Pu,maxiter,ttc,grad,ODS,odsmaxiter,odsfinrange);
% function [x, MSE] = gradientprojection(x,maxiter,ttc,grad,ODS,odsmaxiter,odsfinrange)
% Inputs:
% x: initial point
% maxiter: the maximum number of iteration for gradient projection
% ttc: threshold value to check whether the norm is too small
% grad: gradient computation is (1:analytically, 2:numerically)
% ODS: one-dim search method (1:Dichotomoous, 2:Fibonacci)
% odsmaxiter: the maximum number of iteration for one-dim search
% odsfinrange: the minimum of IOU for one-dim search
sgmopt = objfun(Popt);
Rs=[Pu' sum(Pu) sgmu;Popt' sum(Popt) sgmopt];
table_as_cell = num2cell(Rs);
set(handles.results_uitable, 'Data', table_as_cell);

axes(handles.axes2);
k = find(MSE==0);
if isempty(k)
    plot(1:numel(MSE),MSE)
    title('MSE vs. Iteration')
    ylabel('MSE, m^2')
    xlabel('Iteration number')
    helpdlg('You may increase the maximum number of iterations to get lower MSE',...
            '');
else
    plot(1:(k-1),MSE(1:(k-1)))
    title('MSE vs. Iteration')
    ylabel('MSE, m^2')
    xlabel('Iteration number')
    msgbox('Optimization is successfully completed','Success!');
end

guidata(hObject, handles);


function mseuni_edit_Callback(hObject, eventdata, handles)
% hObject    handle to mseuni_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of mseuni_edit as text
%        str2double(get(hObject,'String')) returns contents of mseuni_edit as a double
val = str2double(get(hObject,'String'));
% Determine whether val is a number between 0 and integer.
if isnumeric(val) && length(val)==1 && val >= 1 && val <= 100
    if handles.mseuni~=val
        handles.mseuni = val;
    end
else
	set(hObject,'String', num2str(handles.mseuni));
% Restore focus to the edit text box after error
	uicontrol(hObject)
%     handles.UC = str2double(get(hObject,'String'));
end
% Update handles structure
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function mseuni_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mseuni_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function noi_edit_Callback(hObject, eventdata, handles)
% hObject    handle to noi_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of noi_edit as text
%        str2double(get(hObject,'String')) returns contents of noi_edit as a double
val = str2double(get(hObject,'String'));
% Determine whether val is a number between 0 and integer.
if isnumeric(val) && length(val)==1 && val >= 1 && val <= 1000
    if handles.noi~=round(val)
        handles.noi = round(val);
    end
else
	set(hObject,'String', num2str(handles.noi));
% Restore focus to the edit text box after error
	uicontrol(hObject)
%     handles.UC = str2double(get(hObject,'String'));
end
% Update handles structure
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function noi_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to noi_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function trgx_edit_Callback(hObject, eventdata, handles)
% hObject    handle to trgx_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of trgx_edit as text
%        str2double(get(hObject,'String')) returns contents of trgx_edit as a double
val = str2double(get(hObject,'String'));
% Determine whether val is a number between 0 and integer.
if isnumeric(val) && length(val)==1 && val >= -1e+3 && val <= 1e+3
    if handles.trg(1)~=round(val)
        handles.trg(1) = round(val);
    end
else
	set(hObject,'String', num2str(handles.trg(1)));
% Restore focus to the edit text box after error
	uicontrol(hObject)
%     handles.UC = str2double(get(hObject,'String'));
end
% Update handles structure
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function trgx_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to trgx_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function trgy_edit_Callback(hObject, eventdata, handles)
% hObject    handle to trgy_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of trgy_edit as text
%        str2double(get(hObject,'String')) returns contents of trgy_edit as a double
val = str2double(get(hObject,'String'));
% Determine whether val is a number between 0 and integer.
if isnumeric(val) && length(val)==1 && val >= -1e+3 && val <= 1e+3
    if handles.trg(2)~=round(val)
        handles.trg(2) = round(val);
    end
else
	set(hObject,'String', num2str(handles.trg(2)));
% Restore focus to the edit text box after error
	uicontrol(hObject)
%     handles.UC = str2double(get(hObject,'String'));
end
% Update handles structure
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function trgy_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to trgy_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ttc_edit_Callback(hObject, eventdata, handles)
% hObject    handle to ttc_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ttc_edit as text
%        str2double(get(hObject,'String')) returns contents of ttc_edit as a double
val = str2double(get(hObject,'String'));
% Determine whether val is a number between 0 and integer.
if isnumeric(val) && length(val)==1 && val >= 1e-5 && val <= 1e-1
    if handles.ttc~=val
        handles.ttc = val;
    end
else
	set(hObject,'String', num2str(handles.ttc));
% Restore focus to the edit text box after error
	uicontrol(hObject)
%     handles.UC = str2double(get(hObject,'String'));
end
% Update handles structure
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function ttc_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ttc_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function finr_edit_Callback(hObject, eventdata, handles)
% hObject    handle to finr_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of finr_edit as text
%        str2double(get(hObject,'String')) returns contents of finr_edit as a double
val = str2double(get(hObject,'String'));
% Determine whether val is a number between 0 and integer.
if isnumeric(val) && length(val)==1 && val >= 1e-10 && val <= 1e-1
    if handles.finr~=val
        handles.finr = val;
    end
else
	set(hObject,'String', num2str(handles.finr));
% Restore focus to the edit text box after error
	uicontrol(hObject)
%     handles.UC = str2double(get(hObject,'String'));
end
% Update handles structure
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function finr_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to finr_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in cases_popupmenu.
function cases_popupmenu_Callback(hObject, eventdata, handles)
% hObject    handle to cases_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns cases_popupmenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from cases_popupmenu
% val = get(hObject,'Value')
% str = get(hObject,'String');
% CSstr = {'Case 1 with H2','Case 2 with H2','Case 3 with H2','Case 4 with H2', ...
%     'Case 1 with H3','Case 2 with H3','Case 3 with H3','Case 4 with H3',...
%     'Case 5 with H1','Case 6 with H1','Case 7 with H1','Case 8 with H1'};
% CSstr = strcmp(CSstr,str);
handles.cases = get(hObject,'Value');
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function cases_popupmenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cases_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in dichotomous_rb.
function dichotomous_rb_Callback(hObject, eventdata, handles)
% hObject    handle to dichotomous_rb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of dichotomous_rb


% --- Executes on button press in fibonacci_rb.
function fibonacci_rb_Callback(hObject, eventdata, handles)
% hObject    handle to fibonacci_rb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of fibonacci_rb



function noi_ods_edit_Callback(hObject, eventdata, handles)
% hObject    handle to noi_ods_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of noi_ods_edit as text
%        str2double(get(hObject,'String')) returns contents of noi_ods_edit as a double
val = str2double(get(hObject,'String'));
% Determine whether val is a number between 0 and integer.
if isnumeric(val) && length(val)==1 && val >= 1 && val <= 100
    if handles.noiods~=round(val)
        handles.noiods = round(val);
    end
else
	set(hObject,'String', num2str(handles.noiods));
% Restore focus to the edit text box after error
	uicontrol(hObject)
%     handles.UC = str2double(get(hObject,'String'));
end
% Update handles structure
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function noi_ods_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to noi_ods_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in grad_popupmenu.
function grad_popupmenu_Callback(hObject, eventdata, handles)
% hObject    handle to grad_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns grad_popupmenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from grad_popupmenu
handles.grad = get(hObject,'Value');
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function grad_popupmenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to grad_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in about_pushbutton.
function about_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to about_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
msgbox(['Gradient Projection Approach for Power Allocation Strategies '...
    'for Target Localization in Distributed Multiple-Radar Architectures,' ...
    ' Omer CAYIR, EE553 Term Project January 2015'],'About');
