function varargout = Esercitazione1_gui(varargin)
% ESERCITAZIONE1_GUI MATLAB code for Esercitazione1_gui.fig
%      ESERCITAZIONE1_GUI, by itself, creates a new ESERCITAZIONE1_GUI or raises the existing
%      singleton*.
%       
%      H = ESERCITAZIONE1_GUI returns the handle to a new ESERCITAZIONE1_GUI or the handle to
%      the existing singleton*.
%   
%      ESERCITAZIONE1_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ESERCITAZIONE1_GUI.M with the given input arguments.
%
%      ESERCITAZIONE1_GUI('Property','Value',...) creates a new ESERCITAZIONE1_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Esercitazione1_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      off.  All inputs are passed to Esercitazione1_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Esercitazione1_gui

% Last Modified by GUIDE v2.5 19-May-2023 12:30:56

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @Esercitazione1_gui_OpeningFcn, ...
    'gui_OutputFcn',  @Esercitazione1_gui_OutputFcn, ...
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


% --- Executes just before Esercitazione1_gui is made visible.
function Esercitazione1_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Esercitazione1_gui (see VARARGIN)

% Choose default command line output for Esercitazione1_gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Esercitazione1_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Esercitazione1_gui_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function image_filename_Callback(hObject, eventdata, handles)
% hObject    handle to image_filename (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of image_filename as text
%        str2double(get(hObject,'String')) returns contents of image_filename as a double


% --- Executes during object creation, after setting all properties.
function image_filename_CreateFcn(hObject, eventdata, handles)
% hObject    handle to image_filename (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in ON.
function ON_Callback(hObject, eventdata, handles)
% hObject    handle to ON (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global KeepRunning;
KeepRunning = 1;

% Variabile Booleana per gestire la procedura di labelling
global LABELLING;
LABELLING = 1;  % Diventa 0 appena premo sul pulsante LABELLING

% Contatore per identificare la traccia corrente da labellare
global to_label;
to_label = 0;   % Ad ogni marker questo contatore si incrementa

% Inizializzo le posizioni dei marker di interesse come variabili globali
global mcp;
global pip;
global dip;
global tip;
global base;
% Definisco l'oggetto vid per gestire la comunicazione con la webcam
handles.vid = videoinput('winvideo', 1);
handles.vid.FramesPerTrigger = 1;
handles.vid.TriggerRepeat = Inf;

% Riempo l'immagine nell GUI
axes(handles.webcam_image)

% Modifico il colore del pulsante "Label" con verde una volta che effettuo
% la connessione
handles.label.BackgroundColor = 'g';

% Inizializzo le tracks
global tracks;
tracks = initializeTracks(); % Create an empty array of tracks.
nextId = "1"; % ID of the next track
centroids = [];
bboxes = [];

% Definisco il vettore angles che riempiremo con gli angoli di giunto
% misurati
angles0 = [];
angles1 = [];
angles2 = [];

% Inizializzo la comunicazione con la webcam
start(handles.vid)

i=0;
j=0;
k=0;
% definizioni robot
a1 = 0.052;
a2 = 0.055;
a3 = 0.025;
a4 = 0.019;

d1=0;
d2=0;
d3=0;
d4=0;


a = [a1, a2, a3,a4];
d = [d1, d2, d3,d4];
angle=[0,0,0,0];
lmax = sum(a);
u = 0;
while(1)
    % Estrapolo l'ultimo frame acquisito
    if handles.enable_antropometric.Value == 0
        handles.image = getdata(handles.vid, 1);
        flushdata(handles.vid);
        %handles.image=imread("CAL.png");
        j=0;
        k=0;
    %disp(newtracks);
    else
        if j<1
            handles.image=imread("CAL.png");
            load("CALIBRAZIONE.mat")
            [handles.image, newOrigin] = undistortImage(handles.image, cameraParams, OutputView = "same");
            magnification = 25;
            j=j+1;
        end
        flushdata(handles.vid);
    end
    if handles.enable_segmentation.Value == 1
        % allora faccio tresholding
        
        if handles.colorspace.Value == 1
            % allora RGB
            thr1 = handles.channel1.Value*255;
            thr2 = handles.channel2.Value*255;
            thr3 = handles.channel3.Value*255;
            
            % calcolo la maschera binaria del ROSSO
            MASK_r = handles.image(:,:,1) > thr1;
            MASK_g = handles.image(:,:,2) > thr2;
            MASK_b = handles.image(:,:,3) > thr3;
            
            MASK = uint8(MASK_r & MASK_g & MASK_b);
            
        elseif handles.colorspace.Value == 2
            % allora HSV
            % converto in HSV
            image = rgb2hsv(handles.image);
            
            thr1 = handles.channel1.Value;
            thr2 = handles.channel2.Value;
            thr3 = handles.channel3.Value;
            
            % calcolo la maschera binaria del Hue
            MASK_h = image(:,:,1) > thr1;
            MASK_s = image(:,:,2) > thr2;
            MASK_v = image(:,:,3) > thr3;
            
            MASK = uint8(MASK_h & MASK_s & MASK_v);
            
        else
            % YCbCr
            % converto in YCbCr
            image = rgb2ycbcr(handles.image);
            
            thr1 = handles.channel1.Value;
            thr2 = handles.channel2.Value;
            thr3 = handles.channel3.Value;
            %disp(thr3)
            % calcolo la maschera binaria del Hue
            MASK_y  = image(:,:,1) > thr1*255;
            MASK_cb = image(:,:,2) > thr2*255;
            MASK_cr = image(:,:,3) > thr3*255;
            
            MASK = uint8(MASK_y & MASK_cb & MASK_cr);
        end
        
        % A questo punto ho nell'oggetto MASK la maschera binaria
        if handles.enable_detection.Value == 1
            
            % devo individuare gli oggetti nella scena
            [MASK, centroids, bboxes] = ObjectDetection(MASK);
            if handles.enable_antropometric.Value == 1
                if k<1
                %%%%%%% prendiamo le bboxes a 2 a 2 e facciamo la
                %%%%%%% differenza
                
                %%
                % Find connected components.
                blobAnalysis = vision.BlobAnalysis(AreaOutputPort = true,...
                    CentroidOutputPort = false,...
                    BoundingBoxOutputPort = true,...
                    MinimumBlobArea = 100, ExcludeBorderBlobs = true);
                [areas, boxes] = step(blobAnalysis, MASK);

                % Sort connected components in descending order by area
                [~, idx] = sort(boxes(:,1));
                %disp(idx)
                % Get the five largest components.
                %disp(boxes)
                boxes = double(boxes(idx(1:5), :));
                disp(boxes)
                i=i+1;

                size(boxes)
                % Reduce the size of the image for display.
                scale = magnification / 100;
                imDetectedMarker = imresize(handles.image, scale);

                % Insert labels for the markers.
                axes(handles.webcam_image)
                imshow(handles.image)

                hold on
                %rectangle('Position',boxes(1,:),'EdgeColor','r','LineWidth',2);
                handles.image = insertObjectAnnotation(handles.image, "rectangle", ...
                    boxes(1,:), "BASE","FontSize",20);
                handles.image = insertObjectAnnotation(handles.image, "rectangle", ...
                    boxes(2,:), "MCP","FontSize",20);
                handles.image = insertObjectAnnotation(handles.image, "rectangle", ...
                    boxes(3,:), "PIP","FontSize",20);
                handles.image = insertObjectAnnotation(handles.image, "rectangle", ...
                    boxes(4,:), "DIP","FontSize",20);
                handles.image = insertObjectAnnotation(handles.image, "rectangle", ...
                    boxes(5,:), "TIP","FontSize",20);
                hold on
                %rectangle('Position',boxes(1,:),'EdgeColor','r','LineWidth',2);
                %MASK = MASK.*0;

                %%
                % Detect the checkerboard.
                [imagePoints, boardSize] = detectCheckerboardPoints(handles.image);

                % Adjust the imagePoints so that they are expressed in the coordinate system
                % used in the original image, before it was undistorted.  This adjustment
                % makes it compatible with the cameraParameters object computed for the original image.
                imagePoints = imagePoints + newOrigin; % adds newOrigin to every row of imagePoints

                % Extract camera intrinsics.
                camIntrinsics = cameraParams.Intrinsics;

                % Compute extrinsic parameters of the camera.
                camExtrinsics = estimateExtrinsics(imagePoints, worldPoints, camIntrinsics);
                %%
                

                % BASE Get the top-left and the top-right corners.
                box1 = double(boxes(1, :));
                imagePoints1 = [box1(1:2); ...
                    box1(1) + box1(3), box1(2)];

                % Get the world coordinates of the corners
                worldPoints1 = img2world2d(imagePoints1, camExtrinsics, camIntrinsics);
                a=worldPoints1;
                % Compute the diameter of the coin in millimeters.
%                 d1 = worldPoints1(2, :) - worldPoints1(1, :);
%                 diameterInMillimeters = hypot(d1(1), d1(2));
%                 fprintf("Measured diameter of first marker (MCP) = %0.2f mm\n", diameterInMillimeters);

                % MCP Get the top-left and the top-right corners.
                box2 = double(boxes(2, :));
                imagePoints2 = [box2(1:2); ...
                    box2(1) + box2(3), box2(2)];

                % Apply the inverse transformation from image to world
                worldPoints2 = img2world2d(imagePoints2, camExtrinsics, camIntrinsics);
                b=worldPoints2;
                % Compute the diameter of the coin in millimeters.
%                 d2 = worldPoints2(2, :) - worldPoints2(1, :);
%                 diameterInMillimeters = hypot(d2(1), d2(2));
%                 fprintf("Measured diameter of the second marker (PIP)= %0.2f mm\n", diameterInMillimeters);

                % PIP Get the top-left and the top-right corners.
                box3 = double(boxes(3, :));
                imagePoints3 = [box3(1:2); ...
                    box3(1) + box3(3), box3(2)];

                % Apply the inverse transformation from image to world
                worldPoints3 = img2world2d(imagePoints3, camExtrinsics, camIntrinsics);
                c=worldPoints3;
                % Compute the diameter of the coin in millimeters.
%                 d3 = worldPoints3(2, :) - worldPoints3(1, :);
%                 diameterInMillimeters = hypot(d3(1), d3(2));
%                 fprintf("Measured diameter of the third marker (DIP)= %0.2f mm\n", diameterInMillimeters);

                % DIP Get the top-left and the top-right corners.
                box4 = double(boxes(4, :));
                imagePoints4 = [box4(1:2); ...
                    box4(1) + box4(3), box4(2)];

                % Apply the inverse transformation from image to world
                worldPoints4 = img2world2d(imagePoints4, camExtrinsics, camIntrinsics);
                d=worldPoints4;
                % Compute the diameter of the coin in millimeters.
%                 d4 = worldPoints4(2, :) - worldPoints4(1, :);
%                 diameterInMillimeters = hypot(d4(1), d4(2));
%                 fprintf("Measured diameter of the fourth marker (TIP)= %0.2f mm\n", diameterInMillimeters);

                % TIP Get the top-left and the top-right corners.
                box5 = double(boxes(5, :));
                imagePoints5 = [box5(1:2); ...
                    box5(1) + box5(3), box5(2)];

                % Apply the inverse transformation from image to world
                worldPoints5 = img2world2d(imagePoints5, camExtrinsics, camIntrinsics);
                e=worldPoints5;
                %%
                %%%%%%%%%%%%%%%%%%%%%%%%%
                D1 = a(2,:)-b(1,:);
                diameterInMillimeters = hypot(D1(1), D1(2));
                fprintf("Measured distance BASE-MCP = %0.2f mm\n", diameterInMillimeters);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                D2 = b(2,:)-c(1,:);
                diameterInMillimeters = hypot(D2(1), D2(2));
                fprintf("Measured distance MCP-PIP = %0.2f mm\n", diameterInMillimeters);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                D3 = c(2,:)-d(1,:);
                diameterInMillimeters = hypot(D3(1), D3(2));
                fprintf("Measured distance PIP-DIP = %0.2f mm\n", diameterInMillimeters);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                D4 = d(2,:)-e(1,:);
                diameterInMillimeters = hypot(D4(1), D4(2));
                fprintf("Measured distance DIP-TIP = %0.2f mm\n", diameterInMillimeters);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                k=k+1;
                end
            end
                
            if handles.enable_tracking.Value == 1
                
                % Predict new locations
                tracks = predictNewLocationsOfTracks(tracks);
                
                % enable_detection
                [assignments, unassignedTracks, unassignedDetections] = ...
                    detectionToTrackAssignment(tracks, centroids);
                
                tracks = updateAssignedTracks(tracks, assignments, ...
                    centroids, bboxes);
                tracks = updateUnassignedTracks(tracks, unassignedTracks);
                tracks = deleteLostTracks(tracks);
                [tracks, nextId] = createNewTracks(tracks, centroids, bboxes, ...
                    unassignedDetections, nextId);
                handles.tracks = tracks;
                
                % Check per vedere se si aggiungono nuove tracce da etichettare
                for i = 1:size(tracks,2)
                    if ~isempty(str2num(tracks(i).id))
                        handles.label.Visible = 'on';
                        handles.marker_label.Visible = 'on';
                    end
                end
                
                %% Calcolo degli angoli
                
                % Si entra nell'if solo se abilito la checkbox "Angoli di giunto" e
                % se il campo id delle tracks non è vuoto
                if handles.angles_computation.Value == 1 && ~isempty([tracks(:).id])
                    
                    % Check se esistono le tracce che mi interessano per calcolare gli
                    % angoli
                    % NB: i 3 marker sono chiamati mcp, pip e dip da codice quindi
                    % quando si effettua il labelling bisogna inserire esattamente mcp,
                    % pip e dip
                    indexes = [tracks(:).id] == ["base";"mcp";"pip";"dip";"tip"];
                    
                    % La variabile check conta il numero di tracce con il nome
                    % assegnato esistenti: in questo caso, con i 3 marker mcp, pip e
                    % dip, questo valore deve essere 3.
                    check = sum(indexes,'all');
                    
                    if check == 5
                        
                        % Individuo per righe l'indice corrispondente ad ogni
                        % marker, che mi servirà per selezionare il marker dal
                        % campo id di tracks
                        base_id = find(indexes(1,:));
                        mcp_id = find(indexes(2,:));
                        pip_id = find(indexes(3,:));
                        dip_id = find(indexes(4,:));
                        tip_id = find(indexes(5,:));
                        for marker = ["base", "mcp", "pip", "dip", "tip"]
                            % Se ho la misura, plotto la misura, altrimenti plotto
                            % la predizione
                            if eval(strcat("isnan(tracks(",marker,"_id).detectedHistory(:,end))"))
                                eval(strcat(marker," = tracks(",marker,"_id).predictedHistory(:,end);"))
                            else
                                eval(strcat(marker," = tracks(",marker,"_id).detectedHistory(:,end);"))
                            end
                        end
                        
                        % Creo i vettori tra i marker
                        v0 = mcp - base;
                        v1 = pip - mcp;
                        v2 = dip - pip;
                        v3 = tip - dip;
                        % Calcolo l'angolo compreso tra i due vettori (l'output è in radianti)
%                         curr_angle0 = 
%                         curr_angle1 = acos(dot(v1,v2)/(norm(v1)*norm(v2)));
%                         curr_angle2 = acos(dot(v2,v3)/(norm(v2)*norm(v3)));
%                         curr_angle0 = 
                        cos0 = dot(v0,v1)/(norm(v0)*norm(v1));
                        cos1 = dot(v1,v2)/(norm(v1)*norm(v2));
                        cos2 = dot(v2,v3)/(norm(v2)*norm(v3));
                        sin0 = sqrt(1 - cos0^2);
                        sin1 = sqrt(1 - cos1^2);
                        sin2 = sqrt(1 - cos2^2);
                        curr_angle0 = atan2(sin0,cos0);
                        curr_angle1 = atan2(sin1,cos1);
                        curr_angle2 = atan2(sin2,cos2);
                        % Conversione in gradi
                        angles0(:,end+1) = 180-rad2deg(curr_angle0);
                        angles1(:,end+1) = 180-rad2deg(curr_angle1);
                        angles2(:,end+1) = 180-rad2deg(curr_angle2);
                        angles0_ = 180-rad2deg(curr_angle0);
                        angles1_ = 180-rad2deg(curr_angle1);
                        angles2_ = 180-rad2deg(curr_angle2);
                        % Plot angoli di giunto
                        axes(handles.angle_plot) 
                        %hold on;
%                         if length(angles0) < 100
%                             plot(angles0,'LineWidth',2)
%                         else
%                             plot(angles0(end-99:end),'LineWidth',2)
%                         end
%                         hold on
%                         if length(angles1) < 100
%                             plot(angles1,'LineWidth',2)
%                         else
%                             plot(angles1(end-99:end),'LineWidth',2)
%                         end
%                         hold on
%                         if length(angles2) < 100
%                             plot(angles2,'LineWidth',2)
%                         else
%                             plot(angles2(end-99:end),'LineWidth',2)
%                         end
%                         hold off
%                         legend("angle0","angle1","angle2");
%                         ylabel("Angolo [deg]")
%                         xlabel("Campioni")
%                         ylim([-10 200])
                            angles0_ = deg2rad(angles0_+180);
                            angles1_ = deg2rad(angles1_+180);
                            angles2_ = deg2rad(angles2_+180);
                            
                            T01 = DH_computation(d(1), a(1), angle(1), 0);
                            T12 = DH_computation(d(2), a(2), angle(2), angles0_);
                            T23 = DH_computation(d(3), a(3), angle(3), angles1_);
                            T34 = DH_computation(d(4), a(4), angle(4), angles2_);
                            
                            T02 = T01*T12;
                            T03 = T01*T12*T23;
                            T04 = T01*T12*T23*T34;
                            
                            XY1 = DirectKinematics(T01);
                            XY2 = DirectKinematics(T02);
                            XY3 = DirectKinematics(T03);
                            XYe = DirectKinematics(T04);
                            %save traiettoria tip -> riferita alla
                            %posiizone di tip che ha all'istante iniziale
                            if u == 0
                                tipXYZ = XYe
                                
                            else
                                tipXYZ(:,end+1) = XYe -tipXYZ(:,1);
                            end
                            u = u+1;
                            
                            plot([0],[0],'.k','MarkerSize',20)
                            hold on
                            plot([0 XY1(1)],[0 XY1(2)],'-r','Linewidth',4)
                            hold on
                            plot([XY1(1) XY2(1)],[XY1(2) XY2(2)],'-b','Linewidth',4)
                            hold on
                            plot([XY1(1)],[XY1(2)],'.k','MarkerSize',20)
                            hold on
                            plot([XY2(1) XY3(1)],[XY2(2) XY3(2)],'-g','Linewidth',4)
                            hold on
                            plot([XY2(1)],[XY2(2)],'.k','MarkerSize',20)
                            hold on
                            plot([XY3(1) XYe(1)],[XY3(2) XYe(2)],'-k','Linewidth',4)
                            hold on
                            plot([XY3(1)],[XY3(2)],'.k','MarkerSize',20)
                            
                            %plot(XYe(1),XYe(2),'-b','LineWidth',1.5)
                            
                            
                             xlim([-0.1 0.2])
                            ylim([-0.2 0.2])
                            
                            
                            xlabel('x [m]','Interpreter','latex')
                            ylabel('y [m]','Interpreter','latex')
                            
                            set(gca,'FontSize',16)
                            %pause(0.3);
                            hold off
                    end
                end
                
            end
            
        end
        
        displayTrackingResults(handles, MASK, tracks, centroids, bboxes)
        
    else
        % mostro l'immagine acquisita
        axes(handles.webcam_image)
        imshow(handles.image);
    end
    
    if KeepRunning == 1
    else
        save("prova.mat","angles0","angles1","angles2","tipXYZ")
        break
    end
    
    % Aggiorno i dati nella GUI
    guidata(hObject, handles)
    drawnow
    hold off
    
end

stop(handles.vid)
close(Esercitazione1_gui)

% --- Executes on button press in off.
function off_Callback(hObject, eventdata, handles)
% hObject    handle to off (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Modifico lo stato della variabile globale
global KeepRunning;
%disp(angles1);
%save("prova.mat","angles1")
KeepRunning = 0;
guidata(hObject, handles)


% --- Executes on button press in click.
function click_Callback(hObject, eventdata, handles)
% hObject    handle to click (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
filename = handles.image_filename.String;
imwrite(handles.image, strcat(filename,".png"))


% --- Executes on button press in enable_segmentation.
function enable_segmentation_Callback(hObject, eventdata, handles)
% hObject    handle to enable_segmentation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of enable_segmentation


% --- Executes on selection change in colorspace.
function colorspace_Callback(hObject, eventdata, handles)
% hObject    handle to colorspace (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns colorspace contents as cell array
%        contents{get(hObject,'Value')} returns selected item from colorspace


% --- Executes during object creation, after setting all properties.
function colorspace_CreateFcn(hObject, eventdata, handles)
% hObject    handle to colorspace (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function channel1_Callback(hObject, eventdata, handles)
% hObject    handle to channel1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function channel1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to channel1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function channel2_Callback(hObject, eventdata, handles)
% hObject    handle to channel2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function channel2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to channel2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function channel3_Callback(hObject, eventdata, handles)
% hObject    handle to channel3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function channel3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to channel3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on button press in enable_tracking.
function enable_tracking_Callback(hObject, eventdata, handles)
% hObject    handle to enable_tracking (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of enable_tracking


% --- Executes on button press in enable_detection.
function enable_detection_Callback(hObject, eventdata, handles)
% hObject    handle to enable_detection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of enable_detection

function [mask, centroids, bboxes] = ObjectDetection(mask)

%mask = medfilt2(mask,[51 51]);
%mask = imfill(mask, 'holes');
mask = bwareaopen(mask, 600);

% Calcolo delle feature geometriche
stats = regionprops(mask,'BoundingBox','Centroid');
if size(stats,1) > 0
    for i = 1:size(stats,1)
        centroids(i,:) = stats(i).Centroid;
        bboxes(i,:) = stats(i).BoundingBox;
    end
else
    centroids = [];
    bboxes = [];
end

function displayTrackingResults(handles, mask, tracks, centroids, bboxes)

global to_label;
global LABELLING;
global base;
global mcp;
global pip;
global dip;
global tip;
% Convert the mask to uint8 RGB.
image = handles.image;
mask = uint8(repmat(mask, [1, 1, 3])) .* 255;

if handles.enable_segmentation.Value == 1 && handles.enable_detection.Value == 0
    % Display the mask and the frame.
    axes(handles.webcam_image)
    imshow(image)
    axes(handles.mask_image)
    imshow(mask)
    
elseif handles.enable_detection.Value == 1 && handles.enable_tracking.Value == 0
    % Display the mask and the frame.
    axes(handles.webcam_image)
    imshow(image)
    axes(handles.mask_image)
    imshow(mask)
    hold on
    for i = 1:size(centroids,1)
        rectangle('Position',bboxes(i,:),'EdgeColor','r','LineWidth',2);
        plot(centroids(i,1), centroids(i,2),'.r','MarkerSize',20)
    end
    hold off
end

minVisibleCount = 10;

if ~isempty(tracks) && handles.enable_tracking.Value == 1
    
    % Noisy detections tend to result in short-lived tracks.
    % Only display tracks that have been visible for more than
    % a minimum number of frames.
    reliableTrackInds = ...
        [tracks(:).totalVisibleCount] > minVisibleCount;
    reliableTracks = tracks(reliableTrackInds);
    
    % Display the objects. If an object has not been detected
    % in this frame, display its predicted bounding box.
    if ~isempty(reliableTracks)
        
        % Get bounding boxes.
        bboxes = cat(1, reliableTracks.bbox);
        
        % Create labels for objects indicating the ones for
        % which we display the predicted rather than the actual
        % location.
        labels = cellstr([reliableTracks(:).id]);%int2str(ids'));
        predictedTrackInds = ...
            [reliableTracks(:).consecutiveInvisibleCount] > 0;
        isPredicted = cell(size(labels));
        isPredicted(predictedTrackInds) = {' predicted'};
        labels = strcat(labels, isPredicted);
        
        % Draw the objects on the mask.
        mask = insertObjectAnnotation(mask, 'rectangle', ...
            bboxes(1:length(labels),:), labels,'FontSize',40,...
            'LineWidth', 3);
        
        % Draw the objects on the mask.
        image = insertObjectAnnotation(handles.image, 'rectangle', ...
            bboxes(1:length(labels),:), labels,'FontSize',40,...
            'LineWidth', 3);
        
        % Quando effettuo il labelling...
        if to_label > 0 && to_label <= size(tracks,2) && LABELLING==0
            mask = insertObjectAnnotation(mask, 'circle', ...
                [bboxes(to_label,1)+bboxes(to_label,3)/2, bboxes(to_label,2)+bboxes(to_label,4)/2,...
                mean(bboxes(to_label,3:4)')'/2], labels(to_label),'FontSize',40,'Color','r','LineWidth', 10);
        end
    end
    % Display the mask and the frame.
    axes(handles.webcam_image)
    imshow(image)
    axes(handles.mask_image)
    imshow(mask)
    hold on
    for i = 1:size(tracks,2)
        if size(tracks(i).detectedHistory,2) > 10
            plot(tracks(i).detectedHistory(1,end-10:end), ...
                tracks(i).detectedHistory(2,end-10:end),'r', ...
                'LineWidth', 2)
            plot(tracks(i).predictedHistory(1,end-10:end), ...
                tracks(i).predictedHistory(2,end-10:end),'g', ...
                'LineWidth', 2)
            
            plot(tracks(i).detectedHistory(1,end), ...
                tracks(i).detectedHistory(2,end),'.r', ...
                'LineWidth', 2, 'MarkerSize', 15)
            plot(tracks(i).predictedHistory(1,end), ...
                tracks(i).predictedHistory(2,end),'+g', ...
                'LineWidth', 2, 'MarkerSize', 15)
        end
    end
    
    if handles.angles_computation.Value == 1
                                
        for marker = ["base","mcp", "pip", "dip", "tip"]        
            % Plot dei marker                
            eval(strcat("plot(",marker,"(1),",marker,"(2),'k.','MarkerSize',5)"))
        end                        
        % Segmenti congiungenti
        plot([base(1); mcp(1)], [base(2); mcp(2)],'y','LineWidth', 3)
        plot([mcp(1); pip(1)], [mcp(2); pip(2)],'c','LineWidth', 3)
        plot([pip(1); dip(1)], [pip(2); dip(2)],'m','LineWidth', 3)
        plot([tip(1); pip(1)], [tip(2); pip(2)],'k','LineWidth', 3)
    end
    
    hold off
end



function [assignments, unassignedTracks, unassignedDetections] = ...
    detectionToTrackAssignment(tracks, centroids)

nTracks = length(tracks);
nDetections = size(centroids, 1);

% Compute the cost of assigning each enable_detection to each track.
cost = zeros(nTracks, nDetections);
for i = 1:nTracks
    cost(i, :) = distance(tracks(i).kalmanFilter, centroids);
end

% Solve the assignment problem.
costOfNonAssignment = 50;
[assignments, unassignedTracks, unassignedDetections] = ...
    assignDetectionsToTracks(cost, costOfNonAssignment);

function tracks = updateAssignedTracks(tracks, assignments, centroids, bboxes)
numAssignedTracks = size(assignments, 1);

for i = 1:numAssignedTracks
    
    trackIdx = assignments(i, 1);
    detectionIdx = assignments(i, 2);
    centroid = centroids(detectionIdx, :);
    bbox = bboxes(detectionIdx, :);
    
    % Correct the estimate of the object's location
    % using the new enable_detection.
    correct(tracks(trackIdx).kalmanFilter, centroid);
    
    % Replace predicted bounding box with detected
    % bounding box.
    tracks(trackIdx).bbox = bbox;
    
    % Update track's age.
    tracks(trackIdx).age = tracks(trackIdx).age + 1;
    
    % Update visibility.
    tracks(trackIdx).totalVisibleCount = ...
        tracks(trackIdx).totalVisibleCount + 1;
    tracks(trackIdx).consecutiveInvisibleCount = 0;
    
    % Append new detected points
    tracks(trackIdx).detectedHistory = [tracks(trackIdx).detectedHistory, ...
        centroid'];
end

function tracks = updateUnassignedTracks(tracks, unassignedTracks)
for i = 1:length(unassignedTracks)
    ind = unassignedTracks(i);
    tracks(ind).age = tracks(ind).age + 1;
    tracks(ind).consecutiveInvisibleCount = ...
        tracks(ind).consecutiveInvisibleCount + 1;
end

function tracks = deleteLostTracks(tracks)
if isempty(tracks)
    return;
end

invisibleForTooLong = 30;
ageThreshold = 10;

% Compute the fraction of the track's age for which it was visible.
ages = [tracks(:).age];
totalVisibleCounts = [tracks(:).totalVisibleCount];
visibility = totalVisibleCounts ./ ages;

% Find the indices of 'lost' tracks.
lostInds = (ages < ageThreshold & visibility < 0.6) | ...
    [tracks(:).consecutiveInvisibleCount] >= invisibleForTooLong;

% Delete lost tracks.
tracks = tracks(~lostInds);

function [tracks, nextId] = createNewTracks(tracks, centroids, bboxes, unassignedDetections, nextId)

centroids = centroids(unassignedDetections, :);
bboxes = bboxes(unassignedDetections, :);

for i = 1:size(centroids, 1)
    
    centroid = centroids(i,:);
    bbox = bboxes(i, :);
    
    % configureKalmanFilter
    % MotionModel: modello utilizzato dal filtro, può essere
    % 'ConstantVelocity' o 'ConstantAcceleration'
    % InitialLocation (Z0): rappresenta la prima misura utile
    % InitialEstimateError (P): rappresenta l'errore nella
    % stima iniziale come varianza sulla posizione e sulla velocità
    % MotionNoise (Q): rumore di processo sulla posizione e velocità
    % MeasurementNoise (R): rumore di misura in pixel
    kalmanFilter = configureKalmanFilter('ConstantVelocity', ...
        centroid, [1, 1]*1e-8, [1, 1]*1e8, 10);
    
    % Create a new track.
    newTrack = struct(...
        'id', nextId, ...
        'bbox', bbox, ...
        'kalmanFilter', kalmanFilter, ...
        'age', 1, ...
        'totalVisibleCount', 1, ...
        'consecutiveInvisibleCount', 0, ...
        'detectedHistory', [], ...
        'predictedHistory', []);
    
    % Add it to the array of tracks.
    tracks(end + 1) = newTrack;
    
    % Increment the next id.
    nextId = num2str(str2num(nextId) + 1);
    disp(newTrack);
end

function tracks = initializeTracks()
% create an empty array of tracks
tracks = struct(...
    'id', {}, ...
    'bbox', {}, ...
    'kalmanFilter', {}, ...
    'age', {}, ...
    'totalVisibleCount', {}, ...
    'consecutiveInvisibleCount', {}, ...
    'detectedHistory', {}, ...
    'predictedHistory', {});

function tracks = predictNewLocationsOfTracks(tracks)
for i = 1:length(tracks)
    bbox = tracks(i).bbox;
    
    % Predict the current location of the track.
    predictedCentroid = predict(tracks(i).kalmanFilter);
    
    % Shift the bounding box so that its center is at
    % the predicted location.
    bbox_position = round(predictedCentroid - bbox(3:4) / 2);
    tracks(i).bbox = [bbox_position, bbox(3:4)];
    
    % Append predicted position
    tracks(i).predictedHistory = [tracks(i).predictedHistory, ...
        predictedCentroid'];
end


% --- Executes on button press in plot.
function plot_Callback(hObject, eventdata, handles)
% hObject    handle to plot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of plot


% --- Executes on button press in angles_computation.
function angles_computation_Callback(hObject, eventdata, handles)
% hObject    handle to angles_computation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of angles_computation


% --- Executes on button press in label.
function label_Callback(hObject, eventdata, handles)
% hObject    handle to label (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Definisco le variabili globali
global LABELLING;
global to_label;
global tracks;

if LABELLING == 1 % Inizio il labelling se LABELLING è pari a 1 (impostato all'inizio)
    to_label = 1;
    LABELLING = 0;
    handles.label.BackgroundColor = 'r'; % Cambio il colore del button "Labelling" in rosso
    handles.marker_label.String = tracks(to_label).id; % Scrivo nella textbox in nome della prima traccia da etichettare
    
else % Finisco il labelling (LABELLING = 0)
    to_label = 0;
    LABELLING = 1;
    handles.label.BackgroundColor = 'g';
    handles.label.Visible = 'off';
    handles.marker_label.Visible = 'off';
end
guidata(hObject, handles);


function marker_label_Callback(hObject, eventdata, handles)
% hObject    handle to marker_label (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of marker_label as text
%        str2double(get(hObject,'String')) returns contents of marker_label as a double

% Definizione variabili globali
global tracks;
global to_label;
global LABELLING;

% Assegno alla variabile new_label il contenuto della textbox current_label
new_label = string(handles.marker_label.String);

% Se ho un numero di marker da etichettare maggiore di 0 e minore del
% numero di tracce, e se LABELLING = 0 (sto effettuando il labelling) entro
% nell'if
if to_label > 0 && to_label <= size(tracks,2) && LABELLING == 0
    tracks(to_label).id = new_label; % Assegno in nome new_label al campo id della traccia individuata dall'indice to_label
    to_label = to_label + 1; % Incremento l'indice to_label
    if to_label <= size(tracks,2)
        handles.marker_label.String = tracks(to_label).id; % Questo comando è
        % per fare in modo che all'interno della textbox sia mostrato
        % il nome attuale del marker che devo etichettare
    end
end
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function marker_label_CreateFcn(hObject, eventdata, handles)
% hObject    handle to marker_label (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in antropometric.
function antropometric_Callback(hObject, eventdata, handles)
% hObject    handle to antropometric (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
filename = handles.image_filename.String;
imwrite(handles.image, strcat(filename,".png"))


% --- Executes on button press in enable_antropometric.
function enable_antropometric_Callback(hObject, eventdata, handles)
% hObject    handle to enable_antropometric (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of enable_antropometric
