function axes = h5animate(filename, groupname, fig)
%H5ANIMATE create and animate a scene according to trajectories stored in an HDF5 file
%
%  H5ANIMATE(filename, groupname)
%  H5ANIMATE(filename, groupname, fig)
%
%  INPUT:
%    filename: 
%      the path of the HDF5 file
%    groupname: 
%      the path of the group, within the file. For instance '/simu1'. If
%      none is provided, '/' is tried.
%    fig:
%      an handle to the figure where the animation should be drawed. If
%      none is provided, a new figure is created
%
%  OUTPUT:
%    axes: an handle to the axes 

if nargin < 2
    groupname = '/';
end
[transforms_data, timeline] = load_scene_datasets(filename, groupname);
if nargin < 3
    fig = figure('Position',[100 100 600 600]);    
end
axes = gca();
H = [0, 0, 1, 0;
     1, 0, 0, 0;
     0, 1, 0, 0;
     0, 0, 0, 1];
parent = hgtransform('Parent', axes, 'Matrix', H, 'Tag', 'Ground');
transforms_handles = init_scene_transforms(fieldnames(transforms_data), parent);
nb_steps = numel(timeline);

    function update_callback(i)
    %Callback function which updates poses at step i
        names = fieldnames(transforms_data);
        for n = 1:numel(names)
            set(transforms_handles.(names{n}), 'Matrix', transforms_data.(names{n})(:,:,i));
        end
    end

update_callback(1);
arb_guislider(nb_steps, @update_callback, fig);

end

function p1p2 = joinpath(p1, p2)
%JOINPATH join two paths, taking care of leading/trailing slashes
%
%  Examples:
%     '/foo/bar' == joinpath('/foo', 'bar')
%                == joinpath('/foo/', 'bar')
%                == joinpath('/foo/', '/bar')

if (numel(p1) > 0) && (p1(end) == '/')
    p1 = p1(1:(end-1));
end
if (numel(p2) > 0) && (p2(1) == '/')
    p2 = p2(2:end);
end
p1p2 = [p1 '/' p2];
end

function transforms = init_scene_transforms(names, arg2)
%INIT_SCENE_TRANSFORMS create htransforms for each dataset in the datasets struct
%
%  transforms = INIT_SCENE_TRANSFORMS(datasets, parent)
%  transforms = INIT_SCENE_TRANSFORMS(datasets, transforms)

if nargin < 2
    transforms = struct();
    parent = gca;
    axis('equal');
else
    if isstruct(arg2);
        transforms = arg2;
        parent = gca; % maybe we could get the axe from the transforms?
        axis('equal');
    else
        transforms = struct();
        parent = arg2;
    end
end
for n = 1:numel(names)
    if ~isfield(transforms, names{n})
        h = hgtransform('Tag',  names{n}, 'Parent', parent);
        transforms.(names{n}) = h;
        draw_frame(h);
    end
end
end

function [transforms, timeline] = load_scene_datasets(filename, groupname)
%LOAD_SCENE_DATASETS load matrix, translate and wrench datasets and from an hdf5 group
%
%  datasets = LOAD_SCENE_DATASETS(group)
%
%  INPUT:
%    cf. main function
%    Within the actual hdf5 group, the data should be laid of as in this
%    example:
%      groupname/timeline (nb_steps)
%      groupname/transforms/Arm (nb_steps x 4 x 4)
%      groupname/transforms/ForeArm (nb_steps x 4 x 4)
%      groupname/transforms/CenterOfMasses (nb_steps x 3)
%
%  OUTPUT:
%    a struct with the following structure:
%      transforms.Arm (4 x 4 x nb_steps)
%      transforms.ForeArm (4 x 4 x nb_steps)
%      transforms.CenterOfMasses (3 x nb_steps)
%    timeline (1 x nb_steps)

timeline = hdf5read(filename, joinpath(groupname, '/timeline'));
nb_steps = numel(timeline);
group = findgroup(filename, joinpath(groupname, '/transforms'));
data = group.Datasets;
transforms = struct();
for i = 1:numel(data)
    d = hdf5read(data(i),...
                 'V71Dimensions', true);
    name = endname(data(i).Name);
    if all(size(d) == [4, 4, nb_steps])
        transforms.(name) = d;
    elseif all(size(d) == [3, nb_steps])
        % this is a translation
        H = repmat(eye(4), [1,1,nb_steps]);
        H(1:3,4,:) = d;
        transforms.(name) = d;
    else
        warning(['skipping dataset "' data.Name '" which has wrong dimensions']);
    end
end
end

function name = endname(path)
%ENDNAME return the last part of a path-like string, such as bar in /foo/bar
names = regexp(path, '/', 'split');
name = names{end};
assert(numel(name)>0);
end

function group = findgroup(filename, groupname)
%FINDGROUP return the group whose name is matching groupname.
if groupname(1) ~= '/'
    error('group name should begin with a slash (/)')
end
if numel(groupname) > 1 && groupname(end) == '/'
    error('group name should not end with a slash (/)')
end
info = hdf5info(filename);
group = info.GroupHierarchy;
if groupname == '/'
    return
end
names = regexp(groupname, '/', 'split');
for i=2:numel(names) % start at i=1 because the first name is always empty
    found = false;
    for group=group.Groups
        gnames = regexp(group.Name, '/', 'split');
        assert(numel(gnames)==i);
        if strcmp(gnames(i), names(i))
            found = true;
            break;
        end
    end
    if ~found
        error(['file ' filename ' has no group named ' groupname]);
    end
end
end

function hPanel = arb_guislider(nSnapshot, update_external, hFigure, position)
% ARB_GUISLIDER: Add widgets to loop/scroll over a range.
% ARB_GUISLIDER: Add widgets to loop/scroll over a range. This is
% useful to play a simulation by looping over the snapshots
%
%  The added widgets are
%    * a "play" button, to play/pause the loop
%    * an edit box to display/set the current snapshot
%    * an edit box to display/set the loop velocity (how many snapshots will
%      be jumped)
%    * a slider, to display/set the current snapshot
%  Each time the user interacts with these widgets, an external function is
%  called to update the gui.
%
% h = arb_gui_snapshotslider(nSnapshot,position,update_external,hFigure)
%   * nSnapshot: number of snapshots (we will loop/scroll over the
%     1:nSnapshot range)
%   * position: position an size of the panel, in pixels (see matlab uipanel
%     help)
%   * update_external: a function handle to the external function called in
%     order to update the gui (typically this will be a nested function)
%   * hFigure: graphic handle to the figure these widgetss shoud be added
%     to.
%
%
% +------------------------------------------------------------------+  ---
% |         ^                                                        |   ^
% |         a                                                        |   |
% |         v                                                        |   |
% |     +------+     +-----------+     +-----------+     +------+    |   |
% | <a> | play | <b> |nSnapJumped| <b> |currentSnap| <b> |slider|<a> |   c
% |     +------+     +-----------+     +-----------+     +------+    |   |
% |         ^                                                        |   |
% |         a                                                        |   |
% |         v                                                        |   v
% +------------------------------------------------------------------+  ---
%
% |<----------------------------d----------------------------------->|
%
% a = panelSpace
% b = horizSpace
% c = position(4)
% d = position(3)

horizSpace=4;
panelSpace=0;

if nargin <4
    position = get(hFigure,'Position');
    position(1)=0;
    position(2)=0;
    position(4)=min(20,position(4));
end
buttonHeight = position(4)-2*panelSpace;

if (position(3) < (120+3*horizSpace+2*panelSpace) + 40)
    warning('ARBORIS:miscError','position is too small')
end

% create the panel (container)
hPanel = uipanel('Parent',hFigure,...
    'FontSize',10,...
    'Unit','pixels',...
    'Position',position,...
    'BorderWidth',0);

% create the "Play" button
hButtonPlay = uicontrol(hPanel,'Style','togglebutton',...
    'String','Play',...
    'Value',0,...
    'Position',[ panelSpace panelSpace 40 buttonHeight ],...
    'Callback',{@buttonPlay_Callback});

% create an edit box for nSnapJumped
nSnapJumped = 1;
hEditNSnapJumped = uicontrol(hPanel,'Style','edit',...
    'Position',[ (40+panelSpace+horizSpace) panelSpace 40 buttonHeight ],...
    'Callback',{@editNSnapJumped_Callback},...
    'String',num2str(nSnapJumped));

% create an edit box for the currentSnap value
currentSnap = 1;
hEditCurrentSnap = uicontrol(hPanel,'Style','edit',...
    'Position',[ (80+panelSpace+2*horizSpace) panelSpace 40 buttonHeight ],...
    'Callback',{@editCurrentSnap_Callback},...
    'String',num2str(currentSnap));

% create a slider
hSliderSnapshot = uicontrol(hPanel,'Style','slider',...
    'Max',nSnapshot,'Min',1,'Value',1,...
    'SliderStep',[1 1]/(nSnapshot-1),...
    'Position',[ (120+panelSpace+3*horizSpace) panelSpace (position(3)-120-3*horizSpace-2*panelSpace) buttonHeight ],...
    'Callback',{@sliderSnapshot_Callback},...
    'Value',currentSnap);

    function update_figure()
    % update_figure according to currentSnap
        set(hEditCurrentSnap,'String',num2str(currentSnap));
        set(hSliderSnapshot,'Value',currentSnap);
        update_external(currentSnap);
        drawnow();
    end

    function play()
    % play the simulation.
        for currentSnap=currentSnap:nSnapJumped:nSnapshot
            if get(hButtonPlay,'Value')==get(hButtonPlay,'Min')
                % if the button has been pulled, stop the movie
                break;
            end
            update_figure();
        end
        % pull the button
        if get(hButtonPlay,'Value')==get(hButtonPlay,'Max')
            min = get(hButtonPlay,'Min');
            set(hButtonPlay,'Value',min);
            currentSnap=1;
            update_figure();
        end
    end

    function editNSnapJumped_Callback(hObject, eventdata)
        newNb=str2double(get(hObject,'String'));
        if isnan(newNb)
            errordlg('You must enter an integer value','Bad Input','modal')
        else
            nSnapJumped = round(newNb);
        end
        set(hObject,'String',num2str(nSnapJumped));
    end

    function sliderSnapshot_Callback(hObject, eventdata)
        s=get(hObject,'Value');
        currentSnap=round(s);
        update_figure();
    end

    function editCurrentSnap_Callback(hObject, eventdata)
        newNb=str2double(get(hObject,'String'));
        if isnan(newNb)
            errordlg('You must enter an integer value','Bad Input','modal')
        else
            currentSnap = round( min( max(newNb,1) , nSnapshot) );
            update_figure();
        end
    end

    function buttonPlay_Callback(hObject, eventdata)
        button_state = get(hObject,'Value');
        if button_state == get(hObject,'Max')
            % toggle button is pressed
            play();
        elseif button_state == get(hObject,'Min')
            % toggle button is not released, nothing to do here
        end
    end

end

function handle = draw_frame(parent, length)
%DRAW_FRAME draw a frame as three lines
    if nargin < 1
        parent = gca;
    end
    if nargin < 2
        length = 1;
    end
    handle = nan(size(parent));
    for i = 1:numel(parent)
        handle(1) = hggroup();
        line([0, length], [0, 0], [0, 0], 'Color', 'r', 'Parent', parent(i));
        line([0, 0], [0, length], [0, 0], 'Color', 'g', 'Parent', parent(i));
        line([0, 0], [0, 0], [0, length], 'Color', 'b', 'Parent', parent(i));
    end
end

