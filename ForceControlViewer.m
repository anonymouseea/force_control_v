function fig = ForceControlViewer(visibility)
% 四轴悬臂俯视示意，不连接控制器，不发送真机目标。
% 运行 ForceControlViewer。控制器轴序：大臂转、升降、伸缩、末端转。
if nargin == 0
    visibility = 'on';
end

base = [0, 0];                  % 大臂回转中心，毫米
fixedLength = 180;              % 示意固定臂长，毫米
toolLength = 100;               % 朝向箭头长度，不是 TCP 偏置
qMin = [-44, -820, 5, -60];      % 沿用 C++ 的关节软件限位
qMax = [44, 1148, 848, 60];
q = [0, 0, 300, 0];
lockedYaw = sum(q([1, 4]));

fig = uifigure('Name','四轴悬臂｜俯视示意', ...
    'Position',[100, 100, 1000, 560], 'Visible',visibility);
gridLayout = uigridlayout(fig,[1 2]);
gridLayout.ColumnWidth = {'1x',380};
ax = uiaxes(gridLayout);
panel = uigridlayout(gridLayout,[13 3]);
panel.ColumnWidth = {108,'1x',82};
panel.RowHeight = {27,28,25,28,28,28,30,25,32,32,32,32,'1x'};

label(1,1:3,'关节构型演示｜与真机标定无关');
label(2,1,'控制模式');
mode = uidropdown(panel,'Items',{'位置模式','旋转模式'}, ...
    'ItemsData',{'position','rotation'},'Tag','Mode', ...
    'ValueChangedFcn',@changeMode);
place(mode,2,2:3);
label(3,1:3,'末端目标（基座坐标）');
labels = {'X（mm）','Y（mm）','Rz（°）'};
pose = gobjects(1,3);
for i = 1:3
    label(i+3,1,labels{i});
    pose(i) = uieditfield(panel,'numeric','Tag',sprintf('Pose%d',i));
    place(pose(i),i+3,2:3);
end
button = uibutton(panel,'Text','计算逆解','Tag','Solve', ...
    'ButtonPushedFcn',@solveIK);
place(button,7,1:3);

label(8,1:3,'关节拖动条 / 数值输入');
names = {'1 大臂（°）','2 升降','3 伸缩（mm）','4 末端（°）'};
slider = gobjects(1,4);
value = gobjects(1,4);
for i = 1:4
    label(i+8,1,names{i});
    slider(i) = uislider(panel,'Limits',[qMin(i), qMax(i)], ...
        'Value',q(i),'MajorTicks',[],'MinorTicks',[], ...
        'Tag',sprintf('Slider%d',i), ...
        'ValueChangingFcn',@(src,event) changeJoint(i,event.Value,i), ...
        'ValueChangedFcn',@(src,event) changeJoint(i,event.Value,0));
    place(slider(i),i+8,2);
    value(i) = uieditfield(panel,'numeric','Limits',[qMin(i), qMax(i)], ...
        'Value',q(i),'Tag',sprintf('Joint%d',i), ...
        'ValueChangedFcn',@(src,event) changeJoint(i,src.Value,0));
    place(value(i),i+8,3);
end
status = label(13,1:3,'位置模式：逆解保持进入时的末端朝向。');
status.WordWrap = 'on';
status.Tag = 'Status';
refresh(0);

    function h = label(row,columns,str)
        h = uilabel(panel,'Text',str);
        place(h,row,columns);
    end

    function place(h,row,columns)
        h.Layout.Row = row;
        if numel(columns) > 1
            % 布局跨列只接受起止列，不接受完整列号列表。
            columns = [columns(1), columns(end)];
        end
        h.Layout.Column = columns;
    end

    function changeMode(~,~)
        lockedYaw = sum(q([1, 4]));   % 切换时以当前姿态作为参考
        if isRotation()
            status.Text = '旋转模式：锁定前三轴，只改变末端关节。';
        else
            status.Text = '位置模式：输入 X、Y；逆解保持末端朝向。';
        end
        refresh(0);
    end

    function yes = isRotation()
        yes = strcmp(mode.Value,'rotation');
    end

    function changeJoint(i,newValue,dragging)
        if isRotation() && i ~= 4
            return
        end
        q(i) = newValue;
        if ~isRotation()
            % 手动修改关节用于摆出示意姿态，之后从此朝向做位置逆解。
            lockedYaw = sum(q([1, 4]));
        end
        status.Text = '手动关节调整（仅仿真）。';
        refresh(dragging);
    end

    function [xy,yaw] = fk(joint)
        xy = base + (fixedLength + joint(3))*[cosd(joint(1)), sind(joint(1))];
        yaw = joint(1) + joint(4);
    end

    function solveIK(~,~)
        candidate = q;
        if isRotation()
            % 和 C++ 自转模式一样：固定前三轴，不做位置逆解。
            desiredYaw = pose(3).Value;
            candidate(4) = wrapAngle(desiredYaw - q(1));
            desiredXY = fk(q);
        else
            desiredXY = [pose(1).Value, pose(2).Value];
            desiredYaw = lockedYaw;
            delta = desiredXY - base;
            candidate(1) = atan2d(delta(2),delta(1));
            candidate(3) = hypot(delta(1),delta(2)) - fixedLength;
            candidate(4) = wrapAngle(desiredYaw - candidate(1));
        end
        if any(~isfinite(candidate)) || any(candidate < qMin-1e-9 | candidate > qMax+1e-9)
            status.Text = '目标超出关节限位，保留当前姿态。';
            return
        end
        candidate = min(qMax,max(qMin,candidate));
        [xy,yaw] = fk(candidate);
        if norm(xy-desiredXY) > 1e-6 || abs(wrapAngle(yaw-desiredYaw)) > 1e-6
            status.Text = '正解回代未达到目标，保留当前姿态。';
            return
        end
        q = candidate;
        status.Text = '目标已应用，正解回代通过。';
        refresh(0);
    end

    function refresh(dragging)
        [xy,yaw] = fk(q);
        pose(1).Value = xy(1);
        pose(2).Value = xy(2);
        pose(3).Value = yaw;
        if isRotation()
            pose(1).Editable = 'off';
            pose(2).Editable = 'off';
            pose(3).Editable = 'on';
        else
            pose(1).Editable = 'on';
            pose(2).Editable = 'on';
            pose(3).Editable = 'off';
        end
        for k = 1:4
            if k ~= dragging
                slider(k).Value = q(k);
            end
            value(k).Value = q(k);
            enable = 'on';
            if isRotation() && k ~= 4
                enable = 'off';
            end
            slider(k).Enable = enable;
            value(k).Enable = enable;
        end
        cla(ax);
        hold(ax,'on');
        elbow = base + fixedLength*[cosd(q(1)),sind(q(1))];
        plot(ax,[base(1),elbow(1)],[base(2),elbow(2)],'-','LineWidth',8,'Color',[.16 .34 .55]);
        plot(ax,[elbow(1),xy(1)],[elbow(2),xy(2)],'-','LineWidth',4,'Color',[.17 .62 .72]);
        plot(ax,base(1),base(2),'ko','MarkerFaceColor',[.2 .2 .2],'MarkerSize',8);
        plot(ax,xy(1),xy(2),'o','MarkerFaceColor',[.85 .35 .14],'MarkerEdgeColor','none');
        quiver(ax,xy(1),xy(2),toolLength*cosd(yaw),toolLength*sind(yaw),0, ...
            'Color',[.85 .35 .14],'LineWidth',2,'MaxHeadSize',.6);
        axis(ax,'equal');
        xlim(ax,[-160, fixedLength + qMax(3) + toolLength + 80]);
        ylim(ax,[-870, 870]);
        grid(ax,'on');
        xlabel(ax,'基座 X（mm）');
        ylabel(ax,'基座 Y（mm）');
        title(ax,sprintf('俯视图｜X %.1f  Y %.1f mm｜Rz %.1f°',xy(1),xy(2),yaw));
        hold(ax,'off');
    end

    function a = wrapAngle(a)
        a = mod(a+180,360)-180;
    end
end
