%==========================================================================

% MATLAB code for Project 3 (Planning Class)
% Point robot planning using A* Algorithm
% Written by Yash Shah (115710498)
% email ID: ysshah95@umd.edu
% 
% Implementation of A* Algorithm for finding a shortest path 
% between two points in an area (with obstacles)

%==========================================================================

clc
clear all
close all

% live_status = 0; % Change this to 1 to see node generation live on image 
% Video = 0; % Change this to 1 if you want to make a live video of the algorithm running.

% if Video == 1
%     Output_Video = VideoWriter('Result');
%     Output_Video.FrameRate = 100;
%     open(Output_Video);
% end

% Defining the whole plot from the given coordinates

% Plotting Obstacle

x2 = ([-7.33 -6.55 -6.55 -7.33]+8)*10;
y2 = ([2.725 2.725  0.725 0.725]+5)*10;
fill(x2,y2,'k')
drawnow
hold on 
x3 = ([-7.35 -6.55 -6.55 -7.33]+8)*10;
y3 = ([0.725 0.725 -1.275 -1.275]+5)*10;
fill(x3,y3,'k')
drawnow

hold on 
x4 = ([-6.15 -4.55 -4.55 -6.15]+8)*10;
y4 = ([-3.55 -3.55 -4.85 -4.85]+5)*10;
fill(x4,y4,'k')
drawnow

hold on 
x5 = ([5.125 5.925 5.925 5.125]+8)*10;
y5 = ([-2.375 -2.375 -4.375 -4.375]+5)*10;
fill(x5,y5,'k')
drawnow

hold on 
x6 = ([3.925 4.725 4.725 3.925]+8)*10;
y6 = ([-2.375 -2.375 -4.375 -4.375]+5)*10;
fill(x6,y6,'k')
drawnow
hold on 

x7 = ([-3.4 -1.8 -1.8 -3.4]+8)*10;
y7 = ([ 1.6  1.6  -1.6 -1.6]+5)*10;
fill(x7,y7,'k')
drawnow
hold on 
x8 = 10*([0.3475 1.9475 1.9475 0.3475]+8);
y8= ([1.6  1.6  -1.6 -1.6]+5)*10;
fill(x8,y8,'k')
drawnow
hold on

% walls
x = ([-7.425 7.425]+8)*10;
y = ([-4.925 +4.925]+5)*10;

xlim([0 x(2)])
ylim([0 y(2)])

status = false; % This variable becomes true only if all the points are in the free space 
while ~status
    
    title("Select End Point on the Graph")
    
    [r c] = ginput(1);
    
    x_g = round(r(1),0);
    y_g = round(c(1),0);

%     prompt_x_start = 'Enter X Coordinate of Starting Point between 0 to 250:  ';
%     x_s = input(prompt_x_start);
%     
%     prompt_y_start = 'Enter Y Coordinate of Starting Point between 0 to 150:  ';
%     y_s = input(prompt_y_start);
%     
%     prompt_x_goal = 'Enter X Coordinate of Goal Point between 0 to 250:  ';
%     x_g = input(prompt_x_goal);
%     
%     prompt_y_goal = 'Enter Y Coordinate of Goal Point between 0 to 150:  ';
%     y_g = input(prompt_y_goal);
     

    x_s = (6.6+8)*10;
    y_s = (4.1+5)*10;

    StartNode = [x_s,y_s];
    GoalNode = [x_g,y_g];
    
    % Check if the points given are inside the obstacle or not using half
    % plane method 

    in_start = insidepoly_halfplane(x_s,y_s);
    in_goal = insidepoly_halfplane(x_g,y_g);

    if in_start
        status = false;
        disp('The start point provided is inside the obstacle.');
    elseif in_goal
        status = false;
        disp('The goal point provided is inside the obstacle.');
    else
        status = true;
    end
end

% PLot the start and End node 
drawnow 
plot(StartNode(1),StartNode(2),'s','color','green','markers',10)
drawnow 
plot(GoalNode(1),GoalNode(2),'s','color','red','markers',10)
title("Searching for the Optimum Path")
hold on 


txt1 = '\leftarrow Start Node';
txt2 = '\leftarrow Goal Node';


drawnow
text(StartNode(1),StartNode(2),txt1)
text(GoalNode(1),GoalNode(2),txt2)
drawnow
hold on

tic

% Start A* algorithm only if status is true (i.e all the points are in
% free space)


if status 
    
    % Initialize the variables.
    Nodes = [];
    NodesInfo = [];
    % Initialize the start node.
    Nodes(:,:,1) = StartNode;
    % Get id of the start Node
    id = getid(StartNode);    
    % Initialize NodeInfo for start node
    % NodesInfo = [Node#, ParentNode#,ctc,ctg,cost,id];
    NodesInfo(:,:,1) = [1,0,0,0,0,id]; 
    
    ClosedNodes = [];
    ClosedNodesInfo = [];
    
    cost_linear = 1;
    cost_diag = sqrt(2);
    
    % Initialize the child and parent node number variables.
    i = 2; % Child node number variable
    j = 1; % Parent Node Number Variable
    z = 1;
    while true
        % Initialize the parent node in each loop
        CurrentNode = Nodes(:,:,j);
        
        ClosedNodes(:,:,z) = CurrentNode;
        id = getid(CurrentNode);
        ClosedNodesInfo(:,:,z) = [id];
%         if live_status == 1
%             drawnow
%         end
%         plot(CurrentNode(1),CurrentNode(2),'.','color','red')
%         if Video == 1
%             writeVideo(Output_Video,getframe);
%         end
        [StatusL, NewNodeL] = ActionMoveLeft(CurrentNode);
        if StatusL == true
            in = insidepoly_halfplane(NewNodeL(1),NewNodeL(2));
            id_test = getid(NewNodeL);
            % Search if the New Node generated is present in Nodes array or not. 
            if in == false
                % Save the node only if the node generated in not inside
                % the obstacle
%                 tic
                if (~any(id_test == NodesInfo(1,6,:))) 
%                     toc
                    ctc = NodesInfo(:,3,j);
                    g = ctc + cost_linear;
                    h = (sqrt(((NewNodeL(1)-GoalNode(1))^2)+((NewNodeL(2)-GoalNode(2))^2)));
                    f = g+h;
                    Nodes(:,:,i) = NewNodeL;
                    NodesInfo(:,:,i) = [i,j,g,h,f,id_test];
                    OpenNode(:,:,i) = [id_test,f];
                    i = i+1;                    
%                     if live_status == 1
%                         drawnow 
%                     end
%                     plot(NewNodeL(1),NewNodeL(2),'.','color','green')
%                     if Video == 1
%                         writeVideo(Output_Video,getframe);
%                     end
                
                elseif  (~any(id_test == ClosedNodesInfo(1,1,:)))
                    k = find(id_test == NodesInfo(1,6,:));
                    cost = NodesInfo(1,5,k);
                    ctc = NodesInfo(:,3,j);
                    g = ctc+cost_linear;
                    h = sqrt(((NewNodeL(1)-GoalNode(1))^2)+((NewNodeL(2)-GoalNode(2))^2));
                    f = g+h;
                    if cost>f
                        NodesInfo(:,:,k) = [i,j,g,h,f,id_test];
                    end
                end
                if NewNodeL(1) == GoalNode(1) && NewNodeL(2) == GoalNode(2)
                    break
                end   
            end
        end

        [StatusR, NewNodeR] = ActionMoveRight(CurrentNode);
        if StatusR == true
            in = insidepoly_halfplane(NewNodeR(1),NewNodeR(2));
            id_test = getid(NewNodeR);
            % Search if the New Node generated is present in Nodes array or not. 
            if in == false
                % Save the node only if the node generated in not inside
                % the obstacle
                if (~any(id_test == NodesInfo(1,6,:))) 
                    ctc = NodesInfo(:,3,j);
                    g = ctc + cost_linear;
                    h = (sqrt(((NewNodeR(1)-GoalNode(1))^2)+((NewNodeR(2)-GoalNode(2))^2)));
                    f = g+h;
                    Nodes(:,:,i) = NewNodeR;
                    NodesInfo(:,:,i) = [i,j,g,h,f,id_test];
                    i = i+1;                    
%                     if live_status == 1
%                         drawnow 
%                     end
%                     plot(NewNodeR(1),NewNodeR(2),'.','color','green')
%                     if Video == 1
%                         writeVideo(Output_Video,getframe);
%                     end
                    
                elseif  (~any(id_test == ClosedNodesInfo(1,1,:)))
                    k = find(id_test == NodesInfo(1,6,:));
                    cost = NodesInfo(1,5,k);
                    ctc = NodesInfo(:,3,j);
                    g = ctc+cost_linear;
                    h = sqrt(((NewNodeR(1)-GoalNode(1))^2)+((NewNodeR(2)-GoalNode(2))^2));
                    f = g+h;
                    if cost>f
                        NodesInfo(:,:,k) = [i,j,g,h,f,id_test];
                    end
                end
                if NewNodeR(1) == GoalNode(1) && NewNodeR(2) == GoalNode(2)
                    break
                end   
            end
        end  
        
        [StatusU, NewNodeU] = ActionMoveUp(CurrentNode);
        if StatusU == true
            in = insidepoly_halfplane(NewNodeU(1),NewNodeU(2));
            id_test = getid(NewNodeU);
            % Search if the New Node generated is present in Nodes array or not. 
            if in == false
                % Save the node only if the node generated in not inside
                % the obstacle
                if (~any(id_test == NodesInfo(1,6,:))) 
                    ctc = NodesInfo(:,3,j);
                    g = ctc + cost_linear;
                    h = (sqrt(((NewNodeU(1)-GoalNode(1))^2)+((NewNodeU(2)-GoalNode(2))^2)));
                    f = g+h;
                    Nodes(:,:,i) = NewNodeU;
                    NodesInfo(:,:,i) = [i,j,g,h,f,id_test];
                    i = i+1;                    
%                     if live_status == 1
%                         drawnow 
%                     end
%                     plot(NewNodeU(1),NewNodeU(2),'.','color','green')
%                     if Video == 1
%                         writeVideo(Output_Video,getframe);
%                     end
                    
                elseif  (~any(id_test == ClosedNodesInfo(1,1,:)))
                    k = find(id_test == NodesInfo(1,6,:));
                    cost = NodesInfo(1,5,k);
                    ctc = NodesInfo(:,3,j);
                    g = ctc+cost_linear;
                    h = sqrt(((NewNodeU(1)-GoalNode(1))^2)+((NewNodeU(2)-GoalNode(2))^2));
                    f = g+h;
                    if cost>f
                        NodesInfo(:,:,k) = [i,j,g,h,f,id_test];
                    end
                end
                if NewNodeU(1) == GoalNode(1) && NewNodeU(2) == GoalNode(2)
                    break
                end   
            end
        end
        
        [StatusD, NewNodeD] = ActionMoveDown(CurrentNode);
        if StatusD == true
            in = insidepoly_halfplane(NewNodeD(1),NewNodeD(2));
            id_test = getid(NewNodeD);
            % Search if the New Node generated is present in Nodes array or not. 
            if in == false
                % Save the node only if the node generated in not inside
                % the obstacle
                if (~any(id_test == NodesInfo(1,6,:))) 
                    ctc = NodesInfo(:,3,j);
                    g = ctc + cost_linear;
                    h = (sqrt(((NewNodeD(1)-GoalNode(1))^2)+((NewNodeD(2)-GoalNode(2))^2)));
                    f = g+h;
                    Nodes(:,:,i) = NewNodeD;
                    NodesInfo(:,:,i) = [i,j,g,h,f,id_test];
                    i = i+1;                    
%                     if live_status == 1
%                         drawnow 
%                     end
%                     plot(NewNodeD(1),NewNodeD(2),'.','color','green')
%                     if Video == 1
%                         writeVideo(Output_Video,getframe);
%                     end
                    
                elseif  (~any(id_test == ClosedNodesInfo(1,1,:)))
                    k = find(id_test == NodesInfo(1,6,:));
                    cost = NodesInfo(1,5,k);
                    ctc = NodesInfo(:,3,j);
                    g = ctc+cost_linear;
                    h = sqrt(((NewNodeD(1)-GoalNode(1))^2)+((NewNodeD(2)-GoalNode(2))^2));
                    f = g+h;
                    if cost>f
                        NodesInfo(:,:,k) = [i,j,g,h,f,id_test];
                    end
                end
                if NewNodeD(1) == GoalNode(1) && NewNodeD(2) == GoalNode(2)
                    break
                end   
            end
        end
        
        [StatusDL, NewNodeDL] = ActionMoveDownLeft(CurrentNode);
        if StatusDL == true
            in = insidepoly_halfplane(NewNodeDL(1),NewNodeDL(2));
            id_test = getid(NewNodeDL);
            % Search if the New Node generated is present in Nodes array or not. 
            if in == false
                % Save the node only if the node generated in not inside
                % the obstacle
                if (~any(id_test == NodesInfo(1,6,:))) 
                    ctc = NodesInfo(:,3,j);
                    g = ctc + cost_linear;
                    h = (sqrt(((NewNodeDL(1)-GoalNode(1))^2)+((NewNodeDL(2)-GoalNode(2))^2)));
                    f = g+h;
                    Nodes(:,:,i) = NewNodeDL;
                    NodesInfo(:,:,i) = [i,j,g,h,f,id_test];
                    i = i+1;                    
%                     if live_status == 1
%                         drawnow 
%                     end
%                     plot(NewNodeDL(1),NewNodeDL(2),'.','color','green')
%                     if Video == 1
%                         writeVideo(Output_Video,getframe);
%                     end
                    
                elseif  (~any(id_test == ClosedNodesInfo(1,1,:)))
                    k = find(id_test == NodesInfo(1,6,:));
                    cost = NodesInfo(1,5,k);
                    ctc = NodesInfo(:,3,j);
                    g = ctc+cost_linear;
                    h = sqrt(((NewNodeDL(1)-GoalNode(1))^2)+((NewNodeDL(2)-GoalNode(2))^2));
                    f = g+h;
                    if cost>f
                        NodesInfo(:,:,k) = [i,j,g,h,f,id_test];
                    end
                end
                if NewNodeDL(1) == GoalNode(1) && NewNodeDL(2) == GoalNode(2)
                    break
                end   
            end
        end
        
        [StatusDR, NewNodeDR] = ActionMoveDownRight(CurrentNode);
        if StatusDR == true
            in = insidepoly_halfplane(NewNodeDR(1),NewNodeDR(2));
            id_test = getid(NewNodeDR);
            % Search if the New Node generated is present in Nodes array or not. 
            if in == false
                % Save the node only if the node generated in not inside
                % the obstacle
                if (~any(id_test == NodesInfo(1,6,:))) 
                    ctc = NodesInfo(:,3,j);
                    g = ctc + cost_linear;
                    h = (sqrt(((NewNodeDR(1)-GoalNode(1))^2)+((NewNodeDR(2)-GoalNode(2))^2)));
                    f = g+h;
                    Nodes(:,:,i) = NewNodeDR;
                    NodesInfo(:,:,i) = [i,j,g,h,f,id_test];
                    i = i+1;                    
%                     if live_status == 1
%                         drawnow 
%                     end
%                     plot(NewNodeDR(1),NewNodeDR(2),'.','color','green')
%                     if Video == 1
%                         writeVideo(Output_Video,getframe);
%                     end
                    
                elseif  (~any(id_test == ClosedNodesInfo(1,1,:)))
                    k = find(id_test == NodesInfo(1,6,:));
                    cost = NodesInfo(1,5,k);
                    ctc = NodesInfo(:,3,j);
                    g = ctc+cost_linear;
                    h = sqrt(((NewNodeDR(1)-GoalNode(1))^2)+((NewNodeDR(2)-GoalNode(2))^2));
                    f = g+h;
                    if cost>f
                        NodesInfo(:,:,k) = [i,j,g,h,f,id_test];
                    end
                end
                if NewNodeDR(1) == GoalNode(1) && NewNodeDR(2) == GoalNode(2)
                    break
                end   
            end
        end
        
        [StatusUL, NewNodeUL] = ActionMoveUpLeft(CurrentNode);
        if StatusUL == true
            in = insidepoly_halfplane(NewNodeUL(1),NewNodeUL(2));
            id_test = getid(NewNodeUL);
            % Search if the New Node generated is present in Nodes array or not. 
            if in == false
                % Save the node only if the node generated in not inside
                % the obstacle
                if (~any(id_test == NodesInfo(1,6,:))) 
                    ctc = NodesInfo(:,3,j);
                    g = ctc + cost_linear;
                    h = (sqrt(((NewNodeUL(1)-GoalNode(1))^2)+((NewNodeUL(2)-GoalNode(2))^2)));
                    f = g+h;
                    Nodes(:,:,i) = NewNodeUL;
                    NodesInfo(:,:,i) = [i,j,g,h,f,id_test];
                    i = i+1;                    
%                     if live_status == 1
%                         drawnow 
%                     end
%                     plot(NewNodeUL(1),NewNodeUL(2),'.','color','green')
%                     if Video == 1
%                         writeVideo(Output_Video,getframe);
%                     end
                    
                elseif  (~any(id_test == ClosedNodesInfo(1,1,:)))
                    k = find(id_test == NodesInfo(1,6,:));
                    cost = NodesInfo(1,5,k);
                    ctc = NodesInfo(:,3,j);
                    g = ctc+cost_linear;
                    h = sqrt(((NewNodeUL(1)-GoalNode(1))^2)+((NewNodeUL(2)-GoalNode(2))^2));
                    f = g+h;
                    if cost>f
                        NodesInfo(:,:,k) = [i,j,g,h,f,id_test];
                    end
                end
                if NewNodeUL(1) == GoalNode(1) && NewNodeUL(2) == GoalNode(2)
                    break
                end   
            end
        end
        
        [StatusUR, NewNodeUR] = ActionMoveUpRight(CurrentNode);
        if StatusUR == true
            in = insidepoly_halfplane(NewNodeUR(1),NewNodeUR(2));
            id_test = getid(NewNodeUR);
            % Search if the New Node generated is present in Nodes array or not. 
            if in == false
                % Save the node only if the node generated in not inside
                % the obstacle
                if (~any(id_test == NodesInfo(1,6,:))) 
                    ctc = NodesInfo(:,3,j);
                    g = ctc + cost_linear;
                    h = (sqrt(((NewNodeUR(1)-GoalNode(1))^2)+((NewNodeUR(2)-GoalNode(2))^2)));
                    f = g+h;
                    Nodes(:,:,i) = NewNodeUR;
                    NodesInfo(:,:,i) = [i,j,g,h,f,id_test];
                    i = i+1;                    
%                     if live_status == 1
%                         drawnow 
%                     end
%                     plot(NewNodeUR(1),NewNodeUR(2),'.','color','green')
%                     if Video == 1
%                         writeVideo(Output_Video,getframe);
%                     end
                    
                elseif  (~any(id_test == ClosedNodesInfo(1,1,:)))
                    k = find(id_test == NodesInfo(1,6,:));
                    cost = NodesInfo(1,5,k);
                    ctc = NodesInfo(:,3,j);
                    g = ctc+cost_linear;
                    h = sqrt(((NewNodeUR(1)-GoalNode(1))^2)+((NewNodeUR(2)-GoalNode(2))^2));
                    f = g+h;
                    if cost>f
                        NodesInfo(:,:,k) = [i,j,g,h,f,id_test];
                    end
                end
                if NewNodeUR(1) == GoalNode(1) && NewNodeUR(2) == GoalNode(2)
                    break
                end   
            end
        end    
        
%         tic
        
        min_cost = [inf,0];
        for y = 1:i-1
        id_test = getid(Nodes(:,:,y));
            if (~any(id_test == ClosedNodesInfo(1,1,:)))
                cost = NodesInfo(1,5,y);
                if cost<min_cost(1,1)
                    min_cost = [cost , y];
                end
            end
        end
        
%         toc

        j = min_cost(1,2);
        z = z+1;
%         if rem(z,10) == 0
%             z
%         end
        
    end
    
    q = i-1;
    count = 0;
    
    path = [];
    
    while q ~= 1 
        NodesInfo(:,:,q);
        a = Nodes(1,1,q);
        b = Nodes(1,2,q); 
        info = NodesInfo(1,2,q);
        q = info;
        count = count+1;
        plot(a,b,'.','color','blue')
        title("Hurray..!! Optimum path Found. The path is shown with Blue Dots")
        a1 =(a/10)-8;
        b1 = (b/10)-5;
        path = [path;[a1 b1]];
    end
    NodesInfo(:,:,q);
    a = Nodes(1,1,q);
    b = Nodes(1,2,q); 
    a1 = (a/10)-8;
    b1 = (b/10)-5;
    path = [path;[a1 b1]];
    
%     if Video == 1
%         writeVideo(Output_Video,getframe);
%     end
    
    
    

end

toc

% if Video == 1
%     close(Output_Video)
% end

save('Path.mat','path')   

