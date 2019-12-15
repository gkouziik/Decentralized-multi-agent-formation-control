%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %     The programm is build based on functions.     % %
% %      main() is the most significant function      % %
% %     There exist also some auxiliary functions     % %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% main function. Implements simulation and the control.
function main()
    %% --> Communication options <-- %%
    %%% -For a simulation closer to reality, we try to implement a communication model- %%%
    %%% We suppose communication between the agents is achieved throught Wi-Fi
    COM_const = 1000;     %constant in exponential distribution of signal transmition
    transmit_threshold = 0.5 ;  %must be within (0,1]

    %% --> Time Unit properties <-- %%
    TS = 0.001 ;      %time step     : In every time step agent pos,vel and acc are updated
    S2D = 15;        %Steps to Draw : Steps that must passed in order to draw
    S2U = 9;        %Steps to Update: Update isn't happening in every time step. It happens every S2U time steps
    S2DP = 50;       %Agent drops an agent state which is 200 time steps old

    %% --> Agent options<-- %%
    N = 7;         %number of agents
    %initiate
    agentPos = 1000+300*randn(N,2);    %the position in plane (x,y) | random Gaussian spread about the (0,0)
    agentVel = zeros(N,2);    %velocity is applied as control to each agent
    agentPackMom = zeros(N,N,5);     % u : estimator output ,i.e. estimation of global moments   |   agentPackMom & agentPackIntern consist the "transmition packet"
    %initialize that. (In the beginning each agent knows only his position, thus this composes the global moment estimation)
    for i=1:N
        agentPackMom(i,i,:) = fi(agentPos(i,:));
    end
    agentPackIntern = zeros(N,N,5) ; % w : the internal estimate. 1-dim -> which agent. 2-dim -> which agent's packet, 3-dim -> 5 are the first and second order moments
    agentPackAge = zeros(N,N,1);     % expresses how old is a packet (state of another agent) inside an agent. If old enough (>packet_old_limit) then agent drops it
    agentSteps2Update = floor(S2U + 2*randn(N,1));
    agentCanComm = zeros(N,N);       % says whether an agent can communicate with another (1 can, 0 cannot) 

    %% --> Control options <-- %%
    Kp = 120/N ;
    Ki = 30/N ;
    gamma = 10;
    desiredForm = [-500 -300 50000 40000 140000]; %(1,2) -> position (3)->x spread (4)->xy spread (5)->y spread
    gaMatrix = diag([10, 10, 0.0001, 0.0002, 0.0001]);  % the positive-definite matrix
    maxVel = 800;

    %% --> Figures that are about to be shown <-- %%
    showControlMap = true;
    showVelocityMap = false;
    showConnectionMap = false;
    showErrorGraph = false;
    set(groot,'defaultFigureDeleteFcn',@endLoop);
    if showControlMap == true
        fmap = figure('color', [0.5 0.5 0.5], 'Name','Formation Map', 'Menubar', 'none', 'WindowButtonDownFcn', @reArrangeGoalFunction, 'Position',[0, 0, 700, 700]);
    end
    if showVelocityMap == true
        velmap = figure('color', [0.5 0.5 0.5], 'Name','Velocity Map', 'Menubar', 'none', 'Position',[700, 0, 500, 500]);
    end
    if showConnectionMap == true
        connectmap = figure('color', [0.5 0.5 0.5], 'Name','Connections number for each Agent', 'Menubar', 'none', 'Position',[700, 534, 500, 500]);
    end
    if showErrorGraph == true
        errorGraphs = figure('color', [0.5 0.5 0.5], 'Name','Error Graphs', 'Menubar', 'none', 'Position',[1200, 0, 700, 1000]);
        subplot(2,2,1)
        title('Distance Error');
        subplot(2,2,2)
        title('Ixx inertia moment Error');
        subplot(2,2,3)
        title('Ixy inertia moment Error');
        subplot(2,2,4)
        title('Iyy inertia moment Error');    
    end
    samplecount=1;
    
    
    Colors = rand(N,3);

    %% Main loop. (Where magic happens)
    step_counter = 0;
    keepLooping = true;
    while keepLooping
        %% update agents position. Happens in every time step 
    %     agentVel(sqrt(agentVel(:,1).^2+agentVel(:,2).^2)>200) = 200;
    %     agentPos = agentPos + agentVel * TS;
            for i=1:N
                if norm(agentVel(i,:)) > maxVel %check if agent velocity surpasses the speed limit
                    agentVel(i,:) = (agentVel(i,:) / norm(agentVel(i,:)) ) * maxVel; %%make agent's velocity the biggest allowed
                end
            end
            agentPos = agentPos + agentVel * TS; %update the position of the agents

        %% if time has come, draw
        if mod(step_counter, S2D) == 0
            %% agent formation figure
            if showControlMap == true
                figure(fmap);
                set(gca,'NextPlot','replacechildren') ;
                clf(fmap);   %clear previous illustrative data so that we can draw the new ones
                axes('xlim', [-2100 2100], 'ylim', [-2100 2100], ...
                  'XTick', -2000:500:2000, 'YTick', -2000:500:2000);
                grid on;
                hold on;
                scatter(agentPos(:,1),agentPos(:,2),[],Colors, 'filled');
                ellipse_given_Points(agentPos);
                ellipse_given_Mom(desiredForm);
                drawnow;
            end

            %% agent velocity figure
            if showVelocityMap == true
                figure(velmap);
                set(gca,'NextPlot','replacechildren') ;
                clf(velmap);   %clear previous illustrative data so that we can draw the new ones
                axes('xlim', [-1000 1000], 'ylim', [-1000 1000], ...
                 'XTick', -1000:200:1000, 'YTick', -1000:200:1000);
                hold on;
                for j=1:N
                    quiver(agentVel(j,1),agentVel(j,2),'color',Colors(j,:));
                end
                drawnow;
            end
            
            %% draw connections
            if showConnectionMap == true
                figure(connectmap);
                set(gca,'NextPlot','replacechildren') ;
                clf(connectmap);   %clear previous illustrative data so that we can draw the new ones
                axes('xlim', [0.5 N+0.5], 'ylim', [0 N-0.5], ...
                 'XTick', 1:N, 'YTick', 0:N);
                hold on;
                b = bar(1:N,sum(agentCanComm(1:N,:) )-1 );  %remove him self
                b.FaceColor = 'flat';
                b.CData(:,:) = Colors;
                drawnow;
            end
            
            %% draw Errors
            if showErrorGraph == true
                figure(errorGraphs);
                hold on;
                errors = zeros(N,5);
                for j=1:N
                    errors(j,:) = desiredForm - reshape(agentPackMom(j,j,:), [1 5]);
                    errors(j,2) = sqrt(errors(j,1)^2 + errors(j,2)^2);  %distance error
                end
                data2(samplecount,:)=errors(:,2);
                data3(samplecount,:)=errors(:,3);
                data4(samplecount,:)=errors(:,4);
                data5(samplecount,:)=errors(:,5);
                % Distance Error
                subplot(2,2,1)
                hold on
                for j=1:N
                    plot(1:samplecount,data2(:,j),'Color',Colors(j,:));
                end
                % Spread X error
                subplot(2,2,2)
                hold on
                for j=1:N
                    plot(1:samplecount,data3(:,j),'Color',Colors(j,:));
                end 
                % Moment XY error
                subplot(2,2,3)
                hold on 
                for j=1:N
                    plot(1:samplecount,data4(:,j),'Color',Colors(j,:));
                end
                % Spread y error
                subplot(2,2,4)
                hold on 
                for j=1:N
                    plot(1:samplecount,data5(:,j),'Color',Colors(j,:));
                end
            end
            
            samplecount = samplecount + 1;
        end

        %% if time has come, update agents
        for i=1:N
            if agentSteps2Update(i) == 0 %check if time has come for each agent
                %% transmit packet
                for j=1:N
                    if j==i
                        continue %don't send packet to itself 
                    end
                    dist = sqrt((agentPos(i,1)-agentPos(j,1))^2 + (agentPos(i,2)-agentPos(j,2))^2);
                    if (1-expcdf(dist,COM_const)+0.4*rand()-0.5*rand() > transmit_threshold ) %we model wifi success with an exponential function (with respect to distance)
                        %packet transmition happens
                        agentPackMom(j,i,:) = agentPackMom(i,i,:) ; 
                        agentPackIntern(j,i,:) = agentPackIntern(i,i,:) ; 
                        agentPackAge(j,i) = 0 ;
                    end
                    %forget packets which are old
                    if agentPackAge(i,j) >= S2DP
                        agentPackMom(i,j,:) = zeros(1,1,5);
                        agentPackIntern(i,j,:) = zeros(1,1,5);
                        agentPackAge(i,j) = 0; %has been dropped, so doesn't has an age.
                    end
                end
                %% prepare for estimation (compute with whom can agent i be able to communicate) 
                agentCanComm(i,:) = zeros(1,N);
                for j=1:N
                    temp = reshape(agentPackMom(i,j,:),[1 5]) == zeros(1,5);
                    for k= 1:5
                        if temp(k) == 0 %den einai palio paketo. Yparxei epikoinvnia
                           agentCanComm(i,j) = 1; 
                           continue;
                        end
                    end
                end

                %% estimation
                w = zeros(size(1,5)); %the estimated swarm sizes
                v = zeros(size(1,5));
                for j=1:N
                    w = w - reshape(agentCanComm(i,j)*Ki*(agentPackMom(i,i,:) - agentPackMom(i,j,:) ), [1 5]) ;
                    v = v - reshape(agentCanComm(i,j)*Kp*(agentPackMom(i,i,:)-agentPackMom(i,j,:)), [1 5]) ...
                        + reshape(agentCanComm(i,j)*Ki*(agentPackIntern(i,i,:)-agentPackIntern(i,j,:)), [1 5]) ;
                end
                v = v + gamma*(fi(agentPos(i,:))- reshape(agentPackMom(i,i,:), [1 5]) ) ;
                % add the swarm estimations from neighbours to the measures of the agent
                agentPackMom(i,i,:) = agentPackMom(i,i,:) + reshape(v*TS*S2U, [1 1 5]);   % (TS*S2U) is the average time that agents Update
                agentPackIntern(i,i,:) = agentPackIntern(i,i,:) + reshape(w*TS*S2U, [1 1 5]);

                %% control
                JacobianFi = [1 0 2*agentPos(i,1)-2*agentPackMom(i,i,1) agentPos(i,2)-agentPackMom(i,i,2) 0 ; ...
                    0 1 0 agentPos(i,1)-agentPackMom(i,i,1) 2*agentPos(i,2)-2*agentPackMom(i,i,2)];  %the partial derivatives
                vTrans = reshape(agentPackMom(i,i,:),[5 1]) - [0 ; 0 ; agentPackMom(i,i,1)^2 ; agentPackMom(i,i,1)*agentPackMom(i,i,2) ; agentPackMom(i,i,2)^2];
                controls = -JacobianFi*gaMatrix*(vTrans-desiredForm'); %desiredForm is the f* --> local goal for every agent
                agentVel(i,:) = controls;

                %restart clock
                agentSteps2Update(i) = floor(S2U + 2*randn()); %2*randn() implements randomness
            end
        end

        step_counter = step_counter + 1;
        agentSteps2Update = agentSteps2Update - 1;
        agentPackAge = agentPackAge + 1;
    end

    function endLoop(~, ~)
        keepLooping = 0;
    end

    function reArrangeGoalFunction(~, ~)
        goalPos = get(gca,'currentpoint');
        desiredForm = [goalPos(1,1) goalPos(1,2) 0.8*rand()*goalPos(1,1)^2 0.2*rand()*goalPos(1,1)*goalPos(1,2) 0.5*rand()*goalPos(1,2)^2];
    end

end


%% fi(position) --> moment-genarating function
function moments = fi(position)
    moments = [mean(position(:,1)) , mean(position(:,2)), mean(position(:,1).^2), mean(position(:,1).*position(:,2)), mean(position(:,2).^2)  ] ;
end


%% Draw ellipse 
function ellipse_given_Points(points)
    % ellipse center
    xc = mean(points(:,1));
    yc = mean(points(:,2));

    % recenter points in order to calculate second-order moments
    x = points(:,1) - xc;
    y = points(:,2) - yc;

    % number of points
    n = size(points, 1);

    % inertia parameters
    Ixx = sum(x.^2) / n;
    Iyy = sum(y.^2) / n;
    Ixy = sum(x.*y) / n;

    % compute ellipse semi-axis lengths
    common = sqrt( (Ixx - Iyy)^2 + 4 * Ixy^2);
    ra = sqrt(2) * sqrt(Ixx + Iyy + common);
    rb = sqrt(2) * sqrt(Ixx + Iyy - common);

    % compute ellipse angle in degrees
    theta = atan2(2 * Ixy, Ixx - Iyy) / 2;

    % create the resulting inertia ellipse
    draw_ellipse(ra,rb,theta,xc,yc,[0.5 0.7 0.2]);
end

function ellipse_given_Mom(mom)
    % compute ellipse semi-axis lengths
    common = sqrt( (mom(3) - mom(5))^2 + 4 * mom(4)^2);
    ra = sqrt(2) * sqrt(mom(4) + mom(5) + common);
    rb = sqrt(2) * sqrt(mom(3) + mom(5) - common);

    % compute ellipse angle in degrees
    theta = atan2(2 * mom(4), mom(3) - mom(5))/ 2;

    % create the resulting inertia ellipse
    draw_ellipse(ra,rb,theta,mom(1),mom(2),[0.5 0.5 0.5]);
end
