function [ GlobalBest  , BestCost ] =  pso( param   , model , CostFunction )

nVar = model.nVar ;       %  节点个数
VarSize=[1 nVar];   %  决策变量维度

VarMin = 0 ; % 自变量取值   低阶
VarMax= 1 ;% 自变量取值   高阶
%% PSO Parameters
MaxIt=  param.MaxIt ;          %    迭代次数
nPop= param.nPop ;           %  种群数目
param.index = 2;
VelMax =  0.01* (VarMax - VarMin );    %   X  最大速度
VelMin =  -VelMax ;                    %    X 最小速度

%%  初始化
rand('seed', sum(clock));

empty_particle.Position=[];
empty_particle.Velocity=[];
empty_particle.Cost=[];
empty_particle.Sol=[];
empty_particle.Best.Position=[];
empty_particle.Best.Cost=[];
empty_particle.Best.Sol=[];


GlobalBest.Cost=inf;


particle=repmat(empty_particle,nPop,1);


for i=1:nPop
    
    particle(i).Position = [0.5342 0.0810 0.0269 0.1175 0.1645 0.1327 0.004*1.5 0.264*1.5 0.7429*1.5 0.0015*1.5 0.31339*1.5 0.1964*1.5];

    particle(i).Velocity = unifrnd(VelMin, VelMax, VarSize);

    [particle(i).Cost, particle(i).Sol]=CostFunction(particle(i).Position);
    

    particle(i).Best.Position=particle(i).Position;
    particle(i).Best.Cost=particle(i).Cost;
    particle(i).Best.Sol=particle(i).Sol;
    
 
    if particle(i).Best.Cost < GlobalBest.Cost
        
        GlobalBest = particle(i).Best;
        
    end
    
end

% 最优解记录
BestCost=zeros(MaxIt,1);

%% PSO Main Loop

%%
    
realmin=exp(-10*5);
C1 = exp( ( particle(i).Best.Position -  GlobalBest.Position)/ abs( GlobalBest.Position + realmin ) );
C2 = exp( ( -particle(i).Best.Position + GlobalBest.Position));
for it=1:MaxIt
    for i=1:nPop
         
        if it <= MaxIt/3
        	w = param.wmax;
            C1 = param.c1max;
            C2 = param.c1max;
        elseif it <= 2*MaxIt/3
            mu = 0.3+(0.6-0.3)*rand(1);
            w = mu + randn(1);
            mu = 0.1+(1.5-0.1)*rand(1);
            c1 = mu + randn(1);
            mu = 0.1+(1.5-0.1)*rand(1);
            c2 = mu + randn(1);
        else
            w = param.wmin;
            C1 = param.c1min;
            C2 = param.c1min;
        end
        
        % Update Velocity
        particle(i).Velocity  = w*particle(i).Velocity ...
            + C1*rand(VarSize).*(particle(i).Best.Position-particle(i).Position) ...
            + C2.*rand(VarSize).*(GlobalBest.Position-particle(i).Position);
        
        % Update Velocity Bounds
        particle(i).Velocity = max(particle(i).Velocity,VelMin);
        particle(i).Velocity = min(particle(i).Velocity,VelMax);
        
        % Update Position
        particle(i).Position = particle(i).Position + particle(i).Velocity ;
        
        
        % Update Position Bounds
        particle(i).Position = max(particle(i).Position,VarMin);
        particle(i).Position = min(particle(i).Position,VarMax);
        
        
        % Evaluation
        [particle(i).Cost, particle(i).Sol]=CostFunction(particle(i).Position);
        
        % Update Personal Best
        if particle(i).Cost<particle(i).Best.Cost
            
            particle(i).Best.Position=particle(i).Position;
            particle(i).Best.Cost=particle(i).Cost;
            particle(i).Best.Sol=particle(i).Sol;
            
            % Update Global Best
            if particle(i).Best.Cost < GlobalBest.Cost
                GlobalBest=particle(i).Best;
            end
            
        end
    end
    for i=1:nPop
    
  
    newparticle(i).Position = [0.5342 0.0810 0.0269 0.1175 0.1645 0.1327 0.004*1.5 0.264*1.5 0.7429*1.5 0.0015*1.5 0.31339*1.5 0.1964*1.5];

    newparticle(i).Velocity =  VelMin + (VelMax-VelMin)*rand(1,VarSize(2));

    [newparticle(i).Cost, newparticle(i).Sol]=CostFunction(newparticle(i).Position);
    

    newparticle(i).Best.Position=newparticle(i).Position;
    newparticle(i).Best.Cost=newparticle(i).Cost;
    newparticle(i).Best.Sol=newparticle(i).Sol;
    
 
    if newparticle(i).Best.Cost < GlobalBest.Cost
        
        GlobalBest = newparticle(i).Best;
        
    end
    
    end


    %%
    
    % 更新最优解
    BestCost(it)=GlobalBest.Cost;
    
    
    disp(it);
    
%     if ~mod( it, param.ShowIteration ) | isequal( it , MaxIt )
    if isequal( it , MaxIt )
        gcf  = figure(1);
        % Plot Solution
        set(gcf, 'unit' ,'centimeters' ,'position',[5 2  25 15 ]);
        
        GlobalBest.BestCost = BestCost(1:it)  ;
        GlobalBest.MaxIt = MaxIt ;
        
        PlotSolution(GlobalBest ,model,param)
        pause( 0.01 );
    end
    
end


