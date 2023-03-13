function [ GlobalBest  , BestCost ] =  pso_ini( param   , model , CostFunction )

nVar = model.nVar ;       %  节点个数
VarSize=[1 nVar];   %  决策变量维度

VarMin = 0 ; % 自变量取值   低阶
VarMax= 1 ;% 自变量取值   高阶
%% PSO Parameters
MaxIt=  param.MaxIt ;          %    迭代次数
nPop= param.nPop ;           %  种群数目
w= param.w ;                %   权重
wdamp=param.wdamp ;         %    退化率


VelMax =  0.1* (VarMax - VarMin );    %   X  最大  速度
VelMin    =  -VelMax ;                    %    X 最小速度

  

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
    
  
    particle(i).Position =  [0.5342 0.0810 0.0269 0.1175 0.1645 0.1327 0.004*0.5 0.264*0.5 0.7429*0.5 0.0015*0.5 0.31339*0.5 0.1964*0.5];

    particle(i).Velocity  =    unifrnd(      VelMin    ,  VelMax  ,  VarSize  ) ;

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
    
 realmin=exp(-10*50);
for it=1:MaxIt
    for i=1:nPop
         
        % Update Velocity
        particle(i).Velocity  = w*particle(i).Velocity ...
            + rand(VarSize).*(particle(i).Best.Position-particle(i).Position);
        
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
            if particle(i).Best.Cost<GlobalBest.Cost
                GlobalBest=particle(i).Best;
            end
            
        end
    end
    %%
    

    %%
    
    % 更新最优解
    BestCost(it)=GlobalBest.Cost;
    
    %权重更新
    w=w*wdamp;

    if isequal( it , MaxIt )
        gcf  = figure(1);
        % Plot Solution
        set(gcf, 'unit' ,'centimeters' ,'position',[5 2  25 15 ]);
        
        GlobalBest.BestCost = BestCost(1:it)  ;
        GlobalBest.MaxIt = MaxIt ;
        
        PlotSolution_psoini(GlobalBest ,model)
        pause( 0.01 );
    end
    
end


