function [ GlobalBest  , BestCost ] =  Wolf( param   , model , CostFunction )

nVar = model.nVar ;   %  节点个数
VarSize=[1 nVar];   %  决策变量维度

VarMin = 0 ; % 自变量取值   低阶
VarMax= 1 ;% 自变量取值   高阶
%% PSO Parameters
MaxIt=  param.MaxIt ;        %迭代次数
nPop= param.nPop ;           %种群数目
param.index = 1;

%%  初始化
rand('seed', sum(clock));

empty_wolf.Position=[];
empty_wolf.Cost=[];
empty_wolf.Sol=[];
empty_wolf.Best.Position=[];
empty_wolf.Best.Cost=[];
empty_wolf.Best.Sol=[];


GlobalBest.Cost=inf;


wolf=repmat(empty_wolf,nPop,1);


for i=1:nPop
    
  
    wolf(i).Position = [0.5342 0.0810 0.0269 0.1175 0.1645 0.1327 0.004*1.5 0.264*1.5 0.7429*1.5 0.0015*1.5 0.31339*1.5 0.1964*1.5];

    [wolf(i).Cost, wolf(i).Sol]=CostFunction(wolf(i).Position);
    

    wolf(i).Best.Position=wolf(i).Position;
    wolf(i).Best.Cost=wolf(i).Cost;
    wolf(i).Best.Sol=wolf(i).Sol;
    
 
    if wolf(i).Best.Cost < GlobalBest.Cost
        
        GlobalBest = wolf(i).Best;
        
    end
    
end

% 最优解记录
BestCost=zeros(MaxIt,1);

%% PSO Main Loop

%%
    
a = 0.1;
for it=1:MaxIt
    for i=1:nPop
        
        %根据头狼和护法的位置计算最优位置
        temp = (GlobalBest.Position + wolf(i).Best.Position)/2;
        
        %参考链接  https://www.jianshu.com/p/97206c3fc51f
        
        
        % Update Position
        for k = 1:1:length(wolf(i).Position)
            wolf(i).Position(k) = temp(k) - (rand(1)*2*a-a)*(rand(1)*2*temp(k) -  wolf(i).Position(k));
        end
        
        
        % Update Position Bounds
        wolf(i).Position = max(wolf(i).Position,VarMin);
        wolf(i).Position = min(wolf(i).Position,VarMax);
        
        
        % Evaluation
        [wolf(i).Cost, wolf(i).Sol]=CostFunction(wolf(i).Position);
        
        % Update Personal Best
        if wolf(i).Cost<wolf(i).Best.Cost
            
            wolf(i).Best.Position=wolf(i).Position;
            wolf(i).Best.Cost=wolf(i).Cost;
            wolf(i).Best.Sol=wolf(i).Sol;
            
            % Update Global Best
            if wolf(i).Best.Cost<GlobalBest.Cost
                GlobalBest=wolf(i).Best;
            end
            
        end
    end
    %%
    
    sol =GlobalBest ;
    %%
    
    % 更新最优解
    BestCost(it)=GlobalBest.Cost;
  
    
    disp(it);
    if isequal( it , MaxIt )
        gcf  = figure(1);
        % Plot Solution
        set(gcf, 'unit' ,'centimeters' ,'position',[5 2  25 15 ]);
        
        GlobalBest.BestCost = BestCost(1:it)  ;
        GlobalBest.MaxIt = MaxIt ;
        
        PlotSolution_Wolf(GlobalBest ,model,param)
        pause( 0.01 );
    end
    
end


