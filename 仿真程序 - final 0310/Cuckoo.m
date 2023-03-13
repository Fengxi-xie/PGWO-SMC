function [ GlobalBest  , BestCost ] =  Cuckoo( param   , model , CostFunction )

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

empty_cuckoo.Position=[];
empty_cuckoo.Cost=[];
empty_cuckoo.Sol=[];
empty_cuckoo.Best.Position=[];
empty_cuckoo.Best.Cost=[];
empty_cuckoo.Best.Sol=[];


GlobalBest.Cost=inf;


cuckoo=repmat(empty_cuckoo,nPop,1);


for i=1:nPop
    
  
    cuckoo(i).Position = [0.5342 0.0810 0.0269 0.1175 0.1645 0.1327 0.004*0.5 0.264*0.5 0.7429*0.5 0.0015*0.5 0.31339*0.5 0.1964*0.5];

    [cuckoo(i).Cost, cuckoo(i).Sol]=CostFunction(cuckoo(i).Position);
    

    cuckoo(i).Best.Position=cuckoo(i).Position;
    cuckoo(i).Best.Cost=cuckoo(i).Cost;
    cuckoo(i).Best.Sol=cuckoo(i).Sol;
    
 
    if cuckoo(i).Best.Cost < GlobalBest.Cost
        
        GlobalBest = cuckoo(i).Best;
        
    end
    
end

% 最优解记录
BestCost=zeros(MaxIt,1);

%% PSO Main Loop

%%
    
a = 0.1;
for it=1:MaxIt
    for i=1:nPop
        
        
 
        % Update Position
        for k = 1:1:length(cuckoo(i).Position)
            cuckoo(i).Position(k) = cuckoo(i).Position(k) + levy()*(rand(1)*GlobalBest.Position(k) - cuckoo(i).Position(k));
        end
        
        % Update Position Bounds
        cuckoo(i).Position = max(cuckoo(i).Position,VarMin);
        cuckoo(i).Position = min(cuckoo(i).Position,VarMax);
        
        
        % Evaluation
        [cuckoo(i).Cost, cuckoo(i).Sol]=CostFunction(cuckoo(i).Position);
        
        % Update Personal Best
        if cuckoo(i).Cost<cuckoo(i).Best.Cost
            
            cuckoo(i).Best.Position=cuckoo(i).Position;
            cuckoo(i).Best.Cost=cuckoo(i).Cost;
            cuckoo(i).Best.Sol=cuckoo(i).Sol;
            
            % Update Global Best
            if cuckoo(i).Best.Cost<GlobalBest.Cost
                GlobalBest=cuckoo(i).Best;
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
        
        GlobalBest.BestCost = BestCost(1:it);
        GlobalBest.MaxIt = MaxIt;
        
        PlotSolution_cuckoo(GlobalBest ,model,param)
        pause( 0.01 );
    end
    
end


