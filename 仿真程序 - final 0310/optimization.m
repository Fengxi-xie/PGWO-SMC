close all;
clc;
clear all;

%%      ------------------算法参数设置---------------------------%%
model.nVar = 12; %编码长度

%参数取值范围
model.alpha1_min = 0;
model.alpha1_max = 1;
model.beta1_min = 0;
model.beta1_max = 1;
model.alpha2_min = 0;   
model.alpha2_max = 1;
model.beta2_min = 0;
model.beta2_max = 1;
model.delta1_min = 0;
model.delta1_max = 1;
model.delta2_min = 0;
model.delta2_max = 1;
model.k1_min = 0;
model.k1_max = 5;
model.k2_min = 0;
model.k2_max = 5;
model.k3_min = 0;
model.k3_max = 50;
model.k4_min = 0;
model.k4_max = 5;
model.k5_min = 0;
model.k5_max = 50;
model.k6_min = 0;
model.k6_max = 50;

%参数取值
model.a = 10000;
model.b = 0.1;
model.c = 0.1;
model.d = 0.01;
model.time = 120;%每次控制仿真时间

param.MaxIt = 20;%迭代次数
param.nPop = 2;%种群数目

param.wmax = 0.8;
param.wmin = 0.5;
param.c1max = 2;
param.c1min = 0.1;
param.c2max = 2;
param.c2min = 0.1;

param.ShowIteration = 200;%每过多少次迭代显示一次图
param.index = 1;
%%      ------------------运行算法---------------------------%%
CostFunction= @(x) MyCost(x,model);%设置目标函数
[GlobalBest, BestCost] = Wolf(param, model, CostFunction);
[GlobalBest1, BestCost1] = pso(param, model, CostFunction);
