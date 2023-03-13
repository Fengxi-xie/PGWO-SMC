function PlotSolution_psoini(BestSol,model,param)
%%      ------------------绘图---------------------------%%

figure(1);
hold on;
plot(BestSol.BestCost,'LineWidth',2);
grid on;
hold on;


alpha_1 = model.alpha1_min + BestSol.Position(1)*(model.alpha1_max - model.alpha1_min);
alpha_2 = model.alpha2_min + BestSol.Position(2)*(model.alpha2_max - model.alpha2_min);
beta_1 = model.beta1_min + BestSol.Position(3)*(model.beta1_max - model.beta1_min);
beta_2 = model.beta2_min + BestSol.Position(4)*(model.beta2_max - model.beta2_min);
delta_s1 = model.delta1_min + BestSol.Position(5)*(model.delta1_max - model.delta1_min);
delta_s2 = model.delta2_min + BestSol.Position(6)*(model.delta2_max - model.delta2_min);
k1 = model.k1_min + BestSol.Position(7)*(model.k1_max - model.k1_min);
k2 = model.k2_min + BestSol.Position(8)*(model.k2_max - model.k2_min);
k3 = model.k3_min + BestSol.Position(9)*(model.k3_max - model.k3_min);
k4 = model.k4_min + BestSol.Position(10)*(model.k4_max - model.k4_min);
k5 = model.k5_min + BestSol.Position(11)*(model.k5_max - model.k5_min);
k6 = model.k6_min + BestSol.Position(12)*(model.k6_max - model.k6_min);


total_time = 120;
step_count = 0.05;
t = 0:step_count:total_time;

%%      ------------------初始化---------------------------%%
x = zeros(total_time/step_count+1, 1);
y = zeros(total_time/step_count+1, 1);
x_d = zeros(total_time/step_count+1, 1);
y_d = zeros(total_time/step_count+1, 1);
psi = zeros(total_time/step_count+1, 1);
w = zeros(total_time/step_count+1, 1);
v = zeros(total_time/step_count+1, 1);
data_close = ones(total_time/step_count+1, 1);
error_yaw1 = zeros(total_time/step_count+1, 1);
error_yaw2 = zeros(total_time/step_count+1, 1);
delta = zeros(total_time/step_count+1, 1);
psi_d = zeros(total_time/step_count+1, 1);
d_psi_d = zeros(total_time/step_count+1, 1);
dd_psi_d = zeros(total_time/step_count+1, 1);
dd_v_d = zeros(total_time/step_count+1, 1);
error_v1 = zeros(total_time/step_count+1, 1);
error_v2 = zeros(total_time/step_count+1, 1);
F_d = zeros(total_time/step_count+1, 1);
error_location = zeros(total_time/step_count+1, 1);
%%  ---- 初始状态设置---------%%
x(1) = 2;
y(1) = 0;
psi(1) = pi/2;
w(1) = 0;
v(1) = 0;
%%  ----------控制参数调节-------%%

kappa1 = 0.5;
kappa2 = 0.3;

%%    --------------     模型参数    --------------------    %%
l = 2.85;%轴距
vr = 20;%期望速度
%%    --------------     定步长仿真开始    --------------------    %%
[x_d,y_d] = path();
for i = 1:(total_time/step_count)
    
    %%    --------------     期望航向    --------------------    %%
    k = length(x_d);
    while k >=1
        if sqrt((x(i)-x_d(k))^2+(y(i)-y_d(k))^2) < sqrt((x(i)-x_d(data_close(i)))^2+(y(i)-y_d(data_close(i)))^2)
            data_close(i) = k;
        end
        k = k-1;
    end
    
    if data_close(i)<= 2
        D = [x_d(data_close(i)) y_d(data_close(i))] - [x(i) y(i)];
        G = 2/pi*atan(kappa1*norm(D));
        H = sqrt(1-G^2);
        T1 = -D(2)/(D(1)+1e-15)*1;
        T2 = 1;
        temp = sqrt(T1^2 + T2^2);
        T1 = T1/temp;
        T2 = T2/temp;
        T = [T1 T2];
    elseif  data_close(i) >= length(x_d) -2
        D = [x_d(data_close(i)) y_d(data_close(i))] - [x(i) y(i)];
        G = 2/pi*atan(kappa1*norm(D));
        H = sqrt(1-G^2);
        T1 = D(2)/(D(1)+1e-15)*1;
        T2 = 1;
        temp = sqrt(T1^2 + T2^2);
        T1 = T1/temp;
        T2 = T2/temp;
        T = [T1 T2];
    else
        D = (1-kappa2)^3*[x_d(data_close(i)-1) y_d(data_close(i)-1)] + 3*kappa2*(1-kappa2)^2*[x_d(data_close(i)) y_d(data_close(i))] ...
            +3*kappa2^2*(1-kappa2)*[x_d(data_close(i)+1) y_d(data_close(i)+1)] + kappa2^3*[x_d(data_close(i)+2) y_d(data_close(i)+2)]- [x(i) y(i)];
        G = 2/pi*atan(kappa1*norm(D));
        H = sqrt(1-G^2);
        temp_norm = ceil(norm(D)/50);
        T = (1-kappa2)^3*([x_d(data_close(i)-temp_norm) y_d(data_close(i)-temp_norm)] - [x_d(data_close(i)-2*temp_norm) y_d(data_close(i)-2*temp_norm)])/norm(([x_d(data_close(i)-1*temp_norm) y_d(data_close(i)-1*temp_norm)] - [x_d(data_close(i)-2*temp_norm) y_d(data_close(i)-2*temp_norm)]))...
            +3*kappa2*(1-kappa2)^2*([x_d(data_close(i)) y_d(data_close(i))] - [x_d(data_close(i)-1*temp_norm) y_d(data_close(i)-1*temp_norm)])/norm(([x_d(data_close(i)) y_d(data_close(i))] - [x_d(data_close(i)-1*temp_norm) y_d(data_close(i)-1*temp_norm)]))...
            +3*kappa2^2*(1-kappa2)*([x_d(data_close(i)+temp_norm) y_d(data_close(i)+temp_norm)] - [x_d(data_close(i)) y_d(data_close(i))])/norm(([x_d(data_close(i)+1*temp_norm) y_d(data_close(i)+1*temp_norm)] - [x_d(data_close(i)) y_d(data_close(i))]))...
            +kappa2^3*([x_d(data_close(i)+2*temp_norm) y_d(data_close(i)+2*temp_norm)] - [x_d(data_close(i)+1*temp_norm) y_d(data_close(i)+1*temp_norm)])/norm(([x_d(data_close(i)+2*temp_norm) y_d(data_close(i)+2*temp_norm)] - [x_d(data_close(i)+1*temp_norm) y_d(data_close(i)+1*temp_norm)]));
    end
    
    psi_temp= H*T+G*D/(norm(D)+0.000000000000000000000000001);
    psi_temp = psi_temp/norm(psi_temp);
    
    psi_d(i) = atan2(psi_temp(2),psi_temp(1));
    
    if i>=2
        d_psi_d(i) = (psi_d(i) - psi_d(i-1))/step_count;
    else
        d_psi_d(i) = 0;
    end

    if i>=3
        dd_psi_d(i) = GetLimitValue((d_psi_d(i) - d_psi_d(i-1))/step_count,3.14,-3.14);
    else
        dd_psi_d(i) = 0;
    end


    %%    --------------     角度控制    --------------------    %%
    temp1 = mod(psi(i),2*pi);%以下几行是为了精准计算转向角误差
    temp2 = mod(psi_d(i),2*pi);
    if temp1-temp2 > pi
        temp1 = temp1 - 2*pi; 
    elseif temp1-temp2 < - pi
        temp2 = temp2 - 2*pi; 
    end

    error_yaw1(i) = temp1 - temp2;%定义转向角误差
    if i>=2
        d_error_yaw1 = (error_yaw1(i) - error_yaw1(i-1))/step_count;
    else
        d_error_yaw1 = 0;
    end
    error_yaw2(i) = k1*error_yaw1(i);
    
    %滑模面
    s1 = k2*error_yaw1(i) + error_yaw2(i);

    %分段函数
    if abs(s1) >= delta_s1
        f_1 = abs(s1)*(abs(s1) - delta_s1);
    else
        f_1 = -(1-abs(s1))/delta_s1/(abs(s1) + delta_s1);
    end

    w(i) =  - (1/(2*k2) + exp(0)*(abs(s1))^beta_1)*s1 - k2*(error_yaw2(i) - k1*error_yaw1(i));


    %%    --------------     速度控制    --------------------    %%
    dd_v_d(i) = 0;
    error_v1(i) = v(i) - vr;%定义速度误差
    if i>=2
        d_error_v1 = (error_v1(i) - error_v1(i-1))/step_count;
    else
        d_error_v1 = 0;
    end
    error_v2(i) = k1*error_v1(i);

    %滑模面
    s2 = k2*error_v1(i) + error_v2(i);

    %分段函数
    if abs(s2) >= delta_s2
        f_2 = abs(s2)*(abs(s2) - delta_s2);
    else
        f_2 = -(1-abs(s2))/delta_s2/(abs(s2) + delta_s2);
    end

    F_d(i) = - k6*(abs(s2))^alpha_2*sign(s2) - (1/(2*k5) + exp(0)*(abs(s2))^beta_2)*s2 - k5*(error_v2(i) - k4*error_v1(i))+ dd_v_d(i) - k4*d_error_v1;
    
    v(i+1) = v(i) + F_d(i)*step_count;
    
     %%    --------------     模型计算    --------------------    %%
    temp1 = mod(psi(i),2*pi);%以下几行是为了精准计算转向角误差
    temp2 = mod(psi_d(i),2*pi);
    if temp1-temp2 > pi
        temp1 = temp1 - 2*pi; 
    elseif temp1-temp2 < - pi
        temp2 = temp2 - 2*pi; 
    end

    error_yaw1(i) = temp1 - temp2;%定义转向角误差
    if i>=2
        d_error_yaw1 = (error_yaw1(i) - error_yaw1(i-1))/step_count;
    else
        d_error_yaw1 = 0;
    end
    

    delta(i) = atan(w(i)*l/vr);
   
    R  = [cos(psi(i)) 0;sin(psi(i)) 0;0 1];
    
    temp = R*[v(i) w(i)]';
    
    %%    --------------     状态量更新    --------------------    %%
    x(i+1) = x(i) + temp(1)*step_count;
    y(i+1) = y(i) + temp(2)*step_count;
    psi(i+1) = mod(psi(i) + temp(3)*step_count,2*pi);
    
    %%    --------------     计算误差量    --------------------    %%
    error_location(i) = abs(D(2));
end

delta(i+1) = delta(i);
k_p = 6*10^(0);
PIDfigure;
error_yaw1(i+1) = error_yaw1(i);
x_d(i+1) = x_d(i);
y_d(i+1) = y_d(i);
delta= mod(delta,2*pi);

figure(2)
plot(x,y,'linewidth',1.5);
hold on;
axis equal;
hold on;

figure(6)
plot(t,error_location,'linewidth',1.5);
ylim([-0.1 6]);

hold on;



