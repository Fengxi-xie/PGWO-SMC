

global InteTotal_1 In_last_1

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
vr = zeros(total_time/step_count+1, 1);
data_close = ones(total_time/step_count+1, 1);
error_yaw1 = zeros(total_time/step_count+1, 1);
error_yaw2 = zeros(total_time/step_count+1, 1);
delta = zeros(total_time/step_count+1, 1);
error_location = zeros(total_time/step_count+1, 1);
%%  ---- 初始状态设置---------%%
x(1) = 2;
y(1) = 0;
psi(1) = pi/2;
w(1) = 0;
vr(1) = 0;

InteTotal_1 = 0; 
In_last_1 = 0;

%%  ----------控制参数调节-------%%
if param.index == 1
    k_p = 15*10^(0);
else
    k_p = 5*10^(0);
end
k_i = 1*10^(-3);
k_d = 1*10^(1);

mu = 0.01;
theta = 1;
eta = 1;
alpha = 0.1;
gamma = 1;

kf = 0.2;
tera = 0.1;

%%    --------------     模型参数    --------------------    %%
l = 2.85;%轴距
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
        G = 2/pi*atan(kf*norm(D));
        H = sqrt(1-G^2);
        T1 = -D(2)/(D(1)+1e-15)*1;
        T2 = 1;
        temp = sqrt(T1^2 + T2^2);
        T1 = T1/temp;
        T2 = T2/temp;
        T = [T1 T2];
    elseif  data_close(i) >= length(x_d) -2
        D = [x_d(data_close(i)) y_d(data_close(i))] - [x(i) y(i)];
        G = 2/pi*atan(kf*norm(D));
        H = sqrt(1-G^2);
        T1 = D(2)/(D(1)+1e-15)*1;
        T2 = 1;
        temp = sqrt(T1^2 + T2^2);
        T1 = T1/temp;
        T2 = T2/temp;
        T = [T1 T2];
    else
        D = (1-tera)^3*[x_d(data_close(i)-1) y_d(data_close(i)-1)] + 3*tera*(1-tera)^2*[x_d(data_close(i)) y_d(data_close(i))] ...
            +3*tera^2*(1-tera)*[x_d(data_close(i)+1) y_d(data_close(i)+1)] + tera^3*[x_d(data_close(i)+2) y_d(data_close(i)+2)]- [x(i) y(i)];
        G = 2/pi*atan(kf*norm(D));
        H = sqrt(1-G^2);
        temp_norm = ceil(norm(D)/50);
        T = (1-tera)^3*([x_d(data_close(i)-temp_norm) y_d(data_close(i)-temp_norm)] - [x_d(data_close(i)-2*temp_norm) y_d(data_close(i)-2*temp_norm)])/norm(([x_d(data_close(i)-1*temp_norm) y_d(data_close(i)-1*temp_norm)] - [x_d(data_close(i)-2*temp_norm) y_d(data_close(i)-2*temp_norm)]))...
            +3*tera*(1-tera)^2*([x_d(data_close(i)) y_d(data_close(i))] - [x_d(data_close(i)-1*temp_norm) y_d(data_close(i)-1*temp_norm)])/norm(([x_d(data_close(i)) y_d(data_close(i))] - [x_d(data_close(i)-1*temp_norm) y_d(data_close(i)-1*temp_norm)]))...
            +3*tera^2*(1-tera)*([x_d(data_close(i)+temp_norm) y_d(data_close(i)+temp_norm)] - [x_d(data_close(i)) y_d(data_close(i))])/norm(([x_d(data_close(i)+1*temp_norm) y_d(data_close(i)+1*temp_norm)] - [x_d(data_close(i)) y_d(data_close(i))]))...
            +tera^3*([x_d(data_close(i)+2*temp_norm) y_d(data_close(i)+2*temp_norm)] - [x_d(data_close(i)+1*temp_norm) y_d(data_close(i)+1*temp_norm)])/norm(([x_d(data_close(i)+2*temp_norm) y_d(data_close(i)+2*temp_norm)] - [x_d(data_close(i)+1*temp_norm) y_d(data_close(i)+1*temp_norm)]));
    end
    
    psi_temp= H*T+G*D/(norm(D)+0.000000000000000000000000001);
    psi_temp = psi_temp/norm(psi_temp);
    
    psi_d(i) = atan2(psi_temp(2),psi_temp(1));

    %%    --------------     模型计算    --------------------    %%
    vr(i) = 20;%速度
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
    
    
    w(i) = PID_1(k_p,k_i,k_d,error_yaw1(i));%偏角控制器

    delta(i) = atan(w(i)*l/vr(i));
   
    R  = [cos(psi(i)) 0;sin(psi(i)) 0;0 1];
    
    temp = R*[vr(i) w(i)]';
    
    %%    --------------     状态量更新    --------------------    %%
    x(i+1) = x(i) + temp(1)*step_count;
    y(i+1) = y(i) + temp(2)*step_count;
    psi(i+1) = psi(i) + temp(3)*step_count;
    %%    --------------     计算误差量    --------------------    %%
    error_location(i) = abs(D(2));
end

delta(i+1) = delta(i);
error_yaw1(i+1) = error_yaw1(i);
x_d(i+1) = x_d(i);
y_d(i+1) = y_d(i);
error_location(i+1) = error_location(i);
for i = 1:1:(total_time/step_count+1)
if psi(i)>pi
    psi(i) = psi(i)-2*pi;
end
end

figure(3)
plot(t,(delta),'linewidth',1.5);
hold on;

figure(4)
plot(t,(error_yaw1),'linewidth',1.5);
hold on;

figure(5)
plot(t,(psi),'linewidth',1.5);
hold on;



disp('粒子群平均位置误差');
disp(mean(error_location));


disp('粒子群最大位置误差');
disp(max(error_location));