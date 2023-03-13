clear all;
clc;

%%    --------------     期望路径    --------------------    %%
omega_n = 0.1;
zeta = 0.8;

total_time = 100;
step_count = 0.01;
temp_t2 = 0;
x1_filter_x = zeros(total_time/(step_count/10)+1, 1);
x2_filter_x = zeros(total_time/(step_count/10)+1, 1);
x1_filter_y = zeros(total_time/(step_count/10)+1, 1);
x2_filter_y = zeros(total_time/(step_count/10)+1, 1);
for i = 1:(total_time/(step_count/10))
    total_time0 = 100;
    t1 = 0:(step_count/10):(total_time0);
    if t1(i)<total_time0/6
        x_d(i) = 0;
        y_d(i) = 20*t1(i);
    elseif t1(i) >= total_time0/6 && t1(i) < total_time0/3
        temp_t2 = temp_t2 + step_count/10;
        x_d(i) = 120*sin(0.12*temp_t2*pi);
        y_d(i) = 20*t1(i);
    elseif t1(i) >= total_time0/3 && t1(i) < total_time0/2
        x_d(i) = 0;
        y_d(i) = 20*t1(i);
    elseif t1(i) >= total_time0/2 && t1(i) < total_time0*2/3
        x_d(i) = 20*(t1(i)-total_time0/2);
        y_d(i) = y_d(i-1);

    else
        x_d(i) = x_d(i-1);
        y_d(i) = y_d(i-1) - 30*(step_count/10);
    end

    dx1_filter_x = x2_filter_x(i);
    dx2_filter_x = -2 * zeta * omega_n * x2_filter_x(i) - omega_n^2* (x1_filter_x(i) - x_d(i));
    dx1_filter_y = x2_filter_y(i);
    dx2_filter_y = -2 * zeta * omega_n * x2_filter_y(i) - omega_n^2* (x1_filter_y(i) - y_d(i));

    x1_filter_x(i+1) = x1_filter_x(i) + dx1_filter_x*step_count;
    x2_filter_x(i+1) = x2_filter_x(i) + dx2_filter_x*step_count;
    x1_filter_y(i+1) = x1_filter_y(i) + dx1_filter_y*step_count;
    x2_filter_y(i+1) = x2_filter_y(i) + dx2_filter_y*step_count;
end

x_d = x1_filter_x;
y_d = x1_filter_y;

total_time = 120;
step_count = 0.01;
t = 0:step_count:total_time;

%%      ------------------初始化---------------------------%%
x = zeros(total_time/step_count+1, 1);
y = zeros(total_time/step_count+1, 1);
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
v(1) = 0.1;
%%  ----------控制参数调节-------%%
kappa1 = 0.5;
kappa2 = 0.3;

alpha_1 = 0.5342;
alpha_2 = 0.0810;
beta_1 = 0.0269;
beta_2 = 0.1175;
delta_s1 = 0.1645;
delta_s2 = 0.1327;
k1 = 0.0040*5;
k2 = 0.2640*5;
k3 = 7.4298*5;
k4 = 0.0015*5;
k5 = 3.1339*5;
k6 = 1.9640*5;

%%    --------------     模型参数    --------------------    %%
l = 3;%轴距
vr = 20;%期望速度
%%    --------------     定步长仿真开始    --------------------    %%
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

error_yaw1(i+1) = error_yaw1(i);
x_d(i+1) = x_d(i);
y_d(i+1) = y_d(i);
error_location(i+1) = error_location(i);

figure(2)
plot(x,y,'linewidth',1.5);
hold on;

figure(3)
plot(t,delta/15,'linewidth',1.5);
hold on;

figure(4)
plot(t,error_yaw1,'linewidth',1.5);
hold on;

figure(5)
plot(t,psi,'linewidth',1.5);
hold on;

figure(6)
plot(t,error_location,'linewidth',1.5);
ylim([-0.1 6]);
hold on;

