function [x_d,y_d] = path()
%%    --------------     期望路径    --------------------    %%
% omega_n = 0.1;
% zeta = 0.8;
% 
% total_time = 100;
% step_count = 0.01;
% temp_t2 = 0;
% x1_filter_x = zeros(total_time/(step_count/10)+1, 1);
% x2_filter_x = zeros(total_time/(step_count/10)+1, 1);
% x1_filter_y = zeros(total_time/(step_count/10)+1, 1);
% x2_filter_y = zeros(total_time/(step_count/10)+1, 1);
% for i = 1:(total_time/(step_count/10))
%     total_time0 = 100;
%     t1 = 0:(step_count/10):(total_time0);
%     if t1(i)<total_time0/6
%         x_d(i) = 0;
%         y_d(i) = 20*t1(i);
%     elseif t1(i) >= total_time0/6 && t1(i) < total_time0/3
%         temp_t2 = temp_t2 + step_count/10;
%         x_d(i) = 120*sin(0.12*temp_t2*pi);
%         y_d(i) = 20*t1(i);
%     elseif t1(i) >= total_time0/3 && t1(i) < total_time0/2
%         x_d(i) = 0;
%         y_d(i) = 20*t1(i);
%     elseif t1(i) >= total_time0/2 && t1(i) < total_time0*2/3
%         x_d(i) = 20*(t1(i)-total_time0/2);
%         y_d(i) = y_d(i-1);
% 
%     else
%         x_d(i) = x_d(i-1);
%         y_d(i) = y_d(i-1) - 30*(step_count/10);
%     end
% 
%     dx1_filter_x = x2_filter_x(i);
%     dx2_filter_x = -2 * zeta * omega_n * x2_filter_x(i) - omega_n^2* (x1_filter_x(i) - x_d(i));
%     dx1_filter_y = x2_filter_y(i);
%     dx2_filter_y = -2 * zeta * omega_n * x2_filter_y(i) - omega_n^2* (x1_filter_y(i) - y_d(i));
% 
%     x1_filter_x(i+1) = x1_filter_x(i) + dx1_filter_x*step_count;
%     x2_filter_x(i+1) = x2_filter_x(i) + dx2_filter_x*step_count;
%     x1_filter_y(i+1) = x1_filter_y(i) + dx1_filter_y*step_count;
%     x2_filter_y(i+1) = x2_filter_y(i) + dx2_filter_y*step_count;
% end
temp1 = load('x_d.mat');
temp2 = load('y_d.mat');

x_d = temp1.x_d;
y_d = temp2.y_d;