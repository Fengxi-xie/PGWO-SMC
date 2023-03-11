function OutPut = PD(k_P,k_D,In)
global In_last_1
%比例
Out = -k_P * In;

%微分
Out = Out - k_D * (In - In_last_1);
In_last_1 = In;

OutPut = Out;
end