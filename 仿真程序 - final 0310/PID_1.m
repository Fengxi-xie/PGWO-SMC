function OutPut = PID_1(k_P,k_I,k_D,In)
global InteTotal_1 In_last_1
%比例
Out = -k_P * In;

%积分
InteTotal_1 = InteTotal_1 + In;
if(InteTotal_1>1000)
    InteTotal_1 = 1000;
elseif(InteTotal_1<-1000)
    InteTotal_1 = -1000;
end
Out = Out - k_I * InteTotal_1;

%微分
Out = Out - k_D * (In - In_last_1);
In_last_1 = In;

OutPut = Out;
end