function CtrlValue = GetLimitValue( CtrlValue, Max, Min)


if(CtrlValue < Min) 
    CtrlValue = Min;
elseif (CtrlValue > Max)
    CtrlValue = Max;
end

end

