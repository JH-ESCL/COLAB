function Tf = hyd_upwind(Q, Tu, Td)
%HYD_UPWIND  유동 방향에 따른 상류(upwind) 온도 선택
%#codegen
if Q >= 0
    Tf = Tu;
else
    Tf = Td;
end
end
