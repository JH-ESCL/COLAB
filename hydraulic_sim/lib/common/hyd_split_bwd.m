function [Qout, Tout] = hyd_split_bwd(bwd_in)
%HYD_SPLIT_BWD  역방향 신호 [Q1;T1;Q2;T2;...] -> 유량 벡터, 온도 벡터
%#codegen
Qout = bwd_in(1:2:end);
Tout = bwd_in(2:2:end);
end
