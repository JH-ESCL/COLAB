function [dx, mon] = hyd_poppet_core(x, fwd_in, bwd_in, prm)
%HYD_POPPET_CORE  포펫형 밸브 공통 모델 (체크밸브 / 릴리프밸브)
%  상태 x = [Q; P; T; o]
%    rho*Lp/Ap*dQ/dt = Pu - P - dp_port(Q) - dp_v(Q)
%    밸브 차압-유량 특성 (단조 증가 -> 채터링 없는 안정한 시스템 수준 모델):
%      Q >= 0 : dp_v = dp_crack*tanh(Q/Q_o) + R_ovr*Q + dp_orifice(Q, CdA_max)
%               (크래킹압 + 오버라이드(스프링) 기울기 + 전개 오리피스)
%      Q <  0 : dp_v = dp_crack*tanh(Q/Q_o) + R_ovr*Q
%               + (R_rev - R_ovr)*Q*(1 - exp(Q/Q_o))  (역방향/폐쇄 누설, |Q|>>Q_o 에서 기울기 R_rev)
%      Q = 0 에서 값과 기울기가 연속(C1)이어야 합니다. 기울기가 끊기면(순방향 ~1e9 -> 역방향 ~1e13)
%      유량이 0을 지날 때 Newton 반복이 잘못 수렴해서 가짜 체크밸브 slamming이 생깁니다.
%      -> 차압 < dp_crack 이면 Q ~ Q_o*atanh(dp/dp_crack) 수준의 미소 누설
%    o : 포펫 개도 표시 (1차 지연)   do/dt = (sat(Q/Q_rated) - o)/tau
%    P,T : 밸브 하류 체적
%#codegen
fl = prm.fl;
Q = x(1); P = x(2); T = x(3); o = x(4);
Pu = fwd_in(1); Tu = fwd_in(2);

Tf = hyd_upwind(Q, Tu, T);
[rho, ~, nu] = hyd_fluid(Tf, 0.5*(Pu + P), fl);
Ap = pi*prm.Dp^2/4;
dpv = prm.dp_crack*tanh(Q/prm.Q_o) + prm.R_ovr*Q;
if Q >= 0
    dpv = dpv + hyd_dp_orifice(Q, rho, nu, prm.CdA_max);
else
    dpv = dpv + (prm.R_rev - prm.R_ovr)*Q*(1 - exp(Q/prm.Q_o));
end
dp = hyd_dp_line(Q, rho, nu, prm.Lp, prm.Dp) + dpv;
dQ = (Pu - P - dp)/(rho*prm.Lp/Ap);

dop = (hyd_sat01(Q/prm.Q_rated) - o)/prm.tau;

[Qout, Tout] = hyd_split_bwd(bwd_in);
[dP, dT] = hyd_node(P, T, prm.V, Q, Tu, Qout, Tout, dp*Q, prm.UA, prm.Tamb, fl);

dx  = [dQ; dP; dT; dop];
mon = [P; Q; T; o; dp];
end
