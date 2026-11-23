function [dx, mon] = lru_line_f(x, fwd_in, bwd_in, ext, prm) %#ok<INUSL>
%LRU_LINE_F  배관 조립체 (Tube assembly / Hose)  - 흡입/압력/리턴 라인
%  상태 x = [Q; P; T]
%    Q : 입구 유량 (관성)     rho*L/A * dQ/dt = Pu - P - dp_fric(Q) - dp_minor(Q)
%    P : 라인 체적 압력        V/beta * dP/dt  = Q - sum(Qout)
%    T : 라인 유체 온도        에너지 방정식 + 마찰발열 dp*Q - UA*(T-Tamb)
%  입력  fwd_in = [Pu; Tu],  bwd_in = [Q1;T1;...;Qn;Tn] (하류 연결별)
%#codegen
fl = prm.fl;
Q = x(1); P = x(2); T = x(3);
Pu = fwd_in(1); Tu = fwd_in(2);

Tf = hyd_upwind(Q, Tu, T);
[rho, ~, nu] = hyd_fluid(Tf, 0.5*(Pu + P), fl);
A  = pi*prm.D^2/4;
v  = Q/A;
dp = hyd_dp_line(Q, rho, nu, prm.L, prm.D) + prm.K*0.5*rho*v*abs(v);
dQ = (Pu - P - dp) / (rho*prm.L/A);

[Qout, Tout] = hyd_split_bwd(bwd_in);
[dP, dT] = hyd_node(P, T, prm.V, Q, Tu, Qout, Tout, dp*Q, prm.UA, prm.Tamb, fl);

dx  = [dQ; dP; dT];
mon = [P; Q; T; dp];
end
