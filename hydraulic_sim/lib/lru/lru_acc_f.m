function [dx, mon] = lru_acc_f(x, fwd_in, bwd_in, ext, prm) %#ok<INUSL>
%LRU_ACC_F  어큐뮬레이터 (Piston-type, N2 precharge)
%  상태 x = [Q; P; T]
%    Q : 포트 유량 (관성 + 리스트릭터)  rho*Lp/Ap*dQ/dt = Pu - P - dp(Q)
%    P : 유체측 압력                     C(P)*dP/dt = Q   (HYD_ACC_CAP)
%    T : 유체 온도
%  mon = [P Q T Vfl Vgas]
%#codegen
fl = prm.fl;
Q = x(1); P = x(2); T = x(3);
Pu = fwd_in(1); Tu = fwd_in(2);
[C, Vfl, Vg] = hyd_acc_cap(P, T, prm);

Tf = hyd_upwind(Q, Tu, T);
[rho, ~, nu, cp] = hyd_fluid(Tf, Pu, fl);
Ap = pi*prm.Dp^2/4;
dp = hyd_dp_line(Q, rho, nu, prm.Lp, prm.Dp) + hyd_dp_orifice(Q, rho, nu, prm.CdA_port);
dQ = (Pu - P - dp)/(rho*prm.Lp/Ap);
dP = Q/C;

dT = hyd_energy(T, rho, cp, Vfl + prm.Vdead, Q, Tu, 0, T, dp*Q, prm.UA, prm.Tamb);

dx  = [dQ; dP; dT];
mon = [P; Q; T; Vfl; Vg];
end
