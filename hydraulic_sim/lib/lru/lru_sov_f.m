function [dx, mon] = lru_sov_f(x, fwd_in, bwd_in, ext, prm)
%LRU_SOV_F  공급 차단밸브 (Supply / Firewall Shutoff Valve, 모터구동)
%  상태 x = [Q; P; T; s]
%    s : 밸브 개도 (0=닫힘, 1=열림), 1차지연 + 스트로크 속도 제한
%    dp = 라인 마찰 + 오리피스(CdA(s))
%  ext(1) = 개폐 명령 (1=OPEN, 0=CLOSE)
%#codegen
fl = prm.fl;
Q = x(1); P = x(2); T = x(3); s = x(4);
Pu = fwd_in(1); Tu = fwd_in(2);

cmd = min(max(ext(1), 0), 1);
ds  = (cmd - s)/prm.tau;
ds  = max(min(ds, prm.rate), -prm.rate);

Tf = hyd_upwind(Q, Tu, T);
[rho, ~, nu] = hyd_fluid(Tf, 0.5*(Pu + P), fl);
A   = pi*prm.D^2/4;
CdA = prm.CdA_max*hyd_sat01(s) + prm.CdA_leak;
dp  = hyd_dp_line(Q, rho, nu, prm.L, prm.D) + hyd_dp_orifice(Q, rho, nu, CdA);
dQ  = (Pu - P - dp) / (rho*prm.L/A);

[Qout, Tout] = hyd_split_bwd(bwd_in);
[dP, dT] = hyd_node(P, T, prm.V, Q, Tu, Qout, Tout, dp*Q, prm.UA, prm.Tamb, fl);

dx  = [dQ; dP; dT; ds];
mon = [P; Q; T; s; dp];
end
