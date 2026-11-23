function [dx, mon] = lru_filter_f(x, fwd_in, bwd_in, ext, prm) %#ok<INUSL>
%LRU_FILTER_F  필터 모듈 (Pressure / Return / Case-drain Filter Module)
%  상태 x = [Q_1..Q_Nin; Ph; Th; Qf; Po; To]
%    Q_i   : 입구 포트별 유량 (관성)
%    Ph,Th : 하우징(엘리먼트 상류) 압력/온도
%    Qf    : 엘리먼트+바이패스 통과 유량 (관성)
%    Po,To : 출구 압력/온도
%  엘리먼트 차압 (층류 다공질): dp = R0*(nu/nu_ref)*(1 + kclog*clog)*Qf
%  바이패스 밸브: dp > dp_crack 이면  dp = (Qf + Kbp*dp_crack)/(1/R + Kbp)
%  차압 지시기(DPI): dp > dp_ind 이면 POP (mon)
%#codegen
fl  = prm.fl;
Nin = prm.Nin;
Qi = x(1:Nin);
Ph = x(Nin+1); Th = x(Nin+2); Qf = x(Nin+3); Po = x(Nin+4); To = x(Nin+5);

% --- 입구 포트
Ain = pi*prm.Din^2/4;
dQi = zeros(Nin, 1);
Tu  = zeros(Nin, 1);
heat_h = 0;
for i = 1:Nin
    Pu    = fwd_in(2*i-1);
    Tu(i) = fwd_in(2*i);
    Tf    = hyd_upwind(Qi(i), Tu(i), Th);
    [rho, ~, nu] = hyd_fluid(Tf, Ph, fl);
    dp     = hyd_dp_line(Qi(i), rho, nu, prm.Lin, prm.Din);
    dQi(i) = (Pu - Ph - dp)/(rho*prm.Lin/Ain);
    heat_h = heat_h + dp*Qi(i);
end
[dPh, dTh] = hyd_node(Ph, Th, prm.Vh, Qi, Tu, Qf, To, heat_h, 0, prm.Tamb, fl);

% --- 필터 엘리먼트 + 바이패스
Tf = hyd_upwind(Qf, Th, To);
[rho, ~, nu] = hyd_fluid(Tf, Ph, fl);
R  = prm.R0*(nu/prm.nu_ref)*(1 + prm.kclog*prm.clog);
dpf = R*Qf;
Qbp = 0;
if Qf > 0 && prm.Kbp > 0 && R*Qf > prm.dp_crack
    dpf = (Qf + prm.Kbp*prm.dp_crack)/(1/R + prm.Kbp);
    Qbp = prm.Kbp*(dpf - prm.dp_crack);
end
dQf = (Ph - Po - dpf)/(rho*prm.Lf/prm.Af);

[Qout, Tout] = hyd_split_bwd(bwd_in);
[dPo, dTo] = hyd_node(Po, To, prm.Vo, Qf, Th, Qout, Tout, dpf*Qf, prm.UA, prm.Tamb, fl);

dx  = [dQi; dPh; dTh; dQf; dPo; dTo];
mon = [Po; Qf; To; dpf; Qbp; double(dpf > prm.dp_ind); Ph];
end
