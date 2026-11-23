function [dx, mon] = lru_rsv_f(x, fwd_in, bwd_in, ext, prm) %#ok<INUSL>
%LRU_RSV_F  저장조 (Reservoir, 공기 가압식 + 압력 레귤레이터)
%  상태 x = [Q_1..Q_Nin; Vfl; Pg; T]
%    Q_i : 리턴 입구별 유량 (관성)   - 예) 1:주 리턴, 2:케이스드레인
%    Vfl : 유체 체적 (유면)            dVfl/dt = sum(Q_i) - sum(Qout)
%    Pg  : 가압 기체 압력              dPg/dt  = n*Pg/Vg*dVfl/dt + (Preg-Pg)/tau_reg
%    T   : 저장조 유체 온도 (완전혼합 가정)
%  출구압 P = Pg + rho*g*h   (h = Vfl/Abase)
%#codegen
fl  = prm.fl;
Nin = prm.Nin;
Qi  = x(1:Nin);
Vfl = x(Nin+1); Pg = x(Nin+2); T = x(Nin+3);

[rho, ~, ~, cp] = hyd_fluid(T, Pg, fl);
Pport = Pg + rho*9.81*prm.nz*max(Vfl, 0)/prm.Abase;

Ain = pi*prm.Din^2/4;
dQi = zeros(Nin, 1);
Tu  = zeros(Nin, 1);
heat = 0;
for i = 1:Nin
    Pu    = fwd_in(2*i-1);
    Tu(i) = fwd_in(2*i);
    Tf    = hyd_upwind(Qi(i), Tu(i), T);
    [rhoi, ~, nui] = hyd_fluid(Tf, Pport, fl);
    dp     = hyd_dp_line(Qi(i), rhoi, nui, prm.Lin, prm.Din) + prm.Kin*0.5*rhoi*(Qi(i)/Ain)*abs(Qi(i)/Ain);
    dQi(i) = (Pu - Pport - dp)/(rhoi*prm.Lin/Ain);
    heat   = heat + dp*Qi(i);
end

[Qout, Tout] = hyd_split_bwd(bwd_in);
dVfl = sum(Qi) - sum(Qout);
Vg   = max(prm.Vtot - Vfl, 0.02*prm.Vtot);
dPg  = prm.ngas*Pg/Vg*dVfl + (prm.Preg - Pg)/prm.tau_reg;
dT   = hyd_energy(T, rho, cp, max(Vfl, prm.Vmin), Qi, Tu, Qout, Tout, heat, prm.UA, prm.Tamb);

dx  = [dQi; dVfl; dPg; dT];
mon = [Pport; sum(Qi); T; Vfl; sum(Qout); Pg; 100*Vfl/prm.Vtot];
end
