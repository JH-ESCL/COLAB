function [dx, mon] = lru_man_f(x, fwd_in, bwd_in, ext, prm) %#ok<INUSL>
%LRU_MAN_F  압력 매니폴드 (Pressure Manifold / Module, 압력 트랜스듀서 포함)
%  상태 x = [Q_1..Q_Nin; P; T]
%    Q_i : 입구 통로별 유량 (관성 + 통로 손실)
%    P,T : 매니폴드 갤러리 압력/온도 (하류: 어큐뮬레이터, 릴리프, 분배 라인 등)
%  mon(1) = 압력 트랜스듀서 (1차 지연은 상위 모델에서 적용 가능)
%#codegen
fl  = prm.fl;
Nin = prm.Nin;
Qi = x(1:Nin); P = x(Nin+1); T = x(Nin+2);
A   = pi*prm.Din^2/4;
dQi = zeros(Nin, 1);
Tu  = zeros(Nin, 1);
heat = 0;
for i = 1:Nin
    Pu    = fwd_in(2*i-1);
    Tu(i) = fwd_in(2*i);
    Tf    = hyd_upwind(Qi(i), Tu(i), T);
    [rho, ~, nu] = hyd_fluid(Tf, P, fl);
    dp     = hyd_dp_line(Qi(i), rho, nu, prm.Lin, prm.Din) + prm.Kin*0.5*rho*(Qi(i)/A)*abs(Qi(i)/A);
    dQi(i) = (Pu - P - dp)/(rho*prm.Lin/A);
    heat   = heat + dp*Qi(i);
end
[Qout, Tout] = hyd_split_bwd(bwd_in);
[dP, dT] = hyd_node(P, T, prm.V, Qi, Tu, Qout, Tout, heat, prm.UA, prm.Tamb, fl);

dx  = [dQi; dP; dT];
mon = [P; sum(Qi); T; sum(Qout)];
end
