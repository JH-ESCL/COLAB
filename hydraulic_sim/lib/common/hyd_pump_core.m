function [dx, Tq, mon] = hyd_pump_core(x, fwd_in, bwd_in, N_rpm, prm)
%HYD_PUMP_CORE  압력보상형 가변용량 피스톤 펌프 공통 모델 (EDP / ACMP 공용)
%  상태 x = [Qs; Pin; Tin; eps; Pout; Tout; Pc; Tc]
%    Qs   : 흡입 포트 유량 (관성)
%    Pin,Tin : 흡입 챔버 압력/온도
%    eps  : 사판(yoke) 개도 0..1   deps/dt = (eps_ss(Pout) - eps)/tau_c
%    Pout,Tout : 토출 챔버 압력/온도
%    Pc,Tc     : 케이스 압력/온도 (내부누설 + 기계손실 발열 -> 케이스드레인)
%  Qth   = eps*Dmax*N/60*fcav(Pin)        (이론 유량, 캐비테이션시 감소)
%  Qleak = Cleak*(nu_ref/nu)*(Pout - Pc)  (점도 의존 내부 누설 -> 케이스)
%  Tq    = Qth*dp/(2*pi*n*eta_m) + Tdrag  (축 토크)
%  bwd_in = [Q_press; T_press; Q_case; T_case]
%#codegen
fl = prm.fl;
Qs = x(1); Pin = x(2); Tin = x(3); epsy = x(4);
Pout = x(5); Tout = x(6); Pc = x(7); Tc = x(8);
Pu = fwd_in(1); Tu = fwd_in(2);

% --- 흡입 포트
Tf = hyd_upwind(Qs, Tu, Tin);
[rho, ~, nu] = hyd_fluid(Tf, Pin, fl);
Ain  = pi*prm.Din^2/4;
dpin = hyd_dp_line(Qs, rho, nu, prm.Lin, prm.Din);
dQs  = (Pu - Pin - dpin)/(rho*prm.Lin/Ain);

% --- 배제 유량, 누설
n     = max(N_rpm, 0)/60;                         % rev/s
fcav  = hyd_sat01((Pin - prm.Pcav)/prm.dPcav);    % 충진 계수
ec    = hyd_sat01(epsy);
Qth   = ec*prm.Dmax*n*fcav;
[~, ~, nuo] = hyd_fluid(Tout, Pout, fl);
Qleak = prm.Cleak*(prm.nu_ref/nuo)*(Pout - Pc);
Qdel  = Qth - Qleak;

% --- 압력 보상기 (compensator)
eps_ss = hyd_sat01((prm.Pdead - Pout)/(prm.Pdead - prm.Pfull));
deps   = (eps_ss - epsy)/prm.tau_c;

% --- 토크/동력/발열
dpp    = max(Pout - Pin, 0);
Phyd   = Qth*dpp;                                  % 이론 유압동력
% 압력 토크는 회전 방향에 대한 저항으로만 작용 (정지 펌프가 모터로 역구동되지 않음)
sg     = tanh(N_rpm/60/0.5);
Tq     = (ec*prm.Dmax*fcav*dpp/(2*pi*prm.eta_m) + prm.Tdrag)*sg;
Pshaft = Tq*2*pi*n;
Pfric  = Pshaft - Phyd;                            % 기계 손실 (열)
Pleak  = Qleak*(Pout - Pc);                        % 누설 교축 손실 (열)

% --- 흡입 챔버
[dPin, dTin] = hyd_node(Pin, Tin, prm.Vin, Qs, Tu, Qth, Tin, dpin*Qs, 0, prm.Tamb, fl);
% --- 토출 챔버
[dPout, dTout] = hyd_node(Pout, Tout, prm.Vout, Qdel, Tin, bwd_in(1), bwd_in(2), ...
                          prm.phi_out*Pfric, 0, prm.Tamb, fl);
% --- 케이스
[dPc, dTc] = hyd_node(Pc, Tc, prm.Vcase, Qleak, Tout, bwd_in(3), bwd_in(4), ...
                      Pleak + (1 - prm.phi_out)*Pfric, prm.UA, prm.Tamb, fl);

dx  = [dQs; dPin; dTin; deps; dPout; dTout; dPc; dTc];
mon = [Pout; Qdel; Tout; Pin; Tin; Pc; Qleak; Tc; epsy; N_rpm; Pshaft; Pfric + Pleak; fcav];
end
