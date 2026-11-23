function [dx, mon] = lru_act_f(x, fwd_in, bwd_in, ext, prm)
%LRU_ACT_F  전기유압 서보 액추에이터 (EHSV + 복동 실린더, 조종면 PCU)
%  상태 x = [Qs; Ps; PA; PB; Pr; xp; vp; xv; T]
%    Qs : 공급 포트 유량 (관성)
%    Ps : 공급 갤러리 압력     PA,PB : 실린더 A/B 챔버 압력    Pr : 리턴 갤러리 압력
%    xp,vp : 피스톤 변위/속도   xv : 서보밸브 스풀 (정규화 -1..1)
%    T  : 액추에이터 유체 온도 (집중)
%  제어(ACE): xv_cmd = sat(Kx*(xcmd - xp)),  tau_v*dxv/dt = xv_cmd - xv
%  밸브 4랜드 오리피스 (언더랩 누설 CdA_ul 포함), 피스톤 내부누설 Cip
%  m*dvp/dt = (PA-PB)*Ap - k_aero*xp - b*vp - Fc*tanh(vp/vc) + Fext + Fstop
%  발열 = 모든 교축손실 + 마찰  (부하에 한 일은 열에서 제외)
%  ext(1) = 위치 명령 xcmd [m],  ext(2) = 외부(공력) 하중 Fext [N]
%#codegen
fl = prm.fl;
Qs = x(1); Ps = x(2); PA = x(3); PB = x(4); Pr = x(5);
xp = x(6); vp = x(7); xv = x(8); T = x(9);
Pu = fwd_in(1); Tu = fwd_in(2);
xcmd = ext(1); Fext = ext(2);

[rho, bs, nu, cp] = hyd_fluid(T, Ps, fl);
[~, bA] = hyd_fluid(T, PA, fl);
[~, bB] = hyd_fluid(T, PB, fl);
[~, bR] = hyd_fluid(T, Pr, fl);

% --- 공급 포트
Ap_port = pi*prm.Dp^2/4;
dps = hyd_dp_line(Qs, rho, nu, prm.Lp, prm.Dp);
dQs = (Pu - Ps - dps)/(rho*prm.Lp/Ap_port);

% --- ACE + EHSV
xv_cmd = max(min(prm.Kx*(xcmd - xp), 1), -1);
dxv = (xv_cmd - xv)/prm.tau_v;
xvc = max(min(xv, 1), -1);
Apos = prm.CdA_max*max(xvc, 0)  + prm.CdA_ul;
Aneg = prm.CdA_max*max(-xvc, 0) + prm.CdA_ul;
QPA = hyd_q_orifice(Ps - PA, rho, nu, Apos);
QAR = hyd_q_orifice(PA - Pr, rho, nu, Aneg);
QPB = hyd_q_orifice(Ps - PB, rho, nu, Aneg);
QBR = hyd_q_orifice(PB - Pr, rho, nu, Apos);
Qip = prm.Cip*(PA - PB);

% --- 챔버
VA = prm.V0A + prm.Ap*xp;
VB = prm.V0B - prm.Ap*xp;
dPs = bs/prm.Vs*(Qs - QPA - QPB);
dPA = bA/VA*(QPA - QAR - Qip - prm.Ap*vp);
dPB = bB/VB*(QPB - QBR + Qip + prm.Ap*vp);
[Qout, Tout] = hyd_split_bwd(bwd_in);
dPr = bR/prm.Vr*(QAR + QBR - sum(Qout));

% --- 기계계
Fh = (PA - PB)*prm.Ap;
Ff = prm.b*vp + prm.Fc*tanh(vp/prm.vc);
Fstop = 0;
if xp > prm.xmax
    Fstop = -prm.kst*(xp - prm.xmax) - prm.cst*vp;
elseif xp < -prm.xmax
    Fstop = -prm.kst*(xp + prm.xmax) - prm.cst*vp;
end
dvp = (Fh - prm.kaero*xp - Ff + Fext + Fstop)/prm.m;

% --- 에너지
Pdiss = QPA*(Ps - PA) + QAR*(PA - Pr) + QPB*(Ps - PB) + QBR*(PB - Pr) ...
      + Qip*(PA - PB) + Ff*vp + dps*Qs;
Vtot = prm.Vs + VA + VB + prm.Vr;
dT = hyd_energy(T, rho, cp, Vtot, Qs, Tu, Qout, Tout, Pdiss, prm.UA, prm.Tamb);

dx  = [dQs; dPs; dPA; dPB; dPr; vp; dvp; dxv; dT];
mon = [xp; xcmd; vp; Ps; PA; PB; Pr; Qs; sum(Qout); T; xv; Pdiss; Fh];
end
