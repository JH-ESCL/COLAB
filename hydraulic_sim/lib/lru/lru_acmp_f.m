function [dx, mon] = lru_acmp_f(x, fwd_in, bwd_in, ext, prm)
%LRU_ACMP_F  교류 모터 펌프 (AC Motor Pump, ACMP / EMDP)
%  상태 x = [Qs; Pin; Tin; eps; Pout; Tout; Pc; Tc; w]
%    1~8 : 펌프 (HYD_PUMP_CORE)
%    w   : 모터 각속도 [rad/s]   J*dw/dt = Tm(w) - Tpump - b*w
%  유도전동기 토크 (Kloss): Tm = 2*Tmax*s*smax/(s^2 + smax^2),  s = (ws - w)/ws
%  ext(1) = 전원 (1=ON, 0=OFF)
%#codegen
w  = x(9);
on = double(ext(1) > 0.5);
N  = w*60/(2*pi);
[dxp, Tq, monp] = hyd_pump_core(x(1:8), fwd_in, bwd_in, N, prm);

s  = (prm.w_sync - w)/prm.w_sync;
Tm = on*2*prm.Tmax*s*prm.smax/(s^2 + prm.smax^2);
dw = (Tm - Tq - prm.b_m*w)/prm.J;

dx  = [dxp; dw];
mon = [monp; Tm; on*Tm*w/prm.eta_e];
end
