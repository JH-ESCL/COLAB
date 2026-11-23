function [dx, mon] = lru_edp_f(x, fwd_in, bwd_in, ext, prm)
%LRU_EDP_F  엔진 구동 펌프 (Engine Driven Pump, EDP)
%  상태 x = [Qs; Pin; Tin; eps; Pout; Tout; Pc; Tc]   (HYD_PUMP_CORE 참고)
%  ext(1) = 펌프 축 회전수 [rpm] (엔진 기어박스)
%  출구 연결 1 = 압력 포트, 2 = 케이스드레인 포트
%  mon = [Pout Qdel Tout Pin Tin Pc Qleak Tc eps N Pshaft Qheat fcav]
%#codegen
[dx, ~, mon] = hyd_pump_core(x, fwd_in, bwd_in, ext(1), prm);
end
