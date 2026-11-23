function [dx, mon] = lru_rv_f(x, fwd_in, bwd_in, ext, prm) %#ok<INUSL>
%LRU_RV_F  시스템 릴리프 밸브 (System Pressure Relief Valve)
%  상태 x = [Q; P; T; o]  (HYD_POPPET_CORE)
%  P,T = 릴리프 밸브 리턴측 체적.  dp_crack ~ 3500 psid, 전개 dp_crack+dp_full
%  릴리프 유량은 교축발열 dp*Q 가 그대로 유체 온도 상승으로 나타남
%#codegen
[dx, mon] = hyd_poppet_core(x, fwd_in, bwd_in, prm);
end
