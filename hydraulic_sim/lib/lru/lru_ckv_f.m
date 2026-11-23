function [dx, mon] = lru_ckv_f(x, fwd_in, bwd_in, ext, prm) %#ok<INUSL>
%LRU_CKV_F  체크밸브 (Pump discharge Check Valve)
%  상태 x = [Q; P; T; o]  (HYD_POPPET_CORE)
%  크래킹 차압 dp_crack (예: 5 psid), 역류시 CdA_leak 로 차단
%#codegen
[dx, mon] = hyd_poppet_core(x, fwd_in, bwd_in, prm);
end
