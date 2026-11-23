function [fwd_out, bwd_out] = lru_edp_g(x, prm) %#ok<INUSD>
%LRU_EDP_G  fwd_out = [Pout;Tout;Pc;Tc],  bwd_out = [Qs;Tin]
%#codegen
fwd_out = [x(5); x(6); x(7); x(8)];
bwd_out = [x(1); x(3)];
end
