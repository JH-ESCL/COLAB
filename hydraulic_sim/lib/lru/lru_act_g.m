function [fwd_out, bwd_out] = lru_act_g(x, prm)
%LRU_ACT_G  fwd_out = [Pr;T] x Nout (리턴 포트),  bwd_out = [Qs;T] (공급 포트)
%#codegen
fwd_out = repmat([x(5); x(9)], prm.Nout, 1);
bwd_out = [x(1); x(9)];
end
