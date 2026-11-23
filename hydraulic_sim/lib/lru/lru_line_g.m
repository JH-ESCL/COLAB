function [fwd_out, bwd_out] = lru_line_g(x, prm)
%LRU_LINE_G  배관 출력: fwd_out = [P;T] x Nout,  bwd_out = [Q;T]
%#codegen
fwd_out = repmat([x(2); x(3)], prm.Nout, 1);
bwd_out = [x(1); x(3)];
end
