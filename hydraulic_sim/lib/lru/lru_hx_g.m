function [fwd_out, bwd_out] = lru_hx_g(x, prm)
%#codegen
fwd_out = repmat([x(2); x(3)], prm.Nout, 1);
bwd_out = [x(1); x(3)];
end
