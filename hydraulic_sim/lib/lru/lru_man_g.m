function [fwd_out, bwd_out] = lru_man_g(x, prm)
%#codegen
Nin = prm.Nin;
fwd_out = repmat([x(Nin+1); x(Nin+2)], prm.Nout, 1);
bwd_out = zeros(2*Nin, 1);
bwd_out(1:2:end) = x(1:Nin);
bwd_out(2:2:end) = x(Nin+2);
end
