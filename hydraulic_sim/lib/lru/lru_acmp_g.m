function [fwd_out, bwd_out] = lru_acmp_g(x, prm) %#ok<INUSD>
%#codegen
fwd_out = [x(5); x(6); x(7); x(8)];
bwd_out = [x(1); x(3)];
end
