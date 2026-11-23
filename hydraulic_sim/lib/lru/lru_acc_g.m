function [fwd_out, bwd_out] = lru_acc_g(x, prm) %#ok<INUSD>
%#codegen
fwd_out = [x(2); x(3)];   % 하류 연결 없음 (Terminator)
bwd_out = [x(1); x(3)];
end
