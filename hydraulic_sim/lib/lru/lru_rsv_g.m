function [fwd_out, bwd_out] = lru_rsv_g(x, prm)
%LRU_RSV_G  저장조 출력: 출구 [Pport;T] x Nout, 입구별 [Q_i;T]
%#codegen
Nin = prm.Nin;
Vfl = x(Nin+1); Pg = x(Nin+2); T = x(Nin+3);
rho   = hyd_fluid(T, Pg, prm.fl);
Pport = Pg + rho*9.81*prm.nz*max(Vfl, 0)/prm.Abase;
fwd_out = repmat([Pport; T], prm.Nout, 1);
bwd_out = zeros(2*Nin, 1);
bwd_out(1:2:end) = x(1:Nin);
bwd_out(2:2:end) = T;
end
