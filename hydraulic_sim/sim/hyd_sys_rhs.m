function [dX, mon] = hyd_sys_rhs(t, X, S)
%HYD_SYS_RHS  ODE 레퍼런스 모델 우변 (Simulink 모델과 동일한 LRU 함수/연결 사용)
nb = S.nb;
fo = cell(nb, 1); bo = cell(nb, 1);
for k = 1:nb    % 1) 출력 함수 g : 상태만의 함수 (대수루프 없음)
    [fo{k}, bo{k}] = S.blk(k).g(X(S.blk(k).idx), S.blk(k).prm);
end
dX = zeros(size(X));
mon = cell(nb, 1);
for k = 1:nb    % 2) 미분 함수 f
    B = S.blk(k);
    fwd_in = zeros(2*B.Nin, 1);
    for i = 1:B.Nin
        a = B.up(i,1); o = B.up(i,2);
        fwd_in(2*i-1:2*i) = fo{a}(2*o-1:2*o);
    end
    if B.Nout > 0
        bwd_in = zeros(2*B.Nout, 1);
        for j = 1:B.Nout
            d = B.dn(j,1); i = B.dn(j,2);
            bwd_in(2*j-1:2*j) = bo{d}(2*i-1:2*i);
        end
    else
        bwd_in = [0; S.P.T0];
    end
    ext = hyd_scenario(t, B.extId);
    [dX(B.idx), mon{k}] = B.f(X(B.idx), fwd_in, bwd_in, ext, B.prm);
end
end
