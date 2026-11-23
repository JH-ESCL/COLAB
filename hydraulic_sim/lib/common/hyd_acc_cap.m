function [C, Vfl, Vg] = hyd_acc_cap(P, T, prm)
%HYD_ACC_CAP  피스톤형 어큐뮬레이터 유체측 컴플라이언스 C = dV/dP 및 유체체적
%   P <  Ppre : 피스톤 바닥 -> 잔류체적 압축     C = Vdead/beta(P,T)
%   P >= Ppre : N2 폴리트로픽 Vg = V0*(Ppre/P)^(1/n),  C += Vg/(n*P)
%   Ppre 전후 dP_s 구간에서 부드럽게 전환 (적분 안정)
%   Vfl = V0 - Vg  (P >= Ppre), 0 (P < Ppre)
%#codegen
[~, beta] = hyd_fluid(T, P, prm.fl);
Pg = max(P, prm.Ppre);
Vg = prm.V0*(prm.Ppre/Pg)^(1/prm.ngas);
w  = 0.5*(1 + tanh((P - prm.Ppre)/prm.dP_s));     % 0: 바닥, 1: 피스톤 자유
C  = prm.Vdead/beta + w*Vg/(prm.ngas*Pg);
Vfl = prm.V0 - Vg;
end
