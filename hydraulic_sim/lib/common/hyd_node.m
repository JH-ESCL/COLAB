function [dP, dT] = hyd_node(P, T, V, Qin, Tin, Qout, Tout, Qheat, UA, Tamb, fl)
%HYD_NODE  유압 노드(용량성 체적)의 압력/온도 미분방정식
%   dP/dt = beta(P,T)/V * (sum(Qin) - sum(Qout))
%   dT/dt : HYD_ENERGY 참고
%#codegen
[rho, beta, ~, cp] = hyd_fluid(T, P, fl);
dP = beta/V * (sum(Qin) - sum(Qout));
dT = hyd_energy(T, rho, cp, V, Qin, Tin, Qout, Tout, Qheat, UA, Tamb);
end
