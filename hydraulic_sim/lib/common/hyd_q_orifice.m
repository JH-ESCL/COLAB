function Q = hyd_q_orifice(dp, rho, nu, CdA)
%HYD_Q_ORIFICE  오리피스 유량 (차압 -> 유량, 대수식). 서보밸브 랜드 등에 사용
%   Q = CdA*sqrt(2/rho) * dp/(dp^2 + dpcr^2)^(1/4)
%#codegen
CdA  = max(CdA, 1e-14);
Qcr  = 12*nu*sqrt(pi*CdA)/2;
dpcr = rho/2*(Qcr/CdA)^2;
Q    = CdA*sqrt(2/rho) * dp/(dp^2 + dpcr^2)^0.25;
end
