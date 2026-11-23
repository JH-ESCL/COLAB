function dp = hyd_dp_orifice(Q, rho, nu, CdA)
%HYD_DP_ORIFICE  오리피스 압력손실 (유량 -> 차압, 0 유량 근처 층류 정규화)
%   dp = rho/(2*CdA^2) * Q*sqrt(Q^2 + Qcr^2)
%   Qcr = Re_cr*nu*sqrt(pi*CdA)/2 ,  Re_cr = 12
%#codegen
CdA = max(CdA, 1e-14);
Qcr = 12*nu*sqrt(pi*CdA)/2;
dp  = rho/(2*CdA^2) * Q*sqrt(Q^2 + Qcr^2);
end
