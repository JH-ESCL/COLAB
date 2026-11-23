function [rho, beta, nu, cp, mu] = hyd_fluid(T, P, fl)
%HYD_FLUID  작동유 물성치 (MIL-PRF-83282 근사)
%   [rho, beta, nu, cp, mu] = hyd_fluid(T, P, fl)
%   T  [degC], P [Pa abs]
%   rho  밀도 [kg/m^3]       : 열팽창 + 압축성
%   beta 유효체적탄성계수 [Pa] : 액체 + 혼입공기(entrained air)
%   nu   동점도 [m^2/s]       : 지수 근사 nu = nu0*exp(-lambda*(T-T0))
%   cp   비열 [J/kg/K]
%   mu   점성계수 [Pa*s]
%#codegen
Pp  = max(P, fl.Pmin);
dT  = T - fl.T0;

rho = fl.rho0 * (1 - fl.alpha*dT) * (1 + (Pp - fl.Patm)/fl.beta0);
nu  = fl.nu0 * exp(-fl.lambda*dT);

betaL = fl.beta0*(1 - fl.kbT*dT) + fl.kbP*(Pp - fl.Patm);
betaL = max(betaL, 0.2*fl.beta0);

% 혼입공기 유효 체적탄성계수 (polytropic 공기 + 액체)
r    = fl.xair * (fl.Patm/Pp)^(1/fl.nair);
beta = betaL*(1 + r) / (1 + r*betaL/(fl.nair*Pp));

cp = fl.cp;
mu = nu*rho;
end
