function dp = hyd_dp_line(Q, rho, nu, L, D)
%HYD_DP_LINE  배관 마찰 압력손실 (Darcy-Weisbach, 층류/난류 연속 블렌딩)
%   층류  : dp = 32*mu*L*v/D^2           (Hagen-Poiseuille)
%   난류  : dp = f*(L/D)*rho*v|v|/2, f = 0.316*Re^-0.25 (Blasius)
%   두 값 중 큰 값을 사용 (Re~1200 에서 연속)
%#codegen
A  = pi*D^2/4;
v  = Q/A;
Re = abs(v)*D/nu;
dp_lam = 32*rho*nu*L*v/D^2;
ft     = 0.316/max(Re, 1)^0.25;
dp_tur = ft*(L/D)*0.5*rho*v*abs(v);
if abs(dp_tur) > abs(dp_lam)
    dp = dp_tur;
else
    dp = dp_lam;
end
end
