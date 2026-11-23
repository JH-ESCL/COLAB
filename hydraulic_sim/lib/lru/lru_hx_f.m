function [dx, mon] = lru_hx_f(x, fwd_in, bwd_in, ext, prm)
%LRU_HX_F  열교환기 (Fuel-cooled / Air-cooled Heat Exchanger)
%  상태 x = [Q; P; T]
%    dp = R0*(nu/nu_ref)*Q + K2*Q|Q|            (튜브 번들 손실)
%    q  = eff*Cmin*(Tu - Tc) + UA_s*(T - Tc)     (e-NTU 대향류 + 정지 전도)
%  ext(1) = 냉각매체 질량유량 [kg/s],  ext(2) = 냉각매체 입구온도 [degC]
%#codegen
fl = prm.fl;
Q = x(1); P = x(2); T = x(3);
Pu = fwd_in(1); Tu = fwd_in(2);
mc = max(ext(1), 0); Tc = ext(2);

Tf = hyd_upwind(Q, Tu, T);
[rho, ~, nu, cp] = hyd_fluid(Tf, 0.5*(Pu + P), fl);
dp = prm.R0*(nu/prm.nu_ref)*Q + prm.K2*Q*abs(Q);
dQ = (Pu - P - dp)/prm.I;

% e-NTU (counter flow)
Ch = rho*cp*max(Q, 0);
Cc = mc*prm.cp_c;
Cmin = min(Ch, Cc); Cmax = max(Ch, Cc);
eff = 0;
if Cmin > 1e-3
    NTU = prm.UA_hx/Cmin;
    Cr  = Cmin/Cmax;
    if abs(1 - Cr) < 1e-6
        eff = NTU/(1 + NTU);
    else
        eff = (1 - exp(-NTU*(1 - Cr)))/(1 - Cr*exp(-NTU*(1 - Cr)));
    end
end
q = eff*Cmin*(Tu - Tc) + prm.UA_s*(T - Tc);

[Qout, Tout] = hyd_split_bwd(bwd_in);
[dP, dT] = hyd_node(P, T, prm.V, Q, Tu, Qout, Tout, dp*Q - q, prm.UA, prm.Tamb, fl);

dx  = [dQ; dP; dT];
mon = [P; Q; T; q; dp; eff];
end
