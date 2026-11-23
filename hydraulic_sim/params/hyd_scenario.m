function e = hyd_scenario(t, id)
%HYD_SCENARIO  외부 입력(운용 시나리오). 모든 ID 에 대해 4x1 벡터 반환
%   id 1 SOV  : [cmd]                      개폐 명령
%   id 2 EDP  : [N_rpm]                    엔진 기어박스 회전수
%   id 3 ACMP : [power_on]                 전원
%   id 4 ACT  : [xcmd; Fext]               위치 명령 [m], 외부 하중 [N]
%   id 5 HX   : [mdot_fuel; T_fuel]        냉각 연료 유량/온도
%#codegen
e = zeros(4, 1);
switch id
    case 1  % SOV: 항상 OPEN, 1100 s 에 CLOSE (엔진 화재 스위치 시뮬레이션)
        e(1) = double(t < 1100);
    case 2  % EDP: 엔진 시동 0->idle(2800rpm) 3 s, 300 s 이륙출력 3750rpm
        Nidle = 2800; Nto = 3750;
        e(1) = Nidle*min(t/3, 1) + (Nto - Nidle)*min(max((t - 300)/10, 0), 1);
    case 3  % ACMP: 20 s 에 ON
        e(1) = double(t >= 20);
    case 4  % ACT: 10 s 부터 0.5 Hz, 20 mm 사인 + 600 s 에 40 mm 스텝
        xc = 0;
        if t >= 10
            xc = 0.020*sin(2*pi*0.5*(t - 10));
        end
        if t >= 600 && t < 620
            xc = 0.040;
        end
        e(1) = xc;
        e(2) = 0;
    case 5  % HX: 연료 0.6 kg/s, 15 degC
        e(1) = 0.6;
        e(2) = 15;
    otherwise
end
end
