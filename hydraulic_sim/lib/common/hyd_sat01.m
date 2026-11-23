function y = hyd_sat01(x)
%HYD_SAT01  [0,1] 부드러운 포화 함수 (밸브 개도, 캐비테이션 계수 등)
%#codegen
e = 0.02;
y = 0.5*(sqrt(x^2 + e^2) - sqrt((x-1)^2 + e^2) + 1);
end
