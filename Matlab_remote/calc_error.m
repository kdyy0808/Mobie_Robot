function error= calc_error(current_x,current_y)
%CALC_ERROR 이 함수의 요약 설명 위치
%   자세한 설명 위치

% size_factor= 2;
% eqn1 = y^4 == size_factor^2*4*(y^2-x^2);
T = 0:0.1:140;
size_factor= 2;
x = size_factor*sin(0.1*T);
y = 2*size_factor*sin(0.05*T);
%plot(x,y);

N = length(x);
Dist = zeros(N,1);

for i = 1:N
    Dist(i) = sqrt((x(i)-current_x)^2 + (y(i)-current_y)^2);
end
[min_distance,loc] = min(Dist);
indx = loc;


error = min_distance;
end

