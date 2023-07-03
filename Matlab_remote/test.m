
L = 0.5;
a = 1.3;%x_cord
b = 1.8;%y_cord


syms x y
eqn1 = x^2 +y^2 == 4;
eqn2 = (x-a)^2+(y-b)^2 ==L*L;

sol = solve([eqn1,eqn2], [x,y]);

x1 = double(sol.x(1));
x2 = double(sol.x(2));

y1 = double(sol.y(1));
y2 = double(sol.y(2));