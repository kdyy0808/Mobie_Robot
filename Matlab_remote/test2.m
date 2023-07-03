
    
    L =0.6;
    a =0.8;
    b = 0.9;

    syms x y

    eqn1 = x^4 == 4*(x^2-y^2);
    eqn2 = (x-a)^2+(y-b)^2 ==L*L;
    
    sol = solve([eqn1,eqn2], [x,y]);

    x_answer = zeros(1,2);
    y_answer = zeros(1,2);
    count = 1;
    for i=1:length(sol.x)
%         if ~isreal(sol.x(i))
%             continue
        if ~(sol.x(i)==real(sol.x(i)))
            continue
        end
        x_answer(count) = double(sol.x(i));
        y_answer(count) = double(sol.y(i));
        count=count+1;
    end

    x1 = double(x_answer(1));
    x2 = double(x_answer(2));
    
    y1 = double(y_answer(1));
    y2 = double(y_answer(2));
