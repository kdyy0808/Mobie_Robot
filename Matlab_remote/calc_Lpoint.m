function [x1,y1,x2,y2,x3,y3,x4,y4,count] = calc_Lpoint(x_robot_center,y_robot_center,Lookahead_Distance);
    L = Lookahead_Distance;
    
    
    a = x_robot_center;%x_cord
    b = y_robot_center;%y_cord

    x3=0;
    x4=0;
    y3=0;
    y4=0;


%     disp("calc time")
%     tic
    syms x y

    size_factor= 2;

    eqn1 = y^4 == size_factor^2*4*(y^2-x^2);
    %https://mathshistory.st-andrews.ac.uk/Curves/Eight/
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
    count=count-1;

    if count>2
        %disp("wowwwwwwwwwwwwwww")
        %disp(count)
    end

    
%     toc
    x1 = double(x_answer(1));
    x2 = double(x_answer(2));
    
    y1 = double(y_answer(1));
    y2 = double(y_answer(2));

    if count>2
        x3 = double(x_answer(3));
        x4 = double(x_answer(4));
        
        y3 = double(y_answer(3));
        y4 = double(y_answer(4));
    
    end





end