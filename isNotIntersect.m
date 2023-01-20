function IsNotIntersect = isNotIntersect(point1, point2)
        %create a line from point1 to point 2
        x1 = point1(1,1);
        x2 = point2(1,1);
        y1 = point1(1,2);
        y2 = point2(1,2);
        
        a = (y1 - y2)/(x1 - x2);
        b = y1 - a*x1;
        
        %not to show any text during calculation
        options = optimoptions('fsolve','Display','off');
        
        %initial guess
        x0 = 0;
        %intersect function of the square obstacle
        %intersect if there exist x such that f(x) = 0
        f = @(x) [abs(x*(a+1)+b) + abs(x*(1-a)-b) - 2];
        
        %ExitFlag determine if the function convert to zero or not
        [x,fval,ExitFlag] = fsolve(f,x0,options);
        %ExitFlag = 2 (not converge to root)
        
        if ExitFlag == -2
            IsNotIntersect = 1;
        else
            IsNotIntersect = 0;
        end
        