function theta = inverseKinematics(d, a, alpha,Point)
    %theta, d, a, alpha, jointNumber
    theta = [0;0];
    Ts = 1e-3;
    threshold = 1e-3;
    p_controller = 10;
    Xd = Point(1,1);
    Yd = Point(1,2);
    distance = 1;
    while distance > threshold
        H = dh2ForwardKinematics(theta,d,a,alpha,1);
        X = H(1,4);
        Y = H(2,4);
        distance = sqrt((X-Xd)^2+(Y-Yd)^2);
        
        for i = 1:size(theta,1)
            theta(i) = theta(i) + Ts;
            H = dh2ForwardKinematics(theta,d,a,alpha,1);
            Xp = H(1,4);
            Yp = H(2,4);
            distancep = sqrt((Xp-Xd)^2+(Yp-Yd)^2);
            rate(i,1) = (distancep - distance)/Ts;
            theta(i,1) = theta(i) - Ts;
        end
        theta = theta - p_controller*rate;
    end
        
    