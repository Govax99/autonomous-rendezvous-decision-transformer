function R = quat2rotm(q)
    qA = [0, -q(3), q(2); ...
          q(3), 0, -q(1);
         -q(2), q(1), 0];

    qv = q(1:3);
    R = qv*qv' + q(4)^2*eye(3) + 2*q(4)*qA + qA*qA;
end
