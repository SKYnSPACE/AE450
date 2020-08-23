function RM = Quat2RM(q)

    q = q./sqrt(sum(q.^2));
    qhat = [    0, -q(4),  q(3);...
             q(4),     0, -q(2);...
            -q(3),  q(2),     0];
    RM = eye(3) + 2*qhat*qhat  + 2*q(1)*qhat;

end