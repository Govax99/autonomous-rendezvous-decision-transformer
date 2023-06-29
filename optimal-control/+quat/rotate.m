function vp = rotate(v, q)
    t = 2*cross(q(1:3), v);
    vp = v + q(4)*t + cross(q(1:3), t);
end
