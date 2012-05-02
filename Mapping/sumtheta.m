function s_theta = sumtheta(theta) %To get an accurate angle relative to the origin, must summate all of the angles up to current angle
for jj = 1:length(theta)
    s_theta = sum(theta(1):theta(jj));
end
end