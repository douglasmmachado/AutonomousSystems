function draw_circular_obstacle(p_obs,radius)
hold on
if radius==0
    plot(p_obs(1),p_obs(2),'oblack','LineWidth',3);
else
    th = 0:pi/50:2*pi;
    x_circle = radius * cos(th) + p_obs(1);
    y_circle = radius * sin(th) + p_obs(2);
    plot(x_circle, y_circle);
    fill(x_circle, y_circle, 'k')
end 
end

