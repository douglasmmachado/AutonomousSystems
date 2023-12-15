%----------- STUDENTS FILE ----------------
function dpobs=distance_obs(p_robot,p_obs,radius)
    dpobs=((p_robot - p_obs) / norm(p_robot - p_obs)) * (norm(p_robot - p_obs) - radius);
end

