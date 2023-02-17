function cost = edge_cost(parent, child, map)


    %%% CALCULATING THE COST FROM VERTEX parent TO VERTEX child
    
%     child_occup_prob = map(child(1),child(2)); % no tiene sentido esto en
%     este tp, map es un objeto de otro estilo al del tp

    child_occup_prob = getOccupancy(map,[child(1), child(2)],"grid");
    
    if child_occup_prob >= 0.2 % umbral de ocupacion
        cost = inf;
    else        
        cost = norm(parent-child) + child_occup_prob;
    end

end