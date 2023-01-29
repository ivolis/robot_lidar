function n = neighbors(cell, map_dimensions)

    n = [];

    pos_x = cell(2);
    pos_y = cell(1);
    size_x = map_dimensions(2);
    size_y = map_dimensions(1);

    %%% YOUR CODE FOR CALCULATING THE NEIGHBORS OF A CELL GOES HERE
    k = 0;
    for i = [-1, 0 1]
        for j = [-1 0 1]
            
            n_y = pos_y + i;
            n_x = pos_x + j;
            
            if n_x > 0 && n_x < size_x
                if n_y > 0 && n_y < size_y
                    k = k+1;
                    n(k,:) = [n_y, n_x];
                end
            end
            
        end
    end


    % Return nx2 vector with the cell coordinates of the neighbors. 
    % Because planning_framework.m defines the cell positions as pos = [cell_y, cell_x],
    % make sure to return the neighbors as [n1_y, n1_x; n2_y, n2_x; ... ]

end