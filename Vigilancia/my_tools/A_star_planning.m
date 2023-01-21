% NOTE that in octave/MATLAB, matrices are accessed with A(y,x),
% where y is the row and x is the column

function path = A_star_planning(map, start ,goal)


    % start and goal position (y,x)
    % convert world coordinates to grid indices (poses estan en metros)
    % (en el tp5 daba lo mismo, el mapa estaba ya asi discretizado, no habia
    % que convertir nada) o sea x=1 matriz era x=1m.
    start = world2grid(map, start);
    goal = world2grid(map,goal);

    % retrieve height and width of the map (in terms of 'discrete cells')
    map_size_discrete = map.GridSize;
    h = map_size_discrete(1);
    w = map_size_discrete(2);

    % cost values for each cell, filled incrementally. Initialize with infinity
    costs = ones(h,w)*inf;

    % estimated costs to the goal.
    heuristics = zeros(h,w);

    % cells that have been visited
    closed_list = zeros(h,w);

    % these matrices implicitly store the path
    % by containing the x and y position of the previous
    % node, respectively. Following these starting at the goal 
    % until -1 is reached returns the computed path, see at the bottom
    previous_x = zeros(h,w)-1;
    previous_y = zeros(h,w)-1;


    %start search at the start 
    parent=start;
    costs(start(1),start(2)) = 0;

    %loop until the goal is found
    while (parent(1)~=goal(1) || parent(2)~=goal(2))

      %generate mask to assign infinite costs for cells already visited
      closed_mask = closed_list;
      closed_mask( closed_mask==1 )=Inf; 

      %find the candidates for expansion (open list/frontier)
      open_list = costs + closed_mask + heuristics;

      %check if a non-infinite entry exists in open list (list is not empty)
      if min(open_list(:))==Inf
        disp('no valid path found');
        break
      end

      %find the cell with the minimum cost in the open list
      [y,x] = find(open_list == min(open_list(:)));
      parent_y = y(1);
      parent_x = x(1);
      parent = [parent_y, parent_x];

      %put parent in closed list
      closed_list(parent_y,parent_x) = 1;

      %get neighbors of parent
      n = neighbors(parent, [h,w]);
      for i=1:size(n,1)
        child_y = n(i,1);
        child_x = n(i,2);
        child = [child_y, child_x];

        %calculate the cost of reaching the cell
        cost_val = costs(parent_y,parent_x) + edge_cost(parent, child, map);

        %estimate the remaining costs from the cell to the goal
        heuristic_val = heuristic(child, goal);

        %update cost of cell
        if cost_val < costs(child_y,child_x)
            costs(child_y,child_x) = cost_val;
            heuristics(child_y,child_x) = heuristic_val;

            %safe child's parent
            previous_x(child_y,child_x) = parent_x;
            previous_y(child_y,child_x) = parent_y;
        end
      end

    end

    % output the final path
    parent = [goal(1), goal(2)];
    distance2 = 0;
    
    path = [];
    i = 1;
    while previous_x(parent(1), parent(2))>=0
        
        path(i, :) = grid2world(map, [parent(1), parent(2)]);

        child_y = previous_y(parent(1), parent(2));
        child_x = previous_x(parent(1), parent(2));
        child = [child_y, child_x];
        distance2 = distance2+norm(parent - child);
        parent = child;
        
        i = i + 1;
    end  
    
    % estoy yendo de nodos padres a hijos viendo el camino, si quiero verlo
    % mas claro lo pongo "de inicio a final"
    path = flip(path);

end