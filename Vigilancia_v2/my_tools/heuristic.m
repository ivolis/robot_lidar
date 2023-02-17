function heur = heuristic(cell, goal)
  
  %%% CALCULATING THE REMAINING COST FROM A CELL TO THE GOAL
  
  h_multip = [0.5,1, 2, 5, 10];
  

  heur = h_multip(1)*norm(cell-goal);
  
  
end
