function heur = heuristic(cell, goal)
  
  %%% YOUR CODE FOR CALCULATING THE REMAINING COST FROM A CELL TO THE GOAL GOES HERE
  
  h_multip = [0.5,1, 2, 5, 10];
  

  heur = h_multip(1)*norm(cell-goal);
  
  
end
