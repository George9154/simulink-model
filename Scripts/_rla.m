%  Written by: Leo Stocco
%  Plot the root locus of the TF with ASSYMPTOTES
% 
%  Syntax: [R K] = rla(GH)
%  Syntax: [R K] = rla(GH, k)
%  Syntax: [R K] = rla(GH, k, lw)
%  Syntax: [R K] = rla(GH, k, lw, zoom)
% 
%  R     = matrix of roots corresponding to each gain value
%  K     = vector of gain values
% 
%  Required:
%  GH    = O-L Transfer Function (system)
% 
%  Optional:
%  k     = vector of gains     def = nan (let Matlab decide)
%  lw    = rl line width       def = nan (lw = 4)
%  zoom  = amt to zoom OUT     def = 1
% 
%  Example:
%  To change line width to 2 but use the default k vector
%  rla(GH, nan, 2);
%