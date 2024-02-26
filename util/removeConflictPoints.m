function [P1, removed] = removeConflictPoints(P1,P2,dist)
% Remove any points from the first given point set that is closer than a
% given distance to the second given point set.
% other given setset.
%
% SYNOPSIS:
%   [Pts, removed] = removeConflicPoints2(P1,P2,dist)
%
% PARAMETERS;
%   P1              A nx2 array of possible conflict points. If a point is
%                   closer to a point in P2 than the allowed tolerance dist
%                   it is removed.
%   P2              A mx2 array of coordinates.
%
%   dist            A mx1 array of allowed distances from the points P2.
%
% RETURNS:
%   P1              All points in P1 that are further away from P2 than the
%                   allowed distance.
%
%   removed         A nx1 logical array that is true for any P1 that are
%                   closer to a point in P2 than the allowed distance
%
% EXAMPLE
%   [X,Y] = meshgrid(1:10,1:10);
%   P1 = [X(:),Y(:)];
%   P2 = [5,5;2,2];
%   dist = [3;2];
%   ptsRem = removeConflictPoints(P1,P2,dist);
%   figure(); hold on
%   plot(P1(:,1),P1(:,2),'o')
%   plot(ptsRem(:,1),ptsRem(:,2),'.')
%   theta = linspace(0,2*pi)'
%   for i = 1:size(P2)
%   X = repmat(P2(i,:),100,1)+repmat(dist(i),100,2).*[cos(theta),sin(theta)]
%   plot(X(:,1), X(:,2))
%   end
%
% SEE ALSO:
%   splitWells, surfaceSites2D, compositePebiGrid2D, pebiGrid2D

%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2015-2020 Runar Lie Berge. See COPYRIGHT.TXT for details.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}
  if ispc
      usermem = memory;
      availableMemGB = usermem.MemAvailableAllArrays/1073741824;
  else
      [~,w] = unix('free | grep Mem');
      stats = str2double(regexp(w, '[0-9]*', 'match'));
      availableMemGB = stats(end)/1e6;
  end
  batchLimit = 0.04e9*availableMemGB; %seems to work on 32GB of memory, dont know how it scales
  nx = size(P1, 1);
  ny = size(P2, 1);
  if ny*nx > batchLimit
      midpoint = round(nx/2);

      [P11, removed1] = removeConflictPoints(P1(1:midpoint,:),P2,dist);
      [P12, removed2] = removeConflictPoints(P1(midpoint+1:end,:),P2,dist);
      removed = vertcat(removed1, removed2);
      P1 = [P11;P12];
      return
  end
  
  if isempty(P2)
    removed = false(size(P1,1),1);
    return
  end
  dist    = repmat(dist',size(P1,1),1);
  removed = any(pdist2(P1,P2)<dist,2);
  P1      = P1(~removed,:);

end
