function zoom = getZoom(viewID)
%getZoom Get the zoom of the view.
%   zoom = getZoom(viewID) Returns the current zoom factor of the view. If 
%   no view ID is given, the function return the results for the default 
%   view.

%   Copyright 2016 Universidad Nacional de Colombia,
%   Politecnico Jaime Isaza Cadavid.
%   Authors: Andres Acosta, Jairo Espinosa, Jorge Espinosa.
%   $Id: getZoom.m 31 2016-09-28 15:16:56Z afacostag $

import traci.constants
if nargin < 1
    viewID = 'View #0';
end
zoom = traci.gui.getUniversal(constants.VAR_VIEW_ZOOM, viewID);