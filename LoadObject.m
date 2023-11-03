function [Mesh_Object] = LoadObject(objectName, position, orientation)
    % LoadObject - Loads an object into the environment at the specified position 
    % and orientation from a given .ply file.
    %
    % Args:
    %   objectName   - Name of the .ply file (string).
    %   position     - 3-element vector specifying x,y,z position.
    %   orientation  - 3-element vector specifying rotations in radians 
    %                  about x, y, and z axes respectively.
    %
    % Returns:
    %   Mesh_Object  - A trisurf object of the loaded mesh.

    % Read the ply file to get vertices, face data, and color data.
    [f, v, data] = plyread(objectName, 'tri');

    % Extract color if available, otherwise use a default gray shade.
    if isfield(data.vertex, 'red') && isfield(data.vertex, 'green') && isfield(data.vertex, 'blue')
        define_colours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
    else
        define_colours = repmat([0.5, 0.5, 0.5], size(v, 1), 1);
    end

   % Plot the trisurf from face, vertex, and color data.
    Mesh_Object = trisurf(f, v(:,1), v(:,2), v(:,3), ...
        'FaceVertexCData', define_colours, ...
        'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', ...
        'AmbientStrength', 0.8, ...  % Increase ambient light strength
        'DiffuseStrength', 0.1);    % Increase diffuse light strength

    % Check if light sources already exist; if not, add them
    if isempty(findall(gca,'Type','light'))
        light('Position', [-1, -1, 1], 'Style', 'infinite'); % Add a light source
        camlight('headlight'); % Add a light that always points at the camera
    end

    % Center the object around the origin.
    objectVerts = v - mean(v);

    % Create a transformation matrix to adjust the object's position and orientation.
    innacuracyOffset = 0.006518;
    translateTR = makehgtform('translate', position + mean(v) - innacuracyOffset);
    
    % Rotations about x, y, and z axes respectively.
    rotateXTR = makehgtform('xrotate', orientation(1));
    rotateYTR = makehgtform('yrotate', orientation(2));
    rotateZTR = makehgtform('zrotate', orientation(3));
    
    objectPose = translateTR * rotateXTR * rotateYTR * rotateZTR;

    % Update vertices based on the transformation matrix.
    updatedPoints = (objectPose * [objectVerts, ones(size(v,1), 1)]')';  
    Mesh_Object.Vertices = updatedPoints(:, 1:3);

    drawnow();
end
