function v = biella_equivalente(A,B,UBJ)
    % given the three points A,B and UBJ (/B,C,LBJ), the function returns
    % the coordinates of the point that lies on the intersection between:
    %
    %   - the vector (B - A), axis rotation of the triangle
    %   - the line that is orthogonal to the vector (B - A) that passes
    %     through UBJ
    % the line that connects the UBJ and the resulting point is the radius
    % of curvature of the trajectory of the UBJ
    n = (B - A)/dist(A,B);
    k = dot((UBJ - A), n);
    v = A + k*n;
end

