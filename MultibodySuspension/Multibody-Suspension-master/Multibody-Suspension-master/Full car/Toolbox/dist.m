function l = distance(A,B)
    % euclidean distance between two points, A and B 
    l = sqrt((B(1) - A(1))^2 + (B(2) - A(2))^2 + (B(3) - A(3))^2);
end

