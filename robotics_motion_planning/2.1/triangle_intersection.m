function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise

%%% All of your code should be between the two lines of stars.
% *******************************************************************
    flag = false;

    P1 = [P1 ones(size(P1, 1), 1)];
    P2 = [P2 ones(size(P2, 1), 1)];

    B1 = P2;
    B1(1, :) = P1(1, :);

    B2 = P2;
    B2(2, :) = P1(1, :);

    B3 = P2;
    B3(3, :) = P1(1, :);

    if abs(det(P2)/2) == abs(det(B1)/2) + abs(det(B2)/2) + abs(det(B3)/2);
        flag = true;
        return;
    end

% *******************************************************************
end
