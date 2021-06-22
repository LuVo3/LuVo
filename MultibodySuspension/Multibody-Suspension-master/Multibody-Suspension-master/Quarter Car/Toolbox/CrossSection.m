function v = CrossSection(a,b,ubj,s)

    % function generating points needed to build the cross section of the
    % arm given the two chassis points, the boll joint and the thickness
    
    P = biella_equivalente(a,b,ubj);
    gamma1 = pi - acos(dot(ubj - a, P - ubj)/(len(ubj - a)*len(P - ubj)));
    gamma2 = pi - acos(dot(ubj - b, P - ubj)/(len(ubj - b)*len(P - ubj)));

    n1 = [0, -s/2];
    n2 = [len(a-ubj)*sin(gamma1) + s/2, len(a-ubj)*cos(gamma1)];
    n3 = [len(a-ubj)*sin(gamma1) - s/2, len(a-ubj)*cos(gamma1)];
    n4 = [0, s/2];
    n5 = [-len(b-ubj)*sin(gamma2) + s/2, len(b-ubj)*cos(gamma2)];
    n6 = [-len(b-ubj)*sin(gamma2) - s/2, len(b-ubj)*cos(gamma2)];

    v = [n1; n2; n3; n4; n5; n6];
    % x = v(:,1);
    % y = v(:,2);
    % plot(x,y)

end