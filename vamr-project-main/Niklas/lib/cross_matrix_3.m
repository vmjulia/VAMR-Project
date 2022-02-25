function mat = cross_matrix_3(v)
    mat = [0, - v(3), v(2);
           v(3), 0, -v(1);
           -v(2), v(1), 0];
end