function [ strOutput ] = printCMatrix( A )
% Print matrix A in a C way

% Get matrix dims from A
[m,n] = size(A);

strOutput = ' { ';

% Print the matrix
%fprintf(1,'{ ');
for i = 1:m,
    if i ~= 1,
        strOutput = [strOutput '\t'];
    end;
    for j = 1:n,
        if i == m && j == n,
            %fprintf(1,'%.4f }\n', A(i,j));
            strOutput = [strOutput sprintf('%.4f };\n', A(i,j))];
        else
            %fprintf(1,'%.4f, ', A(i,j));
            strOutput = [strOutput sprintf('%.4f, ', A(i,j))];
        end;
    end;
    if i ~= m,
        %fprintf(1,'}\n');
        strOutput = [strOutput '\n'];
    end;
end

