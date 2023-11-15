%%
function LaTexEx = Mat2LaTex(A)
% Copyright: 一般星人@知乎

Asize = size(A);
LaTexEx = ['\left[\begin{matrix}'];

for i=1:Asize(1)
    for j=1:Asize(2)
        if j < Asize(2)
            LaTexEx = [LaTexEx num2str(A(i,j)) '&'];
        else
            LaTexEx = [LaTexEx num2str(A(i,j))];
        end
    end
    LaTexEx = [LaTexEx '\\'];
end
LaTexEx = [LaTexEx '\end{matrix}\right]'];

end