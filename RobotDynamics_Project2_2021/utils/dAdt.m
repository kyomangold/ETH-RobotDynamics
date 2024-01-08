function [ dA ] = dAdt( A, q, dq )
% Computes the time derivative of a matrix that depends on the generalized
% coordinates. Inputs and outputs are symbolic expressions.
% dA/dt(q, dq) = d/dt(A(q))

dA = sym(zeros(size(A)));
for i=1:size(A,1)
    for j=1:size(A,2)
        dA(i,j) = jacobian(A(i,j),q)*dq;
    end
end
end

