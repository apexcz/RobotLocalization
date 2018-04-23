function [ response ] = vectorProduct(A, B)
%% vectorProduct
% two vectors
% return the dot product of them
response = A(1) * B(2) - A(2) * B(1);
end

