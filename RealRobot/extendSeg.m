function [ from, to ] = extendSeg(A, B, Bias)
% extend the segment
d = distance(A, B);

from = A + Bias / d * (A - B);
to = B + Bias / d * (B - A);

end

