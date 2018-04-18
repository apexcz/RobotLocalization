function A = navigate(qr, qn, dist, stepsize)
   qnew = [0 0];
   
   % Steer towards qn with maximum step size of stepsize
   if dist >= stepsize
       qnew(1) = qn(1) + ((qr(1)-qn(1))*stepsize)/distance(qr,qn);
       qnew(2) = qn(2) + ((qr(2)-qn(2))*stepsize)/distance(qr,qn);
   else
       qnew(1) = qr(1);
       qnew(2) = qr(2);
   end   
   A = [qnew(1), qnew(2)];
end