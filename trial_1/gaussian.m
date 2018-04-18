function distribution=gaussian(mu,sigma,x)
    k=0.5*(((x-mu)^2)/(sigma^2));
    distribution= (exp(-k))/(sqrt(2*pi*sigma^2));
end