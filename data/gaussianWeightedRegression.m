function [y] = gaussianWeightedRegression(x,mus,sigmas,weights)
%TODO add a check that all vectors are same size
size(mus,2);
psi = 0;
psiW = 0;
for m=1:size(mus,2); %all gaussians
    psi = psi + exp((-1/(2*sigmas(m)^2)) * ((x-mus(m))^2));
    psiW = psiW + weights(m)*exp((-1/(2*sigmas(m)^2)) * ((x-mus(m))^2));
end
y = psiW/psi;
