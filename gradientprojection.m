% EE553 Term Project
% Gradient projection method of Rosen with modification to guarantee the
% convergence
% M.S. Bazaraa, H.D. Sherali, C.M. Shetty, “Nonlinear Programming: Theory
% and Algorithms? 2nd. ed., Wiley, New York, 1993.
% Omer CAYIR
% 29.01.2015

% Inputs:
% x: initial point
% maxiter: the maximum number of iteration for gradient projection
% ttc: threshold value to check whether the norm is too small
% grad: gradient computation is (1:analytically, 2:numerically)
% ODS: one-dim search method (1:Dichotomoous, 2:Fibonacci)
% odsmaxiter: the maximum number of iteration for one-dim search
% odsfinrange: the minimum of IOU for one-dim search

% Outputs:
% x: optimal solution
% MSE: function value at optimal solution
function [x, MSE] = gradientprojection(x,maxiter,ttc,grad, ...
                                            ODS,odsmaxiter,odsfinrange,kfactor)
global qmax qmin %Jacob Pend_velocity
% Obtain required matrices and vectors for Eq. 15
M = numel(x);
A = [eye(M);-eye(M)];%;Jacob
Q = ones(1,M);
b = [transpose(qmin);-transpose(qmax)];%;transpose(Pend_velocity)
s = 0.5; % scalar for Eq. 16
MSE = zeros(1,maxiter+1);
MSE(1) = objfun(x);
for k=1:maxiter
%%%%% Step 1
    gradx = objfungrad(x,grad);
    if (norm(gradx)>1e-4) % stop if the norm of the gradient is nearly zero
        % decomposition of A into A1 and A2
        I = find(abs(A*x-b)<ttc);
        A1 = A(I,:);
        J = setdiff(1:size(A,1),I);
%         A2=A(J,:);
        % obtain M
        Mt = [A1' Q'];
        M = Mt';
        if (isempty(M)) % if M is vacuous, let dk be the negative of gradient
            d = -gradx;
        else % otherwise obtain the projection matrix
            Pr = eye(numel(x))-(M'/(M*M'))*M;
            dI = -Pr*gradx; % compute dI
            w = -(M*M')\M*gradx; % also compute w
            u = w(1:numel(I)); % decomposition of w into u and v
            if (min(u)>=0) % if u is non-negative
                if (norm(dI)<ttc/10)
                    break % then stop if the norm of dI is nearly zero
                else
                    d = dI; % otherwise, let dk=dI
                end
            else % otherwise, find minimum element of u
                [uh,uj] = min(u);
                A1 = A1(setdiff(1:size(A1,1),uj),:); % drop the row of A1
                % which corresponds to uh
                Mhat = [A1' Q'];
                Mhat = Mhat'; % obtain Mhat
                Phat = eye(numel(x))-(Mhat'/(Mhat*Mhat'))*Mhat; % obtain the
                % new projection matrix Phat
                dII = -Phat*gradx; % compute dII
                if (norm(dI)>s*abs(uh)) % find dk by using Eq. 16
                    d = dI;
                else
                    d = dII;
                end
            end
        end
        % scale dk by its minimum element to avoid smaller norm    
        d = kfactor*d/max(min(abs(d)),1);        
%%%%%%%%% Step 2
        % find lambda_max by using Eq. 18
        bhat = b(J)-A(J,:)*x;
        dhat = A(J,:)*d;
        J = find(dhat>0);
        lambda_max = min(bhat(J)./dhat(J));
        % obtain lambda by one-dim search, as shown in Eq. 17
        if (ODS==1)
            lambda = dichotomousODS('objfun',0,lambda_max,odsfinrange,1e-10,odsmaxiter,x,d);
        else
            lambda = fibonacciODS('objfun',0,lambda_max,odsfinrange,1e-10,odsmaxiter,x,d);
        end       
        x = x+lambda*d; % compute the new point for k+1
        MSE(k+1) = objfun(x);
        
    else
        break
    end
end