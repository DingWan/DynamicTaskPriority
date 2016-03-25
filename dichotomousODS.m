% Omer CAYIR
% 09.01.2015

% Inputs:
% f: function in string
% var: variable of f
% lambda_min: start point of IOU
% b: end point of IOU
% f_range: minimum IOU for ODS,
% epsilon: minimum of allowable difference
% ITERMAX: number of iteration for ODS,

% Outputs:
% minx: minimum point of the function
% fmin: function value of the minimum point
function minx = dichotomousODS(f,lambda_min,lambda_max,f_range,epsilon,ITERMAX,xk,dk)
% One dim search technique using Dichotomous Search
iter=0;

found=true;

while found
    iter=iter+1;
    c= (lambda_min+lambda_max)/2 - epsilon;
    d= (lambda_min+lambda_max)/2 + epsilon;
    
     fc= feval(f,xk+c*dk);
     fd= feval(f,xk+d*dk);
     
     if fd>fc
         lambda_max=d;
     else
         lambda_min=c;    
     end
    
    if  norm(lambda_max-lambda_min) < f_range
    minx=(lambda_min+lambda_max)/2;    
    found=false;
    end 
    
    if iter>ITERMAX
        minx=(lambda_min+lambda_max)/2; 
        found=false;     
    end
end
end

