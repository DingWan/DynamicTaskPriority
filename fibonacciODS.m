% Omer CAYIR
% 09.01.2015

% Inputs:
% f: function in string
% var: variable of f
% lambda_min: start point of IOU
% lambda_max: end point of IOU
% f_range: minimum IOU for ODS,
% epsilon: minimum of allowable difference
% ITERMAX: number of iteration for ODS,

% Outputs:
% minx: minimum point of the function
% fmin: function value of the minimum point
function [minx, fmin] = fibonacciODS(f,lambda_min,lambda_max,f_range,epsilon,ITERMAX,xk,dk)
% One dim search technique using Fibonacci Search

N = 1;
i_range = lambda_max - lambda_min;
fi = 1;

while(fi < ((1 + 2*epsilon)/(f_range/i_range)))
fi = fi + fi;
N = N + 1;
end

K = N + 1;

fibo = zeros(1, K);
fibo(1) = 1;
fibo(2) = 2;

if(K > 2)    
    for i = 3 : K
        fibo(i) = fibo(i-2) + fibo(i-1);
    end
end

for n=1:min(N-1,ITERMAX)
%     disp(['Iteration ', num2str(n)]);
    
    h = 1 - (fibo(K-1)/fibo(K));
%     disp(['h ', num2str(h)]);
    
    p1 = lambda_min + h*(lambda_max-lambda_min);
%     disp(['Point 1: ', num2str(p1)]);
    p2 = lambda_min + (1 - h)*(lambda_max-lambda_min);
%     disp(['Point 2: ', num2str(p2)]);
    
    fx1 = feval(f,xk+p1*dk);
%     disp(['Function value at p1: ', num2str(fx1)]);
    fx2 = feval(f,xk+p2*dk);
%     disp(['Function value at p2: ', num2str(fx2)]);
     
    if(fx1 > fx2)
        lambda_min = p1;
    elseif(fx1 < fx2)
        lambda_max = p2;
    end
    
%     disp(['Range [a,b]: [', num2str(a), ',', num2str(b), ']']);
%     disp(' ');
    
    K = K - 1;
end

% disp(['Iteration ', num2str(N)]);
h = 1 - (fibo(K-1)/fibo(K));
% disp(['h ', num2str(h)]);
    
p1 = lambda_min + (h-epsilon)*(lambda_max-lambda_min);
% disp(['Point 1: ', num2str(p1)])
p2 = lambda_min + (1 - h)*(lambda_max-lambda_min);
% disp(['Point 2: ', num2str(p2)]);
    
fx1 = feval(f,xk+p1*dk);
% disp(['Function value at p1: ', num2str(fx1)]);
fx2 = feval(f,xk+p2*dk);
% disp(['Function value at p2: ', num2str(fx2)]);
     
if(fx1 > fx2)
    lambda_min = p1;
elseif(fx1 < fx2)
    lambda_max = p2;
end
    
% disp(['Range [a,b]: [', num2str(a), ',', num2str(b), ']']);
% disp(' ');

minx = (lambda_min + lambda_max) /2;
fmin = feval(f,xk+minx*dk);