% r = @(t) (0.1 + 1.2i) + 0.6*cos(t) + 1.05*1i*sin(t);
% t = linspace(0,2*pi);
% r1 = chebfun(@(t) r(t), [0, 2*pi], 'trig');
% r2 = 2*r1;
% r3 = r2 - (1 + 0i);
% plot(r1);

g_ = @(t) [0.2+0.3i] + 0.6*cos(t) + 0.2*1i*sin(t);
t = linspace(0,2*pi);
g = chebfun(@(t) g_(t), [0, 2*pi], 'trig'); %Initial boundary
init = plot(g, 'k')
set(init,'LineWidth',2);

f_=@(x,y)x.^2+y.^2-1;
f = chebfun2(@(x,y) f_(x,y), [-5 5 -5 5]);
hold on

for k = 1:7
    dg = diff(g); n_ = -1i*dg; n = n_./abs(n_); %Unit normal to boundary
    fn = n.*f(real(g), imag(g)); %Boundary search direction
    g = g - 0.2*fn; %Updated boundary
    plot(g, 'b')
end

exact = fimplicit(f_, 'r');
set(exact,'LineWidth',2);
axis equal

