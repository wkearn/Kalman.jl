# Test pykalman implementation against Kalman.jl

# Harmonic oscillator problem

N = parse(ARGS[1])

using Kalman

a = [1 0.01;
     -0.01 1]

h = [1.0 0]

g = eye(2)

q = 1e-10*eye(2)

r = [0.1]'

x0 = State([1.,0],[1 0.1; -0.1 1])

f = LinearModel(a,g,q)

z = LinearObservationModel(h,r)

kf = BasicKalmanFilter(x0,f,z)

# Generate observations:

xs = fill(x0.x,1000)
ys = fill(h*x0.x,1000)
xp = fill(x0.x,1000)
pp = fill(x0.p,1000)


for i = 2:1000
    xs[i] = a*xs[i-1]
    ys[i] = h*xs[i] + sqrt(r)*randn(1)
end

for i = 2:1000
    predictupdate!(kf,Observation([ys[i]]))
    xp[i] = kf.x.x
    pp[i] = kf.x.p
end

println(@elapsed for j = 1:N

for i = 2:1000
    xs[i] = a*xs[i-1]
    ys[i] = h*xs[i] + sqrt(r)*randn(1)
end

for i = 2:1000
    predictupdate!(kf,Observation([ys[i]]))
    xp[i] = kf.x.x
    pp[i] = kf.x.p
end

end)


