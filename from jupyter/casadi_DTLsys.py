ode = Function('ode', [x, u], [normails])
dT = 1/20

# set up RK4
k1 = ode(x,       u)
k2 = ode(x+dT/2*k1,u)
k3 = ode(x+dT/2*k2,u)
k4 = ode(x+dT*k3,  u)
xf = x + dT/6 * (k1 + 2*k2 + 2*k3 + k4) 

DTsolution = Function('f',[u,x],[xf])
input_u = [1,1,1,1]
current_x = [0,0,0,0,0,0,0,0,0,0,0,0]
result = DTsolution(input_u,current_x)
