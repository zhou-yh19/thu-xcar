using JuMP
using Ipopt

mdl = Model(with_optimizer(Ipopt.Optimizer, print_level=0))
dt = 0.1
N = 5
car_length = 0.5

#=
About the index:
s[1] -> u[1] -> s[2] -> ... -> s[N] -> u[N]
=#

# 1. State variables
@variable(mdl, x[1:N], start=0.0)
@variable(mdl, y[1:N], start=0.0)
@variable(mdl, psi[1:N], start=0.0)

# 2. Input variables
@variable(mdl, -0.5 <= delta[1:N] <= 0.5, start=0.0)
@variable(mdl, 0.0 <= v[1:N] <= 5.0, start=0.0)

# 3. Objective
@NLparameter(mdl, x_r[i = 1:N] == 0.0)
@NLparameter(mdl, y_r[i = 1:N] == 0.0)
@NLparameter(mdl, psi_r[i = 1:N] == 0.0)

@NLparameter(mdl, C_x == 1.0)
@NLparameter(mdl, C_y == 1.0)
@NLparameter(mdl, C_psi == 10.0)

@NLparameter(mdl, C_delta == 0.0)
@NLparameter(mdl, C_v == 0.0)

function build_objective()
    mdl[:obj_tracking] = @NLexpression(mdl,
        C_x * sum((x[i] - x_r[i]) ^ 2 for i in 2:N) +
        C_y * sum((y[i] - y_r[i]) ^ 2 for i in 2:N) +
        C_psi * sum(1 - cos(psi[i] - psi_r[i]) for i in 2:N)
    )
    mdl[:obj_energy] = @NLexpression(mdl,
        C_delta * sum(delta[i] ^ 2 for i in 1:N) +
        C_v * sum(v[i] ^ 2 for i in 1:N)
    )
    @NLobjective(mdl, Min,
        mdl[:obj_tracking] +
        mdl[:obj_energy]
    )
end

build_objective()
    
# 4. System dynamics constraints
@NLparameter(mdl, x0 == 0.0); @NLconstraint(mdl, x[1] == x0);
@NLparameter(mdl, y0 == 0.0); @NLconstraint(mdl, y[1] == y0);
@NLparameter(mdl, psi0 == 0.0); @NLconstraint(mdl, psi[1] == psi0);

function forward(x, y, psi, delta, v)
    x_next = @NLexpression(mdl, x + dt * v * cos(delta + psi))
    y_next = @NLexpression(mdl, y + dt * v * sin(delta + psi))
    psi_next = @NLexpression(mdl, psi + dt * sin(delta) / car_length)
    
    return [x_next, y_next, psi_next]
end

for i in 1:(N - 1)
    x_next, y_next, psi_next = forward(x[i], y[i], psi[i], delta[i], v[i])
    @NLconstraint(mdl, x[i + 1] == x_next)
    @NLconstraint(mdl, y[i + 1] == y_next)
    @NLconstraint(mdl, psi[i + 1] == psi_next)
end

function update_state(s)
    x, y, psi = s
    set_value(x0, x)
    set_value(y0, y)
    set_value(psi0, psi)
end

function update_reference(ref)
    set_value.(x_r, ref[:, 1])
    set_value.(y_r, ref[:, 2])
    set_value.(psi_r, ref[:, 3])
    build_objective()
end                            

function mpc_step(current_state, ref)
    update_state(current_state)
    update_reference(ref)
    
    optimize!(mdl)
    status = termination_status(mdl)
    println(status)

    soln = (value.(delta[1:N]), value.(v[1:N]))

    u = [value(delta[1]), value(v[1])]
    
    set_start_value.(delta[1:N], vcat(value.(delta[2:N]), [value(delta[N])]))
    set_start_value.(v[1:N], vcat(value.(v[2:N]), [value(v[N])]))
    set_start_value.(x[1:N], vcat(value.(x[2:N]), [value(x[N])]))
    set_start_value.(y[1:N], vcat(value.(y[2:N]), [value(y[N])]))
    set_start_value.(psi[1:N], vcat(value.(psi[2:N]), [value(psi[N])]))
    
    return u
end