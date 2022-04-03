clearvars;
syms timestep positive
syms gamma_s gamma_v gamma_b tau L_pos L_vel real
syms a b c real

A_estimator_hat = [-L_pos, 1; -L_vel, -gamma_s];
exp_A_t = simplify(expm(A_estimator_hat * timestep));

%Check out simplification
d = (gamma_s^2 - 4*L_vel - 2*L_pos*gamma_s + L_pos^2)^(1/2);
exp_p = exp(-timestep*(L_pos + gamma_s + d)/2);
exp_m = exp(-timestep*(L_pos + gamma_s - d)/2);

%exp_A_t_simp = [(exp_p*(d + L_pos - gamma_s) + exp_m*(d - L_pos + gamma_s))/(2*d),  (exp_m - exp_p)/d; (L_vel*exp_p - L_vel*exp_m)/d, (exp_p*(d - L_pos + gamma_s) + exp_m*(d + L_pos - gamma_s))/(2*d)];

%simplify(exp_A_t - exp_A_t_simp)

B_estimator_hat = [0, 0, L_pos; gamma_v, -gamma_b, L_vel];

%coefficient_for_u = simplify(int(expm(A_estimator*(timestep - tau))*B_estimator_hat, tau, [0, timestep]))


%% Trapeziodal version
determinant_inv = simplify(det(eye(2) - timestep*A_estimator_hat))

A_trapezoid_times_det = simplify((eye(2) + timestep*A_estimator_hat) * inv(eye(2) - timestep*A_estimator_hat) * determinant_inv)
B_trapezoid_times_det = simplify(inv(eye(2) - timestep*A_estimator_hat) * timestep/2 * B_estimator_hat * determinant_inv)
C_trapezoid_times_det = inv(eye(2) - timestep*A_estimator_hat) * determinant_inv