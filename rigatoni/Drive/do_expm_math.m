clearvars;
syms delta_t positive
syms gamma_1m gamma_2m gamma_2b tau L_pos L_vel real
syms a b c real

L = [L_pos; L_vel];
C_e = [1, 0];
A_e = [0, 1; 0, -gamma_1m];
A_estimator_hat = [-L_pos, 1; -L_vel, -gamma_1m];
exp_A_t = simplify(expm(A_estimator_hat * delta_t));

%Check out simplification
d = (gamma_1m^2 - 4*L_vel - 2*L_pos*gamma_1m + L_pos^2)^(1/2);
exp_p = exp(-delta_t*(L_pos + gamma_1m + d)/2);
exp_m = exp(-delta_t*(L_pos + gamma_1m - d)/2);

%exp_A_t_simp = [(exp_p*(d + L_pos - gamma_1m) + exp_m*(d - L_pos + gamma_1m))/(2*d),  (exp_m - exp_p)/d; (L_vel*exp_p - L_vel*exp_m)/d, (exp_p*(d - L_pos + gamma_1m) + exp_m*(d + L_pos - gamma_1m))/(2*d)];

%simplify(exp_A_t - exp_A_t_simp)

B_estimator_hat = [0, 0, L_pos; gamma_2m, gamma_2b, L_vel];
C_estimator_hat = [0, 1];

%coefficient_for_u = simplify(int(expm(A_estimator*(delta_t - tau))*B_estimator_hat, tau, [0, delta_t]))


%% Trapeziodal version
%We do all this math factoring out the determinant of the inverse
syms est_xi_1 est_xi_2 real
syms motor_V brake_force enc_pos real

determinant_inv = simplify(det(eye(2) - delta_t*A_estimator_hat/2))
inv_not_det = simplify(determinant_inv * inv(eye(2) - delta_t*A_estimator_hat/2));

A_trapezoid_times_det = simplify((eye(2) + delta_t*A_estimator_hat/2) * inv_not_det);
B_trapezoid_times_det = simplify(inv_not_det * delta_t * B_estimator_hat);
C_trapezoid_times_det = C_estimator_hat * inv_not_det;
D_trapezoid_times_det = C_estimator_hat * B_trapezoid_times_det/2;

xi_new_times_det = simplify(A_trapezoid_times_det*[est_xi_1; est_xi_2] + B_trapezoid_times_det*[motor_V; brake_force; enc_pos])
vel_hat_times_det = simplify(C_trapezoid_times_det*[est_xi_1; est_xi_2] + D_trapezoid_times_det*[motor_V; brake_force; enc_pos])