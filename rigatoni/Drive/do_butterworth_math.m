clearvars;
syms delta_t positive
syms butter_vel butter_accel positive
syms filtered_target_vel filtered_target_accel real
syms trapezoidal_target_vel_avg real  %trapezoidal_target_vel_avg = (target_vel[k-1] + target_vel[k])/2

A_butter = [0, 1; -butter_vel, -butter_accel];
B_butter = [0; butter_vel];
C_butter = eye(2);



%% Trapeziodal version
%We do all this math factoring out the determinant of the inverse
syms vel_filt_xi_1 vel_filt_xi_2 real

disp("============ Trapezoidal ============")

determinant_inv = simplify(det(eye(2) - delta_t*A_butter/2))
inv_not_det = simplify(determinant_inv * inv(eye(2) - delta_t*A_butter/2));

vel_accel_filt_times_det = simplify(inv_not_det*((eye(2) + delta_t*A_butter/2)*[filtered_target_vel; filtered_target_accel] + delta_t*B_butter*trapezoidal_target_vel_avg))