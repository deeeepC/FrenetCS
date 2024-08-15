library(e1071)    # naive bayes classifier
library(pracma)   # numerical integration and polynomial tools
library(ggplot2)  # visualisation

# frenet-serret frame dynamics
frenet_serret_derivatives <- function(kappa, tau, T_vec, N_vec, B_vec) {
  # calc derivatives of f-s frame vectors
  dT_ds <- kappa * N_vec
  dN_ds <- -kappa * T_vec + tau * B_vec
  dB_ds <- -tau * N_vec
  return(list(dT_ds = dT_ds, dN_ds = dN_ds, dB_ds = dB_vec))
}


# trajectory prediction gnb
# gen filler trajectory data (x, y, class)
set.seed(123)
n_samples <- 1000
trajectory_data <- data.frame(
  x = c(rnorm(n_samples/2, 0, 1), rnorm(n_samples/2, 2, 1)),
  y = c(rnorm(n_samples/2, 0, 1), rnorm(n_samples/2, 3, 1)),
  class = factor(rep(c("straight", "curve"), each = n_samples/2))
)

# train nbc
nb_model <- naiveBayes(class ~ x + y, data = trajectory_data)

# predict function
predict_trajectory <- function(new_data) {
  predict(nb_model, new_data)
}

# 3. behaviour planning fsm
fsm_controller <- function(current_state, distance_to_stop, speed) {
  threshold <- 20  # meters
  speed_limit <- 50 # km/h
  
  if (current_state == "track speed") {
    if (distance_to_stop < threshold) {
      new_state <- "decelerate to stop"
      action <- list(deceleration = 2.5)  # m/s²
    } else {
      new_state <- "track speed"
      action <- list(target_speed = speed_limit)
    }
  } else if (current_state == "decelerate to stop") {
    if (distance_to_stop >= threshold) {
      new_state <- "track speed"
      action <- list(target_speed = speed_limit)
    } else {
      new_state <- "decelerate to stop"
      action <- list(deceleration = 2.5)
    }
  }
  return(list(state = new_state, action = action))
}

# 4.jerk-minimised path planning
# quintic poly traj gen
generate_trajectory <- function(t, alpha) {
  s <- alpha[1] + alpha[2]*t + alpha[3]*t^2 + 
       alpha[4]*t^3 + alpha[5]*t^4 + alpha[6]*t^5
  return(s)
}

# jerk mini obj func
jerk_objective <- function(alpha, t_span) {
  jerk <- function(t) {
    (60*alpha[6]*t^2 + 24*alpha[5]*t + 6*alpha[4])^2
  }
  integrate(jerk, t_span[1], t_span[2])$value
}

# opt poly coefficients
optimise_trajectory <- function(t_span, boundary_conditions) {
  # boundary conditions [s0, v0, a0, sf, vf, af]
  A <- matrix(c(
    1, t_span[1], t_span[1]^2, t_span[1]^3, t_span[1]^4, t_span[1]^5,
    0, 1, 2*t_span[1], 3*t_span[1]^2, 4*t_span[1]^3, 5*t_span[1]^4,
    0, 0, 2, 6*t_span[1], 12*t_span[1]^2, 20*t_span[1]^3,
    1, t_span[2], t_span[2]^2, t_span[2]^3, t_span[2]^4, t_span[2]^5,
    0, 1, 2*t_span[2], 3*t_span[2]^2, 4*t_span[2]^3, 5*t_span[2]^4,
    0, 0, 2, 6*t_span[2], 12*t_span[2]^2, 20*t_span[2]^3
  ), nrow = 6, byrow = TRUE)
  
  alpha <- solve(A) %*% boundary_conditions
  return(alpha)
}

# filler usage
# gen opt traj
t_span <- c(0, 10)  # 10-second horizon
boundary_conditions <- c(0, 20, 0, 100, 20, 0)  # [s0, v0, a0, sf, vf, af]
optimal_alpha <- optimise_trajectory(t_span, boundary_conditions)

# plot traj
t_seq <- seq(t_span[1], t_span[2], length.out = 100)
s_traj <- sapply(t_seq, function(t) generate_trajectory(t, optimal_alpha))

ggplot(data.frame(t = t_seq, s = s_traj), aes(t, s)) +
  geom_line(color = "blue", linewidth = 1) +
  labs(title = "jerk-minimised trajectory",
       x = "Time (s)", y = "Position (m)") +
  theme_minimal()

# behaviour planning filler
current_state <- "track speed"
distance_to_stop <- 25  # meters
fsm_result <- fsm_controller(current_state, distance_to_stop, 45)
print(fsm_result)

# traj pred filler
new_vehicle <- data.frame(x = 1.8, y = 2.9)
predicted_class <- predict_trajectory(new_vehicle)
print(paste("predicted trajectory type:", predicted_class))