# Vehicle Trajectory Modeling with Frenet Coordinates

An R implementation of trajectory prediction, behavior planning, and jerk-minimised path optimisation using Frenet-Serret dynamics, as described in *"Mathematical Modelling of Vehicle Trajectories Using the Frenet Coordinate System"*.

---

## Features

- **Frenet-Serret Dynamics**: Mathematical framework for modeling vehicle kinematics in a local coordinate system  
- **Gaussian Naive Bayes Prediction**: Probabilistic trajectory classification for surrounding vehicles  
- **Finite State Machine (FSM)**: Behavior planning with state transitions (e.g., "Track Speed" ↔ "Decelerate to Stop")  
- **Jerk-Optimised Paths**: Quintic polynomial trajectories minimising jerk for smooth motion  
- **Visualisation**: Integrated ggplot2 plotting for trajectory analysis  

---

## Installation

1. Ensure [R](https://www.r-project.org/) (≥ 4.0.0) is installed  
2. Install required packages:  
```r
install.packages(c("e1071", "pracma", "ggplot2"))
```


## Usage
### Calculate derivatives of tangent/normal/binormal vectors
```
frenet_serret_derivatives(kappa = 0.1, tau = 0.05, T_vec = c(1,0,0), N_vec = c(0,1,0), B_vec = c(0,0,1))
```

### Train classifier (sample data included)
```
nb_model <- naiveBayes(class ~ x + y, data = trajectory_data)
```

### Predict vehicle intent
```
predict_trajectory(new_data = data.frame(x = 1.8, y = 2.9))
```

### State transitions based on distance to stop
```
fsm_controller(current_state = "Track Speed", distance_to_stop = 25, speed = 45)
```

### Optimise 5th-order polynomial trajectory
```
optimal_alpha <- optimise_trajectory(t_span = c(0, 10), boundary_conditions = c(0, 20, 0, 100, 20, 0))
```

### Generate and plot trajectory
```
t_seq <- seq(0, 10, length.out = 100)
s_traj <- sapply(t_seq, function(t) generate_trajectory(t, optimal_alpha))
ggplot(data.frame(t = t_seq, s = s_traj), aes(t, s)) + geom_line()
```