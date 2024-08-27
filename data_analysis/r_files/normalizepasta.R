

# Sample values for manual check
sample_data <- data$AvgAccel
min_val <- min(sample_data)
max_val <- max(sample_data)

# Manually normalize a sample value
sample_value <- 11.223993
normalized_value <- (sample_value - min_val) / (max_val - min_val)
print(normalized_value)

# Define a tolerance level
tolerance <- 1e-14

# Find approximate matches
approx_match <- abs(data$AvgAccel - sample_value) < tolerance
print(any(approx_match))


# Check against normalized data
normalized_sample_value <- normalized_data$AvgAccel[which(data$AvgAccel == sample_value)]
print(normalized_sample_value)