# read the CSV file
data <- read.csv("C:/Users/Student/Documents/kinova_share/data_analysis/robot_data.csv")

# filters the data for conditions 1 and 2, where 1 and 2 are the two conditions being compared
var1 <- "C"
var2 <- "D"
condition_1 <- subset(data, condition == var1)
condition_2 <- subset(data, condition == var2)

get_answers <- function(condition_1, condition_2){
  
  # COMPUTE AVERAGE TOTAL CHANGE
  # calculate the total change in position for each participant in each condition
  condition_1$total_change <- condition_1$RangeX + condition_1$RangeY + condition_1$RangeZ
  condition_2$total_change <- condition_2$RangeX + condition_2$RangeY + condition_2$RangeZ
  
  # compute the average total change for each participant
  average_change_1 <- mean(condition_1$total_change)
  average_change_2 <- mean(condition_2$total_change)
  
  # COMPUTE AVERAGE RANGE
  # compute the average range in position for each axis (RangeX, RangeY, RangeZ) for each condition
  average_range_x_1 <- mean(condition_1$RangeX)
  average_range_y_1 <- mean(condition_1$RangeY)
  average_range_z_1 <- mean(condition_1$RangeZ)
  
  average_range_x_2 <- mean(condition_2$RangeX)
  average_range_y_2 <- mean(condition_2$RangeY)
  average_range_z_2 <- mean(condition_2$RangeZ)
  
  # calculate the overall average range for each condition
  average_range_1 <- mean(c(average_range_x_1, average_range_y_1, average_range_z_1))
  average_range_2 <- mean(c(average_range_x_2, average_range_y_2, average_range_z_2))

  # Return the averages as a list
  return(list(average_change_1 = average_change_1, average_change_2 = average_change_2))
}

# Call the function and store the results
results <- get_answers(condition_1, condition_2)

# Extract the average changes
average_change_1 <- results$average_change_1
average_change_2 <- results$average_change_2

# Print the results
cat("Average change for ", var1, " = ", average_change_1, ".\n")
cat("Average change for ", var2, " = ", average_change_2, ".\n")

# Create a summary string
summary_results <- paste0("Average change for ", var1, " = ", average_change_1, 
                          ". \nAverage change for ", var2, " = ", average_change_2, ".")

# Print the summary string
cat(summary_results)




