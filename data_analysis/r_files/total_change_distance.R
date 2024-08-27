library(ggsci)
library(RColorBrewer)
library(tidyverse)
library(ggpubr)
library(rstatix)
library(magrittr)
library(dplyr)
library(ggplot2)
library(extrafont)
library(tidyr)
library(stringr)
font_import()
loadfonts(device = "win")
fonts()
library(readr)

# Read the CSV file
distance_data <- read.csv("C:/Users/Student/Documents/kinova_share/data_analysis/robot_data.csv")

folder_path <- paste0("C:/Users/Student/Documents/AllPlots")
if (!file.exists(folder_path)) {
  dir.create(folder_path)
}

# Filters the data for conditions 1 to 8
var1 <- "A"
var2 <- "B"
var3 <- "C"
var4 <- "D"
var5 <- "E"
var6 <- "F"
var7 <- "G"
var8 <- "H"

condition_1 <- subset(distance_data, condition == var1)
condition_2 <- subset(distance_data, condition == var2)
condition_3 <- subset(distance_data, condition == var3)
condition_4 <- subset(distance_data, condition == var4)
condition_5 <- subset(distance_data, condition == var5)
condition_6 <- subset(distance_data, condition == var6)
condition_7 <- subset(distance_data, condition == var7)
condition_8 <- subset(distance_data, condition == var8)

# Compute total change for each condition
condition_1$total_change <- condition_1$Joint1_Distance + condition_1$Joint2_Distance + condition_1$Joint3_Distance +
  condition_1$Joint4_Distance + condition_1$Joint6_Distance + 
  condition_1$Joint7_Distance

condition_2$total_change <- condition_2$Joint1_Distance + condition_2$Joint2_Distance + condition_2$Joint3_Distance +
  condition_2$Joint4_Distance + condition_2$Joint6_Distance + 
  condition_2$Joint7_Distance

condition_3$total_change <- condition_3$Joint1_Distance + condition_3$Joint2_Distance + condition_3$Joint3_Distance +
  condition_3$Joint4_Distance + condition_3$Joint6_Distance + 
  condition_3$Joint7_Distance

condition_4$total_change <- condition_4$Joint1_Distance + condition_4$Joint2_Distance + condition_4$Joint3_Distance +
  condition_4$Joint4_Distance + condition_4$Joint6_Distance + 
  condition_4$Joint7_Distance

condition_5$total_change <- condition_5$Joint1_Distance + condition_5$Joint2_Distance + condition_5$Joint3_Distance +
  condition_5$Joint4_Distance + condition_5$Joint6_Distance + 
  condition_5$Joint7_Distance

condition_6$total_change <- condition_6$Joint1_Distance + condition_6$Joint2_Distance + condition_6$Joint3_Distance +
  condition_6$Joint4_Distance + + condition_6$Joint6_Distance + 
  condition_6$Joint7_Distance

condition_7$total_change <- condition_7$Joint1_Distance + condition_7$Joint2_Distance + condition_7$Joint3_Distance +
  condition_7$Joint4_Distance + condition_7$Joint6_Distance + 
  condition_7$Joint7_Distance

condition_8$total_change <- condition_8$Joint1_Distance + condition_8$Joint2_Distance + condition_8$Joint3_Distance +
  condition_8$Joint4_Distance + condition_8$Joint6_Distance + 
  condition_8$Joint7_Distance

# Combine data frames and add a 'Condition' column
condition_1$Condition <- var1
condition_2$Condition <- var2
condition_3$Condition <- var3
condition_4$Condition <- var4
condition_5$Condition <- var5
condition_6$Condition <- var6
condition_7$Condition <- var7
condition_8$Condition <- var8

combined_data <- rbind(condition_1, condition_2, condition_3, condition_4, condition_5, condition_6, condition_7, condition_8)

get_answers <- function(condition_1, condition_2, condition_3, condition_4, condition_5, condition_6, condition_7, condition_8) {
  
  # Compute the average total change for each condition
  average_change_1 <- mean(condition_1$total_change)
  average_change_2 <- mean(condition_2$total_change)
  average_change_3 <- mean(condition_3$total_change)
  average_change_4 <- mean(condition_4$total_change)
  average_change_5 <- mean(condition_5$total_change)
  average_change_6 <- mean(condition_6$total_change)
  average_change_7 <- mean(condition_7$total_change)
  average_change_8 <- mean(condition_8$total_change)
  
  # Return the averages as a list
  return(list(average_change_1 = average_change_1, average_change_2 = average_change_2, 
              average_change_3 = average_change_3, average_change_4 = average_change_4,
              average_change_5 = average_change_5, average_change_6 = average_change_6,
              average_change_7 = average_change_7, average_change_8 = average_change_8))
}



# Call the function and store the results
results <- get_answers(condition_1, condition_2, condition_3, condition_4, condition_5, condition_6, condition_7, condition_8)

# Extract the average changes
average_change_1 <- results$average_change_1
average_change_2 <- results$average_change_2
average_change_3 <- results$average_change_3
average_change_4 <- results$average_change_4
average_change_5 <- results$average_change_5
average_change_6 <- results$average_change_6
average_change_7 <- results$average_change_7
average_change_8 <- results$average_change_8

# Print the results
cat("Average change for ", var1, " = ", average_change_1, ".\n")
cat("Average change for ", var2, " = ", average_change_2, ".\n")
cat("Average change for ", var3, " = ", average_change_3, ".\n")
cat("Average change for ", var4, " = ", average_change_4, ".\n")
cat("Average change for ", var5, " = ", average_change_5, ".\n")
cat("Average change for ", var6, " = ", average_change_6, ".\n")
cat("Average change for ", var7, " = ", average_change_7, ".\n")
cat("Average change for ", var8, " = ", average_change_8, ".\n")

target <- clean_data |> subset(attribute == var)

# Proceed with the rest of the analysis

# Specify conditions
target$condition <- factor(target$condition, levels = c('A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'))

print(paste0("One-Way ANOVA for ", var))
res.aov <- target |> 
  anova_test(value ~ condition) %>% adjust_pvalue(method = "bonferroni")

anova_table <- get_anova_table(res.aov)

# Export the ANOVA table to a CSV file
anova_filename <- paste0("One_way_ANOVA_results_combo.csv")
#write_csv(anova_table, file.path(folder_path, anova_filename))

print(anova_table)

print(paste0("Repeated measures ANOVA for ", var))
res.raov <- target |> distinct() |> # distinct removes duplicate rows
  anova_test(value ~ condition+ Error(id/condition)) |>
  adjust_pvalue(method = "bonferroni")

ranova_table <- get_anova_table(res.raov)

# Export the ANOVA table to a CSV file
ranova_filename <- paste0("Repeated_Measures_ANOVA_results_combo.csv")
#write_csv(ranova_table, file.path(folder_path, ranova_filename))

print(ranova_table)

# T-tests
print(paste0("T-test for ", var))
# compares the means of value across the conditions to determine if there is a difference
# bonferroni correction adjusts the significance level to account for the number of comparisons
# the correction reduces the chance of a false positive ("it's significant!" when it's not)
pwc <- target %>%
  t_test(
    value ~ condition,
    p.adjust.method = "bonferroni"
  )
# Filter for specific comparisons: A vs B, C vs D, E vs F, G vs H
specific_comparisons <- pwc |> 
  filter((group1 == "A" & group2 == "H") |
           (group1 == "B" & group2 == "D") |
           (group1 == "E" & group2 == "F") |
           (group1 == "G" & group2 == "H")) |>
  select(group1, group2, p.adj.signif, p.adj)

# Define y-axis limits to prevent altering the range
y_max <- max(target$value, na.rm = TRUE)
y_limits <- c(min(target$value, na.rm = TRUE), y_max + 1)  # Adding space for annotations

# Define a fixed y-position for the p-values to avoid affecting the box plots
fixed_y_position <- y_max + 17 # Adjust as necessary to avoid overlap

# Export the t-test results to a CSV file
ttest_filename <- paste0("t_test_results_combo.csv")
write_csv(pwc, file.path(folder_path, ttest_filename))
print(pwc)


# Create the box plot
plot_boxplot <- function(data) {
    p <- data |> na.omit() |>  # Remove any rows with NA values
      ggplot(aes(x = Condition, y = total_change, fill = Condition)) +
      geom_boxplot(width = 0.7, color = "black", alpha = 0.7, outlier.size = 2, outlier.shape = 16) +  # Show outliers and adjust size
      stat_summary(fun = mean, geom = "point", size = 3, shape = 23, fill = "white", color = "black") + # Mean points with distinct style
      xlab("CONDITION") +
      ylab("Total Change in Position") +
      ggtitle("Total Change in Position by Condition", subtitle = paste0("Comparison of ", var1, ", ", var2, ", ", var3, ", ", var4, ", ", var5, ", ", var6, ", ", var7, ", ", var8)) +
      theme_minimal(base_family = "Inter", base_size = 14) +
      theme(
        panel.background = element_rect(fill = "whitesmoke", color = NA),
        plot.background = element_rect(fill = "white", color = NA),
        panel.border = element_rect(color = "black", fill = NA, size = 1),
        axis.title.x = element_text(face = "bold", size = 12),
        axis.title.y = element_text(face = "bold", size = 12),
        axis.text.x = element_text(angle = 45, hjust = 1, vjust = 1, size = 10), # Rotate x-axis labels to prevent overlap
        plot.title = element_text(face = "bold", size = 16, hjust = 0.5),
        plot.subtitle = element_text(size = 12, hjust = 0.5),
        legend.position = "none"
      )
    # Add significance annotations for the specific comparisons
    for (i in 1:nrow(specific_comparisons)) {
      p <- p + geom_signif(
        comparisons = list(c(specific_comparisons$group1[i], specific_comparisons$group2[i])),
        annotations = paste0("p = ", round(specific_comparisons$p.adj[i], 3)),  # Show the exact p-value
        map_signif_level = FALSE,  # Disable default significance symbols
        y_position = fixed_y_position  # Fixed y-position for all p-values
      )
    }
    
    return(p)
}

# Check combined_data to ensure columns exist
str(combined_data)
head(combined_data)

# Generate and display the plot
plot_boxplot(combined_data)

# Generate the plot and store it in a variable
combined_plot <- plot_boxplot(combined_data)
plot_filename <- (paste0("PLOT_totalJointDistance.png"))

# Save the plot to a file
ggsave(filename = plot_filename, plot = combined_plot, width = 8, height = 6, dpi = 300)

