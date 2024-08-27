# Load required libraries
library(ggsci)
library(RColorBrewer)
library(tidyverse)
library(ggpubr)
library(rstatix)
library(magrittr)
library(ggplot2)
library(extrafont)
library(tidyr)
library(stringr)
library(readr)
library(ggsignif)

## Idk what I was doing here, I wanted to see if this would be anything


# Font handling
font_import()
loadfonts(device = "win")

# Load the CSV file and pivot to long format
clean_data <- read_csv("C:/Users/Student/Documents/kinova_share/data_analysis/robot_data.csv") |>
  pivot_longer(!c(id,condition), names_to = "attribute", values_to = "value")

# List of variables to process
variables <- c("TwistX", "TwistY", "TwistZ", "AvgVelocity", "MaxVelocity",
               "AvgAccel", "MaxAccel", "AvgArea", "PathLengthDifference", "RangeX", 
               "RangeY", "RangeZ", "TimeElapsed", "Joint7_Distance", "Joint6_Distance", 
               "Joint4_Distance", "Joint3_Distance", "Joint2_Distance", 
               "Joint1_Distance")

# Function to create a box plot for a specific metric and save ANOVAs and t-tests
single_boxplot <- function(var){
  # Filter the data for the specific variable and conditions A and B
  target <- clean_data |> 
    filter(attribute == var & condition %in% c('A', 'B')) |> 
    na.omit()
  
  # Ensure conditions are factorized
  target$condition <- factor(target$condition, levels = c('A', 'B'))
  
  # Perform the t-test with Bonferroni correction
  pwc <- target %>%
    t_test(
      value ~ condition,
      p.adjust.method = "bonferroni"
    )
  
  # Check if p.adj exists and is numeric, if not, use p-value instead
  if ("p.adj" %in% colnames(pwc)) {
    pwc$p.adj <- as.numeric(pwc$p.adj)
    p_value <- pwc$p.adj[1]
  } else if ("p" %in% colnames(pwc)) {
    p_value <- pwc$p[1]
  } else {
    stop("No p-value found in t-test results.")
  }
  
  # Export the t-test results to a CSV file
  ttest_filename <- paste0("t_test_results_", var, ".csv")
  write_csv(pwc, ttest_filename)
  
  # Define y-axis limits to prevent altering the range
  y_max <- max(target$value, na.rm = TRUE)
  
  # Plot formatting
  p <- target |>
    ggplot(aes(x = condition, y = value, fill = condition)) +
    geom_boxplot(width = 0.7, color = "black", alpha = 0.7, outlier.size = 2, outlier.shape = 16) +  # Show outliers and adjust size
    stat_summary(fun = mean, geom = "point", size = 3, shape = 23, fill = "white", color = "black") + # Mean points with distinct style
    scale_fill_npg() +
    xlab("CONDITION") +
    ylab("VALUE") +
    ggtitle("Comparison of A vs B", subtitle = var) +
    theme_minimal(base_family = "Inter", base_size = 14) +
    theme(
      panel.background = element_rect(fill = "whitesmoke", color = NA),
      plot.background = element_rect(fill = "white", color = NA),
      panel.border = element_rect(color = "black", fill = NA, linewidth = 1),
      axis.title.x = element_text(face = "bold", size = 12),
      axis.title.y = element_text(face = "bold", size = 12),
      axis.text.x = element_text(angle = 45, hjust = 1, vjust = 1, size = 10), # Rotate x-axis labels to prevent overlap
      plot.title = element_text(face = "bold", size = 16, hjust = 0.5),
      plot.subtitle = element_text(size = 12, hjust = 0.5),
      legend.position = "none"
    ) 
  
  # Add significance annotations for the comparison between A and B
  p <- p + geom_signif(
    comparisons = list(c("A", "B")),
    annotations = paste0("p = ", round(p_value, 3)),  # Show the exact p-value
    map_signif_level = FALSE,  # Disable default significance symbols
    y_position = y_max + 0.1 * y_max  # Fixed y-position for all p-values
  )
  
  return(p)
}

# Loop through each variable to generate and save plots
for (var in variables) {
  # Define the path to save results for the current variable
  folder_path <- file.path("C:/Users/Student/Documents/TestResultsvXXX", var)
  
  # Create directory if it does not exist
  if (!dir.exists(folder_path)) {
    dir.create(folder_path, recursive = TRUE)
  }
  
  # Generate the plot
  p <- single_boxplot(var)
  
  # Save the plot
  plot_filename <- paste0("PLOT_", var, ".png")
  ggsave(filename = file.path(folder_path, plot_filename), plot = p, units = 'in', width = 5, height = 5)
}
