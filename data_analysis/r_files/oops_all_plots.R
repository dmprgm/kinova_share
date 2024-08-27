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
library(ggsignif)

# Load the CSV file
clean_data <- read_csv("C:/Users/Student/Documents/ConfidenceFilter/robot_data.csv") |>
  # attribute becomes another label and the values are long rather than wide
  pivot_longer(!c(id,condition), names_to = "attribute", values_to = "value")

# see structure by looking at the first several rows:
head(clean_data)

# List of variables to process
variables <- c("Yaw", "Pitch", "Roll", "TwistX", "TwistY", "TwistZ", "AvgVelocity", "MaxVelocity",
               "AvgAccel", "MaxAccel", "AvgArea", "COGZ", "PathLengthDifference", "RangeX", 
               "RangeY", "RangeZ", "TimeElapsed", "Joint7_Distance", "Joint6_Distance", 
               "Joint5_Distance", "Joint4_Distance", "Joint3_Distance", "Joint2_Distance", 
               "Joint1_Distance")

# Function to create a box plot for a specific metric and save ANOVAs and t-tests
single_boxplot <- function(var){
  # Setup target data by filtering for variable
  # expects pivot_longer() format
  target <- clean_data |> subset(attribute == var)
  
  # Proceed with the rest of the analysis
  
  # Specify conditions
  target$condition <- factor(target$condition, levels = c('A', 'H', 'D', 'E', 'B', 'G', 'C', 'F'))
  
  # Setup up one-way and rANOVA -- when using with your data we're just within, remove between
  # the strategy in ANOVA is to estimate overall variance and w/in groups variance
  # two-way ANOVA investigates whether adding some explanatory variable explains a meaningful amount of variance
  # repeated measures ANOVA compares mean scores across multiple observations of the same subjects under diff conditions
  
  # t-tests help examine whether certain factor levels' observations are significantly different than others,
  # for example A might be different from D and F but those may not be different from each other
  
  print(paste0("One-Way ANOVA for ", var))
  res.aov <- target |> 
    anova_test(value ~ condition) %>% adjust_pvalue(method = "bonferroni")
  
  anova_table <- get_anova_table(res.aov)
  
  # Export the ANOVA table to a CSV file
  #anova_filename <- paste0("One_way_ANOVA_results_", var, ".csv")
  #write_csv(anova_table, anova_filename)
  
  print(anova_table)
  
  print(paste0("Repeated measures ANOVA for ", var))
  res.raov <- target |> distinct() |> # distinct removes duplicate rows
    anova_test(value ~ condition+ Error(id/condition)) |>
    adjust_pvalue(method = "bonferroni")
  
  ranova_table <- get_anova_table(res.raov)
  
  # Export the ANOVA table to a CSV file
  #ranova_filename <- paste0("Repeated_Measures_ANOVA_results_", var, ".csv")
  #write_csv(ranova_table, ranova_filename)
  
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
  
  # Filter for specific comparisons: 
  specific_comparisons <- pwc |> 
    filter((group1 == "A" & group2 == "B") |
             (group1 == "A" & group2 == "E") |
             (group1 == "B" & group2 == "G") |
             (group1 == "C" & group2 == "F")) |>
    select(group1, group2, p.adj.signif, p.adj)
  
  # Define y-axis limits to prevent altering the range
  y_max <- max(target$value, na.rm = FALSE)
  y_limits <- c(min(target$value, na.rm = TRUE), y_max + 3)  # Adding space for annotations
  
  # Define a fixed y-position for the p-values to avoid affecting the box plots
  fixed_y_position <- y_max  # Adjust as necessary to avoid overlap
  
  # Export the t-test results to a CSV file
  #ttest_filename <- paste0("t_test_results_", var, ".csv")
  #write_csv(pwc, ttest_filename)
  print(pwc)
  
  # Plot formatting
  p <- target |> na.omit() |>
    ggplot(aes(x = condition, y = value, fill = condition)) +
    geom_boxplot(width = 0.7, color = "black", alpha = 0.7, outlier.size = 2, outlier.shape = 16) +  # Show outliers and adjust size
    stat_summary(fun = mean, geom = "point", size = 3, shape = 23, fill = "white", color = "black") + # Mean points with distinct style
    scale_fill_npg() +
    xlab("CONDITION") +
    ylab("VALUE") +
    ggtitle("User Input Comparison", subtitle = var) +
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

# Loop through each variable
for (var in variables) {
  # Define the path to save results for the current variable
  folder_path <- paste0("C:/Users/Student/Documents/AllPlots_P4")
  if (!file.exists(folder_path)) {
    dir.create(folder_path)
  }
  
  name <- single_boxplot(var) #+ guides(fill="none")
  plot_filename <- paste0("PLOT_", var, ".png")
  ggsave(filename = file.path(folder_path, plot_filename), units = 'in', width = 5, height = 5)
  
}

