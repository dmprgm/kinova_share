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
clean_data <- read_csv("C:/Users/Student/Documents/kinova_share/data_analysis/pull_from/ConfidenceFilter/robot_data.csv") |>
  # attribute becomes another label and the values are long rather than wide
  pivot_longer(!c(id,condition), names_to = "attribute", values_to = "value")

# see structure by looking at the first several rows:
head(clean_data)

# List of variables to process
variables <- c("Yaw", "Pitch", "Roll", "AvgVelocity", "MaxVelocity",
               "AvgAccel", "MaxAccel", "AvgArea", "COGZ", "PathLengthDifference", "RangeX", 
               "RangeY", "RangeZ", "TimeElapsed", "Joint7_Distance", "Joint6_Distance", 
               "Joint4_Distance", "Joint3_Distance", "Joint2_Distance", 
               "Joint1_Distance")

# Function to create a box plot for a specific metric and save ANOVAs and t-tests
single_boxplot <- function(var){
  # Setup target data by filtering for variable
  # expects pivot_longer() format
  target <- clean_data |> subset(attribute == var)
  
  # Proceed with the rest of the analysis
  
  # Specify conditions
  target$condition <- factor(target$condition, levels = c('A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'))
  
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
  
  print(anova_table)
  
  # List of pairs to compare
  condition_pairs <- list(c("A", "B"), c("C", "D"), c("E", "F"), c("G", "H"))
  
  for (pair in condition_pairs) {
    pair_name <- paste(pair, collapse = "_vs_")
    
    print(paste0("Repeated measures ANOVA for ", pair_name))
    
    # Subset the data for the current pair of conditions
    target_pair <- target |> filter(condition %in% pair)
    
    # Perform the rANOVA
    res.raov <- target_pair |> 
      anova_test(dv = value, wid = id, within = condition) %>%
      adjust_pvalue(method = "bonferroni")
    
    ranova_table <- get_anova_table(res.raov)
    
    # Export the ANOVA table to a CSV file
    ranova_filename <- paste0("RANOVA_results_", var, "_", pair_name, ".csv")
    write_csv(ranova_table, file.path(folder_path, ranova_filename))
    
    print(ranova_table)
  }
  
}


# Loop through each variable
for (var in variables) {
  # Define the path to save results for the current variable
  folder_path <- paste0("C:/Users/Student/Documents/kinova_share/data_analysis/OUTPUTS/rANOVAs/")
  if (!file.exists(folder_path)) {
    dir.create(folder_path)
  }
  
  name <- single_boxplot(var) #+ guides(fill="none")
  plot_filename <- paste0("PLOT_", var, ".png")
  ggsave(filename = file.path(folder_path, plot_filename), units = 'in', width = 5, height = 5)
  
}
