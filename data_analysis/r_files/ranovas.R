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

# Generate boxplot for a specific metric, e.g., Yaw
var <- 'Roll'

folder_path <- paste0("C:/Users/Student/Documents/kinova_share/data_analysis/OUTPUTS/rANOVAs/")
if (!file.exists(folder_path)) {
  dir.create(folder_path)
}

# Function to create a box plot for a specific metric and save ANOVAs and t-tests
single_boxplot <- function(var) {
  # Setup target data by filtering for the variable
  target <- clean_data |> subset(attribute == var)
  
  # Specify conditions
  target$condition <- factor(target$condition, levels = c('A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'))
  
  # One-Way ANOVA for the entire dataset
  print(paste0("One-Way ANOVA for ", var))
  res.aov <- target |> 
    anova_test(value ~ condition) %>%
    adjust_pvalue(method = "bonferroni")
  
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

name <- single_boxplot(var) #+ guides(fill="none")




