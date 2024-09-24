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
clean_data <- read_csv("C:/Users/nnamd/Documents/GitHub/kinova_share/data_analysis/robot_data.csv") |>
  # attribute becomes another label and the values are long rather than wide
  pivot_longer(!c(id,condition), names_to = "attribute", values_to = "value")

# see structure by looking at the first several rows:
head(clean_data)

# List of variables to process
variables <- c("NrPks")

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
  
  # # Export the ANOVA table to a CSV file
  # anova_filename <- paste0("One_way_ANOVA_results_", var, ".csv")
  # write_csv(anova_table, file.path(folder_path, anova_filename))
  
  print(anova_table)
  
  print(paste0("Repeated measures ANOVA for ", var))
  res.raov <- target |> distinct() |> # distinct removes duplicate rows
    anova_test(value ~ condition+ Error(id/condition)) |>
    adjust_pvalue(method = "bonferroni")
  
  ranova_table <- get_anova_table(res.raov)
  
  # Export the ANOVA table to a CSV file
  ranova_filename <- paste0("Repeated_Measures_ANOVA_results_", var, ".csv")
  # write_csv(ranova_table, file.path(folder_path, ranova_filename))
  
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
    filter((group1 == "A" & group2 == "H") |
             (group1 == "C" & group2 == "G") |
             (group1 == "E" & group2 == "D") |
             (group1 == "G" & group2 == "H")) |>
    select(group1, group2, p.adj.signif, p.adj)
  
  # Define y-axis limits to prevent altering the range
  y_max <- max(target$value, na.rm = TRUE)
  y_limits <- c(min(target$value, na.rm = TRUE), y_max + 1)  # Adding space for annotations
  
  # Define a fixed y-position for the p-values to avoid affecting the box plots
  fixed_y_position <- y_max  # Adjust as necessary to avoid overlap
  
  # Export the t-test results to a CSV file
  ttest_filename <- paste0("t_test_results_", var, ".csv")
  write_csv(pwc, file.path(folder_path, ttest_filename)) 
  print(pwc, n=28)
  
  

  
}

# Loop through each variable
for (var in variables) {
  # Define the path to save results for the current variable
  folder_path <- paste0("C:/Users/nnamd/Documents/GitHub/kinova_share/data_analysis/OUTPUTS/rANOVAs")
  if (!file.exists(folder_path)) {
    dir.create(folder_path)
  }
  
  name <- single_boxplot(var) #+ guides(fill="none")

}
