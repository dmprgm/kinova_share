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

# Load the CSV file
clean_data <- read_csv("C:/Users/Student/Documents/kinova_share/data_analysis/robot_data.csv") |>
  # attribute becomes another label and the values are long rather than wide
  pivot_longer(!c(id,condition), names_to = "attribute", values_to = "value")

# see structure by looking at the first several rows:
head(clean_data)

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
  
  # Export the ANOVA table to a CSV file
  anova_filename <- paste0("One_way_ANOVA_results_", var, ".csv")
  write_csv(anova_table, anova_filename)
  
  print(anova_table)
  
  print(paste0("Repeated measures ANOVA for ", var))
  res.raov <- target |> distinct() |> # distinct removes duplicate rows
    anova_test(value ~ condition+ Error(id/condition)) |>
    adjust_pvalue(method = "bonferroni")
  
  ranova_table <- get_anova_table(res.raov)
  
  # Export the ANOVA table to a CSV file
  ranova_filename <- paste0("Repeated_Measures_ANOVA_results_", var, ".csv")
  write_csv(ranova_table, ranova_filename)
  
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
  
  # Export the t-test results to a CSV file
  ttest_filename <- paste0("t_test_results_", var, ".csv")
  write_csv(pwc, ttest_filename)
  print(pwc)
  
  # Plot formatting
  return(
  target |> na.omit() |>
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
  )
}

# Generate boxplot for a specific metric, e.g., Yaw
var <- 'TwistX'
name <- single_boxplot(var) #+ guides(fill="none")
plot_filename <- paste0("PLOT_", var, ".png")
ggsave(plot_filename, units = 'in', width = 5, height = 5)



