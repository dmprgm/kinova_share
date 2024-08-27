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

# Load the CSV file
clean_data <- read_csv("C:/Users/Student/Documents/kinova_share/data_analysis/robot_data.csv") |>
  pivot_longer(!c(id,condition), names_to = "attribute", values_to = "value")

# Filter data for condition 'G' and specific attributes
filtered_data <- clean_data %>%
  filter(condition == 'G') %>%
  filter(attribute %in% c('RangeX', 'RangeY', 'RangeZ'))

# Function to create a box plot for a specific metric and save ANOVAs and t-tests
single_boxplot <- function(var) {
  # Setup target data by filtering for variable
  target <- filtered_data |> subset(attribute == var)
  
  # Set factor levels for attribute (not condition)
  target$attribute <- factor(target$attribute, levels = c('RangeX', 'RangeY', 'RangeZ'))
  
  # Check if attribute has more than one level
  if (length(unique(target$attribute)) > 1) {
    # One-Way ANOVA
    print(paste0("One-Way ANOVA for ", var))
    res.aov <- target |> 
      anova_test(value ~ attribute) %>% adjust_pvalue(method = "bonferroni")
    
    anova_table <- get_anova_table(res.aov)
    anova_filename <- paste0("One_way_ANOVA_results_", var, ".csv")
    write_csv(anova_table, anova_filename)
    print(anova_table)
    
    # Repeated Measures ANOVA
    print(paste0("Repeated measures ANOVA for ", var))
    res.raov <- target |> distinct() |> 
      anova_test(value ~ attribute + Error(id/attribute)) |>
      adjust_pvalue(method = "bonferroni")
    
    ranova_table <- get_anova_table(res.raov)
    ranova_filename <- paste0("Repeated_Measures_ANOVA_results_", var, ".csv")
    write_csv(ranova_table, ranova_filename)
    print(ranova_table)
    
    # T-tests
    print(paste0("T-test for ", var))
    pwc <- target %>%
      t_test(
        value ~ attribute,
        p.adjust.method = "bonferroni"
      )
    
    ttest_filename <- paste0("t_test_results_", var, ".csv")
    write_csv(pwc, ttest_filename)
    print(pwc)
  } else {
    print(paste0("Skipping statistical tests for ", var, " because there is only one level in the attribute factor."))
  }
  
  # Plot formatting
  return(
    target |> na.omit() |>
      ggplot(aes(x = attribute, y = value, fill = attribute)) +
      geom_boxplot(width = 0.7, color = "black", alpha = 0.7, outlier.size = 2, outlier.shape = 16) + 
      stat_summary(fun = mean, geom = "point", size = 3, shape = 23, fill = "white", color = "black") +
      scale_fill_npg() +
      xlab("Attribute") +
      ylab("Value") +
      ggtitle("User Input Comparison", subtitle = var) +
      theme_minimal(base_family = "Inter", base_size = 14) +
      theme(
        panel.background = element_rect(fill = "whitesmoke", color = NA),
        plot.background = element_rect(fill = "white", color = NA),
        panel.border = element_rect(color = "black", fill = NA, size = 1),
        axis.title.x = element_text(face = "bold", size = 12),
        axis.title.y = element_text(face = "bold", size = 12),
        axis.text.x = element_text(angle = 45, hjust = 1, vjust = 1, size = 10), 
        plot.title = element_text(face = "bold", size = 16, hjust = 0.5),
        plot.subtitle = element_text(size = 12, hjust = 0.5),
        legend.position = "none"
      ) 
  )
}

# Generate boxplot for specific metrics
attributes <- c('RangeX', 'RangeY', 'RangeZ')
for (var in attributes) {
  plot <- single_boxplot(var)
  plot_filename <- paste0("PLOT_", var, ".png")
  ggsave(plot_filename, plot, units = 'in', width = 5, height = 5)
}

# Generate a combined plot:
combined_plot <- filtered_data |> 
  na.omit() |>
  ggplot(aes(x = attribute, y = value, fill = attribute)) +
  geom_boxplot(width = 0.7, color = "black", alpha = 0.7, outlier.size = 2, outlier.shape = 16) +
  stat_summary(fun = mean, geom = "point", size = 3, shape = 23, fill = "white", color = "black") +
  scale_fill_npg() +
  xlab("Attribute") +
  ylab("Value") +
  ggtitle("Comparison of RangeX, RangeY, and RangeZ for Condition 'G'") +
  theme_minimal(base_family = "Inter", base_size = 14) +
  theme(
    panel.background = element_rect(fill = "whitesmoke", color = NA),
    plot.background = element_rect(fill = "white", color = NA),
    panel.border = element_rect(color = "black", fill = NA, size = 1),
    axis.title.x = element_text(face = "bold", size = 12),
    axis.title.y = element_text(face = "bold", size = 12),
    axis.text.x = element_text(angle = 45, hjust = 1, vjust = 1, size = 10), 
    plot.title = element_text(face = "bold", size = 16, hjust = 0.5),
    plot.subtitle = element_text(size = 12, hjust = 0.5),
    legend.position = "none"
  )

# Save the plot
ggsave("PLOT_Comparison_G.png", combined_plot, units = 'in', width = 7, height = 5)

# Display the plot
print(combined_plot)


