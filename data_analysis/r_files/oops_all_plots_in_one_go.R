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
  pivot_longer(!c(id, condition), names_to = "attribute", values_to = "value")

# see structure by looking at the first several rows:
head(clean_data)

# List of variables to process
variables <- c("Yaw", "Pitch", "Roll", "AvgVelocity", "MaxVelocity",
               "AvgAccel", "MaxAccel", "AvgArea", "COGZ", "PathLengthDifference", "RangeX", 
               "RangeY", "RangeZ", "TimeElapsed", "Joint7_Distance", "Joint6_Distance", 
               "Joint4_Distance", "Joint3_Distance", "Joint2_Distance", 
               "Joint1_Distance")

# Create a single plot with facets for each variable
p <- clean_data |> 
  filter(attribute %in% variables) |> # Filter for relevant attributes
  na.omit() |>
  ggplot(aes(x = condition, y = value, fill = condition)) +
  geom_boxplot(width = 0.7, color = "black", alpha = 0.7, outlier.size = 2, outlier.shape = 16) +  # Show outliers and adjust size
  stat_summary(fun = mean, geom = "point", size = 3, shape = 23, fill = "white", color = "black") + # Mean points with distinct style
  scale_fill_npg() +
  xlab("CONDITION") +
  ylab("VALUE") +
  ggtitle("User Input Comparison") +
  theme_minimal(base_family = "Inter", base_size = 14) +
  theme(
    panel.background = element_rect(fill = "whitesmoke", color = NA),
    plot.background = element_rect(fill = "white", color = NA),
    panel.border = element_rect(color = "black", fill = NA, size = 1),
    axis.title.x = element_text(face = "bold", size = 12),
    axis.title.y = element_text(face = "bold", size = 12),
    axis.text.x = element_text(angle = 45, hjust = 1, vjust = 1, size = 10), # Rotate x-axis labels to prevent overlap
    plot.title = element_text(face = "bold", size = 16, hjust = 0.5),
    legend.position = "none"
  ) +
  facet_wrap(~ attribute, scales = "free_y")  # Create a facet for each attribute

# Save the combined plot
combined_plot_filename <- "C:/Users/Student/Documents/kinova_share/data_analysis/OUTPUTS/Sig2_AllPlots/Combined_Plots.png"
ggsave(filename = combined_plot_filename, plot = p, units = 'in', width = 15, height = 10)
