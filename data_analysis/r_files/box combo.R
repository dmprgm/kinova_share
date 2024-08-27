# Load required libraries
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
  # Convert data to long format
  pivot_longer(!c(id, condition), names_to = "attribute", values_to = "value")

# Filter data to include only conditions A and B
clean_data <- clean_data |> filter(condition %in% c("A", "B"))

# Create a box plot for all variables on a single graph
p <- clean_data |> na.omit() |>
  ggplot(aes(x = attribute, y = value, fill = condition)) +
  geom_boxplot(width = 0.7, color = "black", alpha = 0.7, outlier.size = 2, outlier.shape = 16) +  # Show outliers and adjust size
  stat_summary(fun = mean, geom = "point", position = position_dodge(0.7), size = 3, shape = 23, fill = "white", color = "black") + # Mean points with distinct style
  scale_fill_npg() +
  xlab("Attribute") +
  ylab("Value") +
  ggtitle("Comparison of A and B Across All Attributes") +
  theme_minimal(base_family = "Inter", base_size = 14) +
  theme(
    panel.background = element_rect(fill = "whitesmoke", color = NA),
    plot.background = element_rect(fill = "white", color = NA),
    panel.border = element_rect(color = "black", fill = NA, size = 1),
    axis.title.x = element_text(face = "bold", size = 12),
    axis.title.y = element_text(face = "bold", size = 12),
    axis.text.x = element_text(angle = 45, hjust = 1, vjust = 1, size = 10), # Rotate x-axis labels to prevent overlap
    plot.title = element_text(face = "bold", size = 16, hjust = 0.5),
    legend.position = "right"
  ) 

# Save the plot
folder_path <- "C:/Users/Student/Documents/AllPlots_P4"
if (!file.exists(folder_path)) {
  dir.create(folder_path)
}

plot_filename <- "Combined_Plot_All_Attributes.png"
ggsave(filename = file.path(folder_path, plot_filename), plot = p, units = 'in', width = 12, height = 8)
