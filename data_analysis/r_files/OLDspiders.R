# Install and load the necessary package
library(fmsb)
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

#Prepare the data
data <- data.frame(
  TwistX = c(0.109, 0.054, 0.085, 0.042, 0.049, 0.043, 0.052, 0.082),
  TwistY = c(0.01, 0.004, 0.01, 0.002, 0.006, 0.005, 0.004, 0.012),
  TwistZ = c(-0.071, -0.052, -0.084, -0.045, -0.045, -0.045, -0.053, -0.078),
  AvgVelocity = c(1.571, 0.656, 0.897, 1.553, 1.73, 0.891, 0.699, 1.563),
  MaxVelocity = c(6.023, 3.448, 3.455, 6.741, 6.694, 4.902, 3.641, 6.478),
  AvgAccel = c(24.457, 8.917, 11.224, 29.535, 25.998, 10.22, 9.393, 26.073),
  MaxAccel = c(276.869, 116.25, 112.214, 296.412, 288.893, 119.153, 123.737, 237.969),
  RangeY = c(0.063, 0.049, 0.049, 0.244, 0.35, 0.079, 0.098, 0.168),
  RangeZ = c(0.35, 0.302, 0.306, 0.41, 0.464, 0.367, 0.341, 0.349),
  TimeElapsed = c(3.829, 6.239, 3.411, 8.393, 8.135, 7.911, 6.908, 4.321),
  PathLengthDifference = c(0.33, 0.18, 0.11, 0.77, 1.35, 0.41, 0.30, 0.53)
  
)

clean_data <- read_csv("C:/Users/Student/Documents/ConfidenceFilter/robot_data.csv") |>
  # Convert data to long format
  pivot_longer(!c(id, condition), names_to = "attribute", values_to = "value")

y <- clean_data %>% group_by(condition) %>% summarise(mean=mean(value), sd=sd(value))
# Set row names as conditions

rownames(data) <- c("A", "B", "C", "D", "E", "F", "G", "H")

stat_summary(data)

mean(data)

# Normalize the data by adding max and min rows
data_normalized <- rbind(
  apply(data, 2, max), # max values
  apply(data, 2, min), # min values
  data
)

# Function to generate radar charts for two conditions
generate_radar_chart <- function(cond1, cond2, filename) {
  
  png(filename, width = 800, height = 800)
  
  par(mar = c(2, 2, 2, 2))
  
  selected_data <- data_normalized[c(1, 2, cond1 + 2, cond2 + 2), ] # Select max, min, and two conditions
  radarchart(
    selected_data,
    axistype = 1,
    pcol = c("red", "blue"),
    pfcol = c( 
              rgb(1, 0.6, 0.6, alpha = 0.75), # Light red with 50% opacity
              rgb(0.6, 0.6, 1, alpha = 0.75)  # Light blue with 50% opacity
              ),
    plwd = 2,
    cglcol = "grey",
    cglty = 1,
    axislabcol = rgb(1, 1, 1, alpha = 0),
    cglwd = 0.8,
    vlcex = 2
  )
  
  # Add legend
  legend(
    "topright",
    legend = rownames(data)[c(cond1, cond2)],
    col = c("red", "blue"),
    lty = 1,
    lwd = 2,
    bty = "n",
    cex = 2
  )
  dev.off()
}

# Loop through pairs of conditions and generate radar charts
for (i in 1:(nrow(data) - 1)) {
  for (j in (i + 1):nrow(data)) {
    cat("Comparing Condition", rownames(data)[i], "and Condition", rownames(data)[j], "\n")
    filename <- paste0(rownames(data)[i], "_vs_", rownames(data)[j], ".png")
    generate_radar_chart(i, j, filename)
    
  }
}
