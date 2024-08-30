library(fmsb)
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
library(readr)
library(ggsignif)
font_import()
loadfonts(device = "win")
fonts()


clean_data <- read_csv("C:/Users/Student/Documents/kinova_share/data_analysis/pull_from/Jointsno5/robot_data.csv") |>
  # Convert data to long format
  pivot_longer(!c(id, condition), names_to = "attribute", values_to = "value")

generate <- function(cond){
  filtered <- clean_data |> filter(condition %in% c(cond))
  y <- filtered %>% group_by(attribute) %>% summarise(mean=mean(value), sd=sd(value))
  y_filename <- paste0("sd_", cond, ".csv")
  y_folder_path <- "C:/Users/Student/Documents/Cond_Sds"
  
  m_list <- setNames(y$mean, y$attribute)
  cat(m_list)
  
  return(m_list)
}

# empty list to store results
results_list <- list()

# Set row names as conditions
condition <- c("A", "B", "C", "D", "E", "F", "G", "H")

for (cond in condition) {
  results_list[[cond]] <- generate(cond)
}

# combine all the data frames into one data frame
data <- do.call(rbind, results_list)
data <- as.data.frame(data)  # Ensure data is in data frame format

#Function to normalize the data
normalize_data <- function(data) {
  maximum <- max(data)
  minimum <- 0
  normalized_data <- as.data.frame(lapply(data, function(x) {
    (x) / (maximum)
  }))
  return(normalized_data)
}



# Function to check the range of normalized data
validate_normalization <- function(normalized_data) {
  range_check <- apply(normalized_data, 2, function(x) {
    minimum >= 0 && maximum <= 1
  })
  return(range_check)
}

# attempt
normalized_data <- normalize_data(data)
range_check <- validate_normalization(normalized_data)
print(range_check)

as.numeric()

# Function to generate radar charts
generate_radar_chart <- function(cond1, cond2, filename) {
  # Path to save radar charts
  folder_path <- "C:/Users/Student/Documents/kinova_share/data_analysis/OUTPUTS/Joint_Spiders"
  if (!dir.exists(folder_path)) {
    dir.create(folder_path)
  }
  
  png(filename = file.path(folder_path, filename), width = 800, height = 800)
  
  par(cex = 0.7, mar = c(5, 5, 5, 5))
  
  
  data_normalized <- normalize_data(data)
  data_to_plot <- rbind(rep(1, ncol(data_normalized)), rep(0, ncol(data_normalized)), data_normalized[cond1, ], data_normalized[cond2, ])
  colnames(data_to_plot) <- colnames(data)
  
  
  #selected_data <- data_normalized[c(1, 2, cond1 + 2, cond2 + 2), ] # Select max, min, and two conditions
  radarchart(
    data_to_plot,
    axistype = 1,
    pcol = c("yellow", "purple"),
    pfcol = c( 
      rgb(1, 0.929, 0.341, alpha = 0.75), # Light yellow with 75% opacity
      rgb(0.451, 0.369, 1, alpha = 0.75)  # Light purple with 75% opacity
    ),
    plwd = 2,
    cglcol = "grey",
    cglty = 1,
    axislabcol = rgb(1, 1, 1, alpha = 0),
    cglwd = 0.8,
    vlcex = 1.25
  )
  
  # Add legend
  legend(
    "topright",
    legend = c(condition[cond1], condition[cond2]),
    col = c("yellow", "purple"),
    lty = 1,
    lwd = 2,
    bty = "n",
    cex = 2.5
  )
  dev.off()
}

# set up data for radar chart
#data <- do.call(rbind, lapply(conditions, generate))
rownames(data) <- data$attribute
column_to_rownames(data) <-data$cond
data$attribute <- NULL
data$cond <- NULL


rownames(data) <- condition  # Set row names of data to condition names

# loop through the rows to generate radar charts
for (i in 1:(nrow(data) - 1)) {
  for (j in (i + 1):nrow(data)) {
    cat("Comparing Condition", rownames(data)[i], "and Condition", rownames(data)[j], "\n")
    
    # Use row names to label charts
    filename <- paste0(rownames(data)[i], "_vs_", rownames(data)[j], ".png")
    
    # Assuming 'generate_radar_chart' is a function you have defined to create radar charts
    generate_radar_chart(i, j, filename)
  }
}

