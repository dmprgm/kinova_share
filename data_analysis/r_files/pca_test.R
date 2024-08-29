library(FactoMineR)
library(corrr)
library(ggcorrplot)
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
library(factoextra)

# Load the CSV file
clean_data <- read_csv("C:/Users/nnamd/Documents/GitHub/kinova_share/data_analysis/robot_data.csv") %>% filter(condition %in% c('A', 'B'))  
# attribute becomes another label and the values are long rather than wide
  # pivot_longer(!c(id,condition), names_to = "attribute", values_to = "value")

# see structure by looking at the first several rows:
head(clean_data)
numerical_data <- clean_data[,9:26]
head(numerical_data)
data_normalized <- scale(numerical_data)
head(data_normalized)

data.pca <- PCA(data_normalized, graph=FALSE)
fviz_pca_ind(data.pca,
             geom.ind = "point", # show points only (nbut not "text")
             col.ind = clean_data$condition, # color by groups
             palette = c("#00AFBB", "#E7B800"),
            addEllipses = TRUE,
             legend.title = "Groups") 

