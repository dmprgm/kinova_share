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


clean_data <- read_csv("C:/Users/Student/Documents/kinova_share/qualtrics2.csv")


phrase <- '_1'
subtitle <- paste0("Question 1: Confidence in Perceivability")

single_boxplot <- function(phrase){
  #Setup Target Data 
  target <- clean_data[ , grepl(phrase , names( clean_data ) ) ]%>%
    mutate_all(as.numeric)
  
  for_row_names <- names(target)
  target <- cbind(target,rownames(target))
  target$id <- clean_data$ResponseId
  
  #Cleaning/Processing Data for Tests
  new_target <- target %>%
    gather(key = "category", value = "total", for_row_names) %>%
    separate(category, into=c('condition','type'), remove=TRUE, extra='merge') %>%
    mutate(across(where(is.character), str_remove_all, pattern = fixed(" "))) %>%
    convert_as_factor(id, condition, type) 
  new_target$condition <- factor(new_target$condition , levels=c('Bound', 'Free', 'Direct', 'Indirect', 'Sudden', 'Sustained', 'Strong', 'Light'))
  
  # Setups up rANOVA -- when using with ur own data we're just within, remove between
  res.aov <- anova_test(
    data = new_target, dv = total, wid = id,
    within = condition
  ) %>% adjust_pvalue(method="bonferroni")
  anova_table <- get_anova_table(res.aov)
  
  # Export the ANOVA table to a CSV file
  anova_filename <- paste0("rANOVA_results", phrase, ".csv")
  write_csv(anova_table, anova_filename)
  
  print(anova_table)
  
  # T-tests -- Essentially when you want to see effects across conditions, we run this if our
  # rANOVA proves to be significant (p corrected<0.05)
  pwc <- new_target %>%
    t_test(
      total~condition,
      p.adjust.method = "bonferroni"
    )
  
  # Export the t-test results to a CSV file
  ttest_filename <- paste0("t_test_results", phrase, ".csv")
  write_csv(pwc, ttest_filename)
  print(pwc)
  
  # plot formatting
  return(
    ggplot(data = new_target[complete.cases(new_target),], aes(x = condition, y = total, fill = condition)) + 
      geom_boxplot(width = 0.7, color = "black", alpha = 0.7, outlier.size = 2, outlier.shape = 16) +  # Show outliers and adjust size
      stat_summary(fun = mean, geom = "point", size = 3, shape = 23, fill = "white", color = "black") + # Mean points with distinct style
      scale_fill_npg() + 
      xlab("CONDITION") + 
      ylab("VALUE (LIKERT-SCALE)") + 
      ggtitle("User Input Comparison", subtitle) +
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
      ) +
      scale_y_continuous(breaks = 1:7, labels = c('1', '', '', '', '', '', '7'), limits = c(1, 7)) # Custom y-axis labels and limits
  ) 
  
}
par(font.lab = 3)

#Graphing and Saving plots

name <- single_boxplot(phrase)+ guides(fill="none")
plot_filename <- paste0("PLOT", phrase, ".png")
ggsave(plot_filename, units = 'in', width=5,height=5)


