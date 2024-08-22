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
yloadfonts(device = "win")
fonts()
library(readr)

#------------------------------------------------------------------
#README!!
#It might be worthwhile to make the csv an input to some of these functions instead
#of the way they're currently setup, as to consider we might have multiple (one for Qualtrics, one for processed bags)
#The comments here aren't the best but R is a pretty well documented resource if u fish around.
#We can also alternatively just talk through some of this. 

#------------------------------------------------------------------
# Should be noted, the actual ANOVA tables are not included in the actual code but here's a few examples
# of their use. You'll want to add them at some point at the end (or maybe in the functions themselves if it
# seems more convenient). If you look up the specific function online that will likely help. This is
#probably the most complex example you'll see since this was a mixed model ANOVA meaning there
#was both a between and within subjects component of the study. 

# Setups up rANOVA -- when using with ur own data we're just within, remove between
#res.aov <- anova_test(
# data = unfair, dv = score, wid = id,
# between = watson,
# within = condition
# ) %>% adjust_pvalue(method="bonferroni")
# get_anova_table(res.aov)


# normal ANOVA --- this is an ANOVA after the rANOVA, so looking within groups to see effects
# Again the study this was pulled from was a bit weird so we likely won't need this
# one.way <- unfair %>%
#   group_by(condition) %>%
#   anova_test(dv = score, wid = V293, between = watson) %>%
#   get_anova_table() %>%
#   adjust_pvalue(method = "bonferroni")
# one.way


# T-tests -- Essentially when you want to see efffects across conditions, we run this if our
# rANOVA proves to be significant (p corrected<0.05)
# pwc <- unfair %>%
#   group_by(condition) %>%
#   t_test(
#     score ~ watson,
#     p.adjust.method = "bonferroni"
#   )
# pwc

#Without context they're a bit hard to understand but let me know if you have any questions.
#If you haven't already reviewed the primer again that's a good resource. There should be a few
# online R resources that should help too. Its a pretty commonly used platform. Well-documented.
#These tables in particular will output ur F, p, and mean values of ur data as neccessary. 
#This is pretty good reference for using these functions: 
# https://www.datanovia.com/en/lessons/repeated-measures-anova-in-r/

#At a basic level, we're trying to see if the data we're collecting across these different
#conditions are actually coming from different distributions. 

#----------------------------------------------------------------------------------------
#MAIN DATA SOURCE HERE ---- REPLACE (DAMIEN)
#Before you do that I recommend going through to clean the data. We won't need to do this
#for the robot data but for the qualtrics data this will need to happen. We can chat about this.
#We would need to essentially do an intial pass in a program called Jamovi first where we'll remove
# a lot of fluff information that isn't essential for us to process the data and also account for
# any erroneous chunks. 

clean_data <- read_csv("C:/Users/nnamd/Documents/research/omic/nameofurfile.csv")

#A lot of what I've semi-implemented here assume you're 

#Creates a double Boxplot to display 2 factors simultaneously
double_boxplot <- function(phrase){
  
  #Condition to omit, cleans data, ensures numeric, and preps for plotting
  do_not_include <- paste('T -',phrase)
  target <- clean_data[ , grepl(phrase , names( clean_data ) ) ]%>%
    mutate_all(as.numeric)%>% select(-contains(do_not_include))
  for_row_names <- names(target)
  target <- cbind(target,rownames(target))
  target$id <- clean_data$ResponseId
  
  
  #Cleaning/Processing Data for Tests, Convert id, conditions into factors for plotting and ANOVA setup
  new_target <- target %>%
    gather(key = "category", value = "total", for_row_names) %>%
    separate(category, into=c('condition','type'),remove=TRUE, extra='merge') %>%
    mutate(across(where(is.character), str_remove_all, pattern = fixed(" ")))%>%
    convert_as_factor(id, condition, type) 
  
  #Setting up graphs
  #new_target$condition <- factor(new_target$condition , levels=c('B', 'LB', 'TD', 'B2', 'UT', 'LF'))
  return(qplot(x=type, y=total, data = new_target[complete.cases(new_target),], geom="boxplot", fill=type)+geom_boxplot()+
           scale_fill_npg() + xlab("CATEGORY") + ylab("VALUE") +stat_summary(fun.y=mean, geom="point", size=4,shape=3,position=position_dodge(width=0.75))+
           theme(text = element_text(size = 12, family="Times New Roman"),panel.border = element_rect(color = "black", fill = NA, size = 1))+
           ylim(0,5) + facet_wrap(~condition)+ stat_summary(fun.y=mean, geom="point", size=4,shape=3,position=position_dodge(width=0.75))+
           scale_y_continuous(breaks=1:5, labels=c('Min','','','', 'Max'), limits=c(1,5))
         
  )}


single_boxplot <- function(phrase, opposite){
  #Setup Target Data 
  do_not_include <- paste('T -',phrase)
  target <- clean_data[ , grepl( phrase , names( clean_data ) ) ]%>%
    mutate_all(as.numeric) %>% select(-contains(do_not_include))
  
  
  for_row_names <- names(target)
  target <- cbind(target,rownames(target))
  target$id <- clean_data$ResponseId
  
  #Cleaning/Processing Data for Tests
  new_target <- target %>%
    gather(key = "category", value = "total", for_row_names) %>%
    separate(category, into=c('condition','type'),remove=TRUE, extra='merge') %>%
    mutate(across(where(is.character), str_remove_all, pattern = fixed(" ")))%>%
    convert_as_factor(id, condition, type) 
  
  new_target$condition <- factor(new_target$condition , levels=c('B', 'B2', 'LB', 'UT', 'TD','LF'))
  
  return(qplot(x=condition, y=total, data = new_target[complete.cases(new_target),], geom="boxplot", fill=condition)+ geom_boxplot()+
           scale_fill_npg() + xlab("CONDITION") + ylab("VALUE") + ylim(0,9) +
           theme(text = element_text(size = 12, family="Times New Roman"),panel.border = element_rect(color = "black", fill = NA, size = 1))+ 
           stat_summary(fun.y=mean, geom="point", size=4,shape=3,position=position_dodge(width=0.75))+
           scale_y_continuous(breaks=1:9, labels=c('Min','','','','','','','', 'Max'), limits=c(1,9))
  ) 
  
}
par(font.lab = 3)

#Graphing and Saving plots
double_boxplot('URGENCY')+
  scale_x_discrete(breaks=c("URGENCY_1","URGENCY_2","URGENCY_3"), labels=c("Urgency", "Ignore", "Error"))+ guides(fill="none")
ggsave('urgency.png',units='in',width=8.6, height=3)
single_boxplot('VALENCE', TRUE)+ guides(fill="none")
ggsave('valence.png',units='in',width=8, height=3)
single_boxplot('AROUSAL', TRUE)+ guides(fill="none")
ggsave('arousal.png',units='in',width=8, height=3)
single_boxplot('DOMINANCE',FALSE)+ guides(fill="none")
ggsave('dominance.png',units='in',width=8, height=3)

