library(ISLR)
library(glmnet)
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
set.seed(1)

lasso_ouput <- function(list){
  #Select Data of Interest (To Compare)
  clean_data <- read_csv("C:/Users/nnamd/Documents/GitHub/kinova_share/data_analysis/pull_from/ConfidenceFilter/robot_data.csv") |>
    filter(condition %in% list)
  #Setup Data
  y = as.factor(clean_data$condition) 
  x = model.matrix(condition~., clean_data[,-1])[,-1]
  #Setup Lasso Model
  lasso.cv = cv.glmnet(x, y, family='binomial')
  #lasso.cv$lambda.min
  #lasso.cv$lambda.1se 
  co<-coef(lasso.cv,s = "lambda.1se")
  inds<-which(co!=0)
  variables<-row.names(co)[inds]
  variables<-variables[!(variables %in% '(Intercept)')];
  return(variables);
}

print(lasso_ouput(c('A','B')))
print(lasso_ouput(c('C','D')))
print(lasso_ouput(c('E','F')))
print(lasso_ouput(c('G','H')))

clean_data <- read_csv("C:/Users/nnamd/Documents/GitHub/kinova_share/data_analysis/pull_from/ConfidenceFilter/robot_data.csv") |>
  filter(condition %in% c('A','B'))

y = as.factor(clean_data$condition) 
x = model.matrix(condition~., clean_data[,-1])[,-1] # Here we exclude the first column 
                                          # because it corresponds to the 
                                          # intercept.
head(x)
set.seed(1)
lasso.cv = cv.glmnet(x, y, family='binomial')
lasso.cv$lambda.min
lasso.cv$lambda.1se 
round(cbind(coef(lasso.cv, s ='lambda.min'), coef(lasso.cv,s='lambda.1se')),3)

# par(mfrow=c(1,2))
# plot(lasso.cv)
# lasso = glmnet(x, y,family='multinomial')
# plot(lasso, xvar = 'lambda', type.coef = "2norm")
# abline(v = log(lasso.cv$lambda.min), lty = 3) # careful to use the log here  
# # and below
# abline(v = log(lasso.cv$lambda.1se), lty = 3)


# repetitions = 50
# cor.1 = c()
# cor.2 = c()

set.seed(1)   
new_data <- clean_data[,-1]

co<-coef(lasso.cv,s = "lambda.1se")
co$A
inds<-which(co!=0)
variables<-row.names(co)[inds]
variables<-variables[!(variables %in% '(Intercept)')];
return(variables);

# for(i in 1:repetitions){
#   
#   # Step (i) data splitting
#   training.obs = sample(1:228,  150)
#   y.train = clean_data$condition[training.obs]
#   x.train = model.matrix(condition~., new_data[training.obs, ])[,-1]
#   y.test = clean_data$condition[-training.obs]
#   x.test = model.matrix(condition~., new_data[-training.obs, ])[,-1]
#   
#   # Step (ii) training phase
#   lasso.train = cv.glmnet(x.train, y.train,family='multinomial')
#   
#   # Step (iii) generating predictions
#   predict.1 = predict(lasso.train, x.test, s = 'lambda.min', type = "class")
#   predict.2 = predict(lasso.train, x.test, s = 'lambda.1se', type = "class")
#   y.test <- as.factor(y.test)
#   y.test <- as.numeric(y.test)
#   predict.1 <- as.factor(predict.1)
#   predict.1 <- as.numeric(predict.1)
#   predict.2 <- as.factor(predict.2)
#   predict.2 <- as.numeric(predict.2)
#   
#   # Step (iv) evaluating predictive performance
#   cor.1[i] = cor(y.test, predict.1)
#   cor.2[i] = cor(y.test, predict.2)
# }
# 
# boxplot(cor.1, cor.2, names = c('min-CV lasso','1-se lasso'), 
#         ylab = 'Test correlation', col = 7)


