# Data Processing
import pandas as pd
import numpy as np

# Modelling
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score, confusion_matrix, precision_score, recall_score, ConfusionMatrixDisplay
from sklearn.model_selection import RandomizedSearchCV, train_test_split
from scipy.stats import randint
import matplotlib.pyplot as plt
# Tree Visualisation
from sklearn.tree import export_graphviz
from IPython.display import Image
import graphviz

class RunRandomForest:
    def __init__(self, path):
        self.path = path
        self.data = pd.read_csv(path)
        self.copy = self.data
    
    def getFactors(self, conditions):
        data_copy = self.data.drop('id', axis=1)[self.data['condition'].str.contains('|'.join(conditions))]
        X = data_copy.drop('condition', axis=1)
        y = data_copy['condition'].map({conditions[0]:0,conditions[1]:1})
        # Split the data into training and test sets
        X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.3)
        #Get best hyperparamters
        
        hypers = self.getHypers(X_train,y_train)
        #Recreate RF
        rf = hypers
        #rf.fit(X_train, y_train)
        y_pred = rf.predict(X_test)
        accuracy = accuracy_score(y_test, y_pred)
        print("Accuracy:", accuracy)
        result = self.extractAndGraph(rf,X, X_train, X_test, y_train, y_test)
        return result

    def reset(self):
        self.data = pd.read_csv(self.path)

    def getHypers(self, X_train, y_train):
        param_dist = {'n_estimators': randint(50,500),
              'max_depth': randint(1,20)}
        # Create a random forest classifier
        rf = RandomForestClassifier(random_state=123)
        # Use random search to find the best hyperparameters
        rand_search = RandomizedSearchCV(rf, 
                                        param_distributions = param_dist, 
                                        n_iter=5, 
                                        cv=5)
        # Fit the random search object to the data
        rand_search.fit(X_train, y_train)
        best_rf = rand_search.best_estimator_
        return best_rf
    
    def extractAndGraph(self, rf, X, X_train, X_test, y_train, y_test):
        # Extract feature importances
        importances = rf.feature_importances_
        feature_names = X.columns
        feature_importance_df = pd.DataFrame({'Feature': feature_names, 'Importance': importances})

        # Rank features by importance
        feature_importance_df = feature_importance_df.sort_values(by='Importance', ascending=False)
        print(feature_importance_df)

        # Select top N features (example selecting top 10 features)
        top_features = feature_importance_df['Feature'][:5].values
        X_train_selected = X_train[top_features]
        X_test_selected = X_test[top_features]

        # Train the Random Forest model with selected features
        rf_selected = RandomForestClassifier(n_estimators=100, random_state=42)
        rf_selected.fit(X_train_selected, y_train)

        # Evaluate the model
        accuracy_after = rf_selected.score(X_test_selected, y_test)
        print(f'Accuracy after feature selection: {accuracy_after:.2f}')
        return feature_importance_df

fig, axes = plt.subplots(nrows=2, ncols=2)
rf = RunRandomForest('C:\\Users\\nnamd\\Documents\\GitHub\\kinova_share\\data_analysis\\robot_data.csv')

rf.getFactors(['A','B']).plot(ax= axes[0,0], kind='bar',x='Feature',y='Importance',title='Weight')
rf.reset()
rf.getFactors(['C','D']).plot(ax= axes[1,0], kind='bar',x='Feature',y='Importance',title='Space')
rf.reset()
rf.getFactors(['E','F']).plot(ax= axes[1,1], kind='bar',x='Feature',y='Importance',title='Flow')
rf.reset()
rf.getFactors(['G','H']).plot(ax= axes[0,1], kind='bar',x='Feature',y='Importance',title='Time')
plt.show()

