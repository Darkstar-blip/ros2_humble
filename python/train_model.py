import pandas as pd
import joblib
from sklearn.model_selection import train_test_split
from sklearn.tree import DecisionTreeClassifier
from sklearn.pipeline import Pipeline
from sklearn.compose import ColumnTransformer
from sklearn.preprocessing import OneHotEncoder

# Load dataset
data = pd.read_csv('project_dataset.csv')

# Separate features and label
X = data[['Time of Day', 'Task Type', 'Dirt Level']]
y = data['Target Room']

# Define preprocessing for categorical columns
preprocessor = ColumnTransformer(
    transformers=[
        ('cat', OneHotEncoder(), ['Time of Day', 'Task Type', 'Dirt Level'])
    ]
)

# Create pipeline
pipeline = Pipeline(steps=[
    ('preprocessor', preprocessor),
    ('classifier', DecisionTreeClassifier())
])

# Train-test split
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Train
pipeline.fit(X_train, y_train)

# Evaluate
print("Accuracy:", pipeline.score(X_test, y_test))

# Save the entire pipeline (preprocessor + model)
joblib.dump(pipeline, 'room_predictor.joblib')
