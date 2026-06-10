# train_classifier.py
import numpy as np
import joblib
from sklearn.svm import SVC
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import make_pipeline
from sklearn.model_selection import train_test_split, GridSearchCV
from sklearn.metrics import classification_report, f1_score

def train_and_export_svm(X, y_raw, save_filename='best_voronoi_model.pkl', RANDOM_STATE=42):
    y = np.array([1 if s > 0 else 0 for s in y_raw])

    # Triple Split: 60% Train, 20% Validate, 20% Test
    X_temp, X_test, y_temp, y_test = train_test_split(
        X, y, 
        test_size=0.20, 
        random_state=RANDOM_STATE, 
        stratify=y
    )

    X_train, X_val, y_train, y_val = train_test_split(
        X_temp, y_temp, 
        test_size=0.25, 
        random_state=RANDOM_STATE, 
        stratify=y_temp
    )

    def print_distribution(name, labels):
        split_count = np.sum(labels)
        total = len(labels)
        percentage = (split_count / total) * 100
        print(f"{name} Set: {total} samples, {percentage:.1f}% Splits")

    print_distribution("Train", y_train)
    print_distribution("Val", y_val)
    print_distribution("Test", y_test)

    # Build standard pipeline
    pipe = make_pipeline(StandardScaler(), SVC(probability=True, random_state=RANDOM_STATE))
    
    # Grid Search parameter configurations
    param_grid = [
        {
            'svc__kernel': ['rbf'],
            'svc__C': [0.1, 1, 10, 100],
            'svc__gamma': ['scale', 0.1, 0.01],
            'svc__class_weight': ['balanced']
        },
        {
            'svc__kernel': ['linear'],
            'svc__C': [0.1, 1, 10],
            'svc__class_weight': ['balanced']
        }
    ]

    print("\nRunning Hyperparameter Grid Search...")
    grid = GridSearchCV(pipe, param_grid, cv=5, scoring='f1', n_jobs=-1)
    grid.fit(X_train, y_train)
    
    best_model = grid.best_estimator_
    print(f"Best Params Found: {grid.best_params_}")

    # Optimize Decision Threshold on validation data
    val_probs = best_model.predict_proba(X_val)[:, 1]
    best_f1 = 0
    best_thresh = 0.5
    
    for t in [0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6]:
        t_preds = (val_probs >= t).astype(int)
        score = f1_score(y_val, t_preds)
        if score > best_f1:
            best_f1 = score
            best_thresh = t
    
    print(f"Optimal Threshold found on Validation Data: {best_thresh} (F1: {best_f1:.4f})")
    
    # Final Model Verification on Blind Test Set
    test_probs = best_model.predict_proba(X_test)[:, 1]
    test_preds = (test_probs >= best_thresh).astype(int)

    print("\n" + "="*40)
    print(f"FINAL TEST SET RESULTS (Decision Threshold: {best_thresh})")
    print("="*40)
    print(classification_report(y_test, test_preds, target_names=['No Split', 'Split']))

    # Export the final unified Pipeline
    joblib.dump(best_model, save_filename)
    print(f"\nPipeline model successfully saved to: {save_filename}")

if __name__ == "__main__":
    X_v = np.load('X_voronoi.npy')
    y_v = np.load('y_local.npy')
    
    if len(y_v) > 0:
        train_and_export_svm(X_v, y_v)
    else:
        print("Error: Feature matrix is empty. Verify that dataset image directories are correct.")