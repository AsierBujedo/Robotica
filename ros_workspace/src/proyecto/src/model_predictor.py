import joblib
import pandas as pd


# Cargar modelos
def load_models(model_path, encoder_path):
    clf = joblib.load(model_path)
    le = joblib.load(encoder_path)
    return clf, le

# Crear DataFrame con las características
def create_feature_dataframe(features, expected_features):
    X_new = pd.DataFrame([features])
    missing_features = set(expected_features) - set(X_new.columns)
    for feature in missing_features:
        X_new[feature] = 0
    X_new = X_new[expected_features]
    return X_new

# Predecir clase
def predict_class(clf, le, features):
    X_new = create_feature_dataframe(features, clf.feature_names_in_)
    prediction_encoded = clf.predict(X_new)
    return le.inverse_transform(prediction_encoded)[0]
