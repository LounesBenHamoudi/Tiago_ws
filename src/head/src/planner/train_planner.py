#!/usr/bin/env python3
import json
from pathlib import Path

from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.linear_model import LogisticRegression
from sklearn.pipeline import Pipeline
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report
import joblib

DATA_DIR = Path(__file__).resolve().parents[2] / "data" / "planner"
DATA_PATH = DATA_DIR / "planner_dataset.jsonl"

MODEL_DIR = Path(__file__).resolve().parents[2] / "models" / "planner"
MODEL_PATH = MODEL_DIR / "planner_policy.joblib"

def load_jsonl(path: Path):
    X, y = [], []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            ex = json.loads(line)
            X.append(ex["x"])
            y.append(ex["y"])
    return X, y

def main():
    X, y = load_jsonl(DATA_PATH)
    print(f"✅ Dataset chargé: {len(X)} exemples, {len(set(y))} actions")

    clf = Pipeline([
        ("tfidf", TfidfVectorizer(ngram_range=(1,2))),  # un peu mieux que (1,1)
        ("logreg", LogisticRegression(max_iter=1500))
    ])

    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.2, random_state=42, stratify=y
    )

    clf.fit(X_train, y_train)
    y_pred = clf.predict(X_test)

    print("=== Rapport ===")
    print(classification_report(y_test, y_pred, zero_division=0))

    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    joblib.dump(clf, MODEL_PATH)
    print(f"✅ Modèle sauvegardé: {MODEL_PATH}")

if __name__ == "__main__":
    main()
