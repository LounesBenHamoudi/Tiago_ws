#!/usr/bin/env python3
import yaml
from pathlib import Path

from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.linear_model import LogisticRegression
from sklearn.pipeline import Pipeline
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report
import joblib

# ============================================================
# 1. Charger le YAML compact
# ============================================================

DATASET_YAML = "dataset.yaml"   # adapte le chemin si besoin
MODEL_DIR = Path("nlu_model")
MODEL_DIR.mkdir(exist_ok=True)

def load_dataset_from_yaml(path):
    """
    Charge le YAML de la forme :
    intents:
      GO_TO:
        slots: ["place"]
        slot_values:
          place: ["porte", "table", "cuisine"]
        templates:
          - "va à la {place}"
          - ...
      COME_BACK:
        slots: []
        templates:
          - "reviens ici"
          - ...
    et génère une liste (texts, labels)
    """
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    intents = data.get("intents", {})
    texts = []
    labels = []

    for intent_name, intent_data in intents.items():
        templates = intent_data.get("templates", [])
        slots = intent_data.get("slots", [])
        slot_values = intent_data.get("slot_values", {})

        # Cas 1 : pas de slots → on prend les templates tels quels
        if not slots:
            for tmpl in templates:
                texts.append(tmpl)
                labels.append(intent_name)
            continue

        # Cas 2 : il y a des slots → on génère toutes les combinaisons
        # Pour l'instant on gère simplement : chaque slot est remplacé
        # par toutes ses valeurs, indépendamment des autres.
        # On fait un produit cartésien simple.
        from itertools import product

        # Construire une liste des listes de valeurs dans l'ordre des slots
        slot_values_lists = []
        for slot_name in slots:
            values = slot_values.get(slot_name, [])
            if not values:
                # Si un slot n'a pas de valeurs définies, on met une valeur vide
                values = [f"{{{slot_name}}}"]
            slot_values_lists.append(values)

        # Produit cartésien de toutes les valeurs possibles pour les slots
        for tmpl in templates:
            for combo in product(*slot_values_lists):
                text = tmpl
                # Remplacer chaque {slot} par la valeur correspondante
                for slot_name, slot_val in zip(slots, combo):
                    text = text.replace("{" + slot_name + "}", slot_val)
                texts.append(text)
                labels.append(intent_name)

    return texts, labels

# ============================================================
# 2. Entraîner le classifieur
# ============================================================

def train_intent_classifier(texts, labels):
    # Pipeline simple : TF-IDF + Logistic Regression
    clf = Pipeline([
        ("tfidf", TfidfVectorizer()),
        ("logreg", LogisticRegression(max_iter=1000, class_weight="balanced"))
    ])

    X_train, X_test, y_train, y_test = train_test_split(
        texts, labels, test_size=0.2, random_state=42, stratify=labels
    )

    clf.fit(X_train, y_train)

    # Évaluation rapide
    y_pred = clf.predict(X_test)
    print("=== Rapport de performance ===")
    print(classification_report(y_test, y_pred))

    return clf

# ============================================================
# 3. Sauvegarde du modèle
# ============================================================

def save_model(model):
    model_path = MODEL_DIR / "intent_classifier.joblib"
    joblib.dump(model, model_path)
    print(f"Modèle sauvegardé dans : {model_path}")

# ============================================================
# Main
# ============================================================

if __name__ == "__main__":
    texts, labels = load_dataset_from_yaml(DATASET_YAML)
    print(f"{len(texts)} exemples chargés, intents : {set(labels)}")

    model = train_intent_classifier(texts, labels)
    save_model(model)
