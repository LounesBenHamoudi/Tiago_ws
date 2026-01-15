#!/usr/bin/env python3
import joblib
from pathlib import Path
import yaml

MODEL_DIR = Path("nlu_model")
MODEL_PATH = MODEL_DIR / "intent_classifier.joblib"
DATASET_YAML = "dataset.yaml"   # pour charger les valeurs des slots

# =============================================================
# Charger le modèle + slot values
# =============================================================

def load_model():
    if not MODEL_PATH.exists():
        raise FileNotFoundError(f"Modèle introuvable : {MODEL_PATH}")
    return joblib.load(MODEL_PATH)

def load_slot_values():
    """Charge les valeurs des slots dans dataset.yaml"""
    with open(DATASET_YAML, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    slot_values = {}
    for intent, intent_data in data["intents"].items():
        if "slot_values" in intent_data:
            for slot_name, values in intent_data["slot_values"].items():
                if slot_name not in slot_values:
                    slot_values[slot_name] = set()
                slot_values[slot_name].update(values)

    return slot_values  # ex: {"place": {"porte","table","cuisine"}}

# =============================================================
# Prédire l'intent
# =============================================================

def predict_intent(model, text: str):
    intent = model.predict([text])[0]
    proba = model.predict_proba([text])[0]
    return intent, proba

# =============================================================
# Extraire les slots de la phrase
# =============================================================

def extract_slots(text: str, slot_values):
    """
    Retourne un dict avec les slots détectés dans la phrase, ex:
    {"place": "porte"}
    """
    text_lower = text.lower()
    extracted = {}

    # Pour chaque slot, tester si une valeur connue apparaît dans la phrase
    for slot_name, values in slot_values.items():
        for val in values:
            if val.lower() in text_lower:
                extracted[slot_name] = val
                break

    return extracted

# =============================================================
# Main
# =============================================================

if __name__ == "__main__":
    model = load_model()
    slot_values = load_slot_values()

    while True:
        try:
            text = input("Commande (Ctrl+C pour quitter) : ").strip()
            if not text:
                continue

            # Prédiction
            intent, proba = predict_intent(model, text)

            # Extraction des slots
            slots = extract_slots(text, slot_values)

            print("\n===================================")
            print(f"Phrase : {text}")
            print(f"Intent prédit : {intent}")
            print(f"Probabilités : {proba}")
            print(f"Slots extraits : {slots}")
            print("===================================\n")

        except KeyboardInterrupt:
            print("\nBye !")
            break
