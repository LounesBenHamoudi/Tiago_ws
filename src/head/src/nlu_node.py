#!/usr/bin/env python3
import rospy
import joblib
from pathlib import Path
import yaml

from std_msgs.msg import String
from auto_nav.msg import Command

# chemins relatifs au package
PKG_DIR = Path(__file__).resolve().parent.parent  # .../auto_nav
NLU_DIR = PKG_DIR / "src" / "nlu"
MODEL_DIR = NLU_DIR / "nlu_model"
MODEL_PATH = MODEL_DIR / "intent_classifier.joblib"
DATASET_YAML = NLU_DIR / "dataset.yaml"

def load_model():
    if not MODEL_PATH.exists():
        rospy.logerr(f"[NLU] Modèle introuvable : {MODEL_PATH}")
        raise FileNotFoundError(MODEL_PATH)
    rospy.loginfo(f"[NLU] Chargement du modèle : {MODEL_PATH}")
    return joblib.load(MODEL_PATH)

def load_slot_values():
    if not DATASET_YAML.exists():
        rospy.logwarn(f"[NLU] dataset.yaml introuvable : {DATASET_YAML}")
        return {}

    with open(DATASET_YAML, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    slot_values = {}
    for intent, intent_data in data.get("intents", {}).items():
        if "slot_values" in intent_data:
            for slot_name, values in intent_data["slot_values"].items():
                if slot_name not in slot_values:
                    slot_values[slot_name] = set()
                slot_values[slot_name].update(values)

    rospy.loginfo(f"[NLU] Slots disponibles : {slot_values}")
    return slot_values

def predict_intent(model, text: str):
    intent = model.predict([text])[0]
    return intent

def extract_slots(text: str, slot_values):
    text_lower = text.lower()
    extracted = {}

    for slot_name, values in slot_values.items():
        for val in values:
            if val.lower() in text_lower:
                extracted[slot_name] = val
                break

    return extracted

class NLURosNode:
    def __init__(self):
        self.model = load_model()
        self.slot_values = load_slot_values()

        self.sub = rospy.Subscriber("/tiago/voice_input", String,
                                    self.cb_voice, queue_size=10)
        self.pub = rospy.Publisher("/tiago/command", Command,
                                   queue_size=10)

        rospy.loginfo("[NLU] Node NLU initialisé, en attente de commandes...")

    def cb_voice(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        intent = predict_intent(self.model, text)
        slots = extract_slots(text, self.slot_values)

        cmd = Command()
        cmd.intent = intent
        cmd.place = slots.get("place", "")
        cmd.raw_text = text

        rospy.loginfo(f"[NLU] '{text}' -> intent={cmd.intent}, place={cmd.place}")
        self.pub.publish(cmd)

if __name__ == "__main__":
    rospy.init_node("nlu_node")
    try:
        node = NLURosNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
