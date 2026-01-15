#!/usr/bin/env python3
import rospy
import joblib
from pathlib import Path
from std_msgs.msg import String

MODEL_PATH = Path(__file__).resolve().parents[2] / "models" / "planner" / "planner_policy.joblib"

class PlannerModelNode:
    def __init__(self):
        rospy.init_node("planner_model_node")

        if not MODEL_PATH.exists():
            raise FileNotFoundError(f"Model not found: {MODEL_PATH}")

        self.model = joblib.load(MODEL_PATH)
        rospy.loginfo("[PLANNER] Loaded model: %s", str(MODEL_PATH))

        self.pub_next = rospy.Publisher("/tiago/planner/next_action", String, queue_size=10)
        rospy.Subscriber("/tiago/planner/state", String, self.state_cb, queue_size=10)

        rospy.loginfo("[PLANNER] Ready. Listening /tiago/planner/state")

    def state_cb(self, msg: String):
        state = msg.data.strip()
        if not state:
            return
        action = self.model.predict([state])[0]
        self.pub_next.publish(String(data=action))
        rospy.loginfo("[PLANNER] state -> %s", action)

if __name__ == "__main__":
    PlannerModelNode()
    rospy.spin()
