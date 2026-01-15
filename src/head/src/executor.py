#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from auto_nav.msg import Command

def build_state(goal_intent: str, goal_args: dict, history: list, last_status: str, retry_count: int):
    args_str = " ".join([f"{k}={v}" for k, v in goal_args.items()])
    hist_str = " ; ".join(history) if history else "EMPTY"
    return f"GOAL={goal_intent} {args_str} | LAST_STATUS={last_status} | RETRY={retry_count} | HISTORY={hist_str}"


class ExecutorNode:
    def __init__(self):
        rospy.init_node("executor_node")

        self.goal_intent = None
        self.goal_args = {}
        self.history = []
        self.last_status = "START"
        self.retry_count = 0


        self.current_action = None
        self.waiting_action = False
        self.waiting_result = False

        self.pub_state = rospy.Publisher("/tiago/planner/state", String, queue_size=10)
        self.pub_action = rospy.Publisher("/tiago/action", String, queue_size=10)

        rospy.Subscriber("/tiago/command", Command, self.goal_cb, queue_size=10)
        rospy.Subscriber("/tiago/planner/next_action", String, self.next_action_cb, queue_size=10)
        rospy.Subscriber("/tiago/action_result", String, self.result_cb, queue_size=10)

        rospy.loginfo("[EXEC] Ready. Waiting for /tiago/command (goal)")

    def command_to_goal(self, cmd: Command):
        intent = cmd.intent.strip()
        place = cmd.place.strip() if cmd.place else ""

        if intent == "GO_TO":
            return "GO_TO", {"place": place}
        if intent == "COME_BACK":
            return "COME_BACK", {}
        if intent in ("GO_TOUR", "GUIDED_TOUR"):
            return "GUIDED_TOUR", {"tour": "default", "shuffle": "False"}

        return "UNKNOWN", {}

    def goal_cb(self, cmd: Command):
        goal_intent, goal_args = self.command_to_goal(cmd)
        if goal_intent == "UNKNOWN":
            rospy.logwarn("[EXEC] Unknown goal: %s", cmd.intent)
            return

        # reset episode
        self.goal_intent = goal_intent
        self.goal_args = goal_args
        self.history = []
        self.last_status = "START"
        self.current_action = None
        self.waiting_action = False
        self.waiting_result = False
        self.retry_count = 0


        rospy.loginfo("[EXEC] New goal: %s %s", self.goal_intent, self.goal_args)
        self.ask_planner()

    def ask_planner(self):
        if self.goal_intent is None:
            return
        state = build_state(self.goal_intent, self.goal_args, self.history, self.last_status, self.retry_count)
        self.waiting_action = True
        rospy.loginfo("[EXEC] -> planner_state")
        self.pub_state.publish(String(data=state))

    def next_action_cb(self, msg: String):
        if not self.waiting_action:
            return
        action = msg.data.strip()
        if not action:
            return

        self.waiting_action = False

        if action == "STOP":
            rospy.loginfo("[EXEC] Planner returned STOP. Done ✅")
            self.goal_intent = None
            return
        if action == "SKIP":
            rospy.logwarn("[EXEC] Planner decided SKIP.")
            # On note un skip “générique”
            self.history.append("SKIP")
            self.last_status = "SUCCESS"
            self.retry_count = 0
            self.ask_planner()
            return


        self.current_action = action
        self.waiting_result = True
        rospy.loginfo("[EXEC] -> action: %s", action)
        self.pub_action.publish(String(data=action))

    def result_cb(self, msg: String):
        if not self.waiting_result or not self.current_action:
            return

        res = msg.data.strip()
        rospy.loginfo("[EXEC] <- result: %s", res)

        # V1 parsing:
        # "SUCCESS action=..."
        # "FAILED action=... reason=..."
        if res.startswith("SUCCESS"):
            self.last_status = "SUCCESS"
            self.retry_count = 0
            self.history.append(self.current_action)
            self.current_action = None
            self.waiting_result = False
            self.ask_planner()

        elif res.startswith("FAILED"):
            # Si échec sur une action:
            # retry une fois, sinon demander SKIP, sinon STOP (le modèle apprendra)
            if self.retry_count == 0:
                rospy.logwarn("[EXEC] FAILED -> retry once.")
                self.last_status = "FAILED"
                self.retry_count = 0  # état = retry=0 avant de proposer la même action
                # On NE met pas dans history (action pas terminée)
                self.waiting_result = False
                # demander au planner : il devrait renvoyer la même action (retry)
                self.ask_planner()
                self.retry_count = 1
                return

    # retry déjà tenté
            rospy.logwarn("[EXEC] FAILED again -> ask planner to SKIP/STOP.")
            self.last_status = "FAILED"
            self.history.append(self.current_action + " [FAILED_TWICE]")
            self.current_action = None
            self.waiting_result = False
            # Ici, on laisse le planner décider SKIP ou STOP (selon dataset)
            self.ask_planner()
            self.retry_count = 0


        else:
            rospy.logwarn("[EXEC] Unknown result format: %s", res)

if __name__ == "__main__":
    ExecutorNode()
    rospy.spin()
