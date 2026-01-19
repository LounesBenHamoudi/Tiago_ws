#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rospy
from std_msgs.msg import String

from pal_interaction_msgs.msg import TtsActionGoal
from actionlib_msgs.msg import GoalID


class SpeakerNode:
    def __init__(self):
        rospy.init_node("speaker")

        # --- TTS (PAL) via action topics ---
        self.tts_goal_pub = rospy.Publisher("/tts/goal", TtsActionGoal, queue_size=10)
        self.tts_cancel_pub = rospy.Publisher("/tts/cancel", GoalID, queue_size=10)

        # Pub/Sub action bus
        self.pub_result = rospy.Publisher("/tiago/action_result", String, queue_size=10)
        rospy.Subscriber("/tiago/action", String, self.action_cb, queue_size=10)

        # LLM bridge
        self.pub_llm_req = rospy.Publisher("/tiago/speaker/llm_request", String, queue_size=10)
        rospy.Subscriber("/tiago/speaker/llm_response", String, self.llm_response_cb, queue_size=10)

        self.pending_action = None
        self.waiting_llm = False

        # Params
        self.qr_topic = rospy.get_param("~qr_topic", "/qr_text")
        self.qr_timeout = float(rospy.get_param("~qr_timeout", 6.0))
        self.llm_timeout = float(rospy.get_param("~llm_timeout", 8.0))

        self.lang_id = rospy.get_param("~lang_id", "fr_FR")
        self.speaker_name = rospy.get_param("~speaker_name", "")
        self.wait_before_speaking = float(rospy.get_param("~wait_before_speaking", 0.0))

        rospy.sleep(0.2)  # laisse le temps aux pubs/subs
        rospy.loginfo("[SPEAKER] Ready. Using /tts/goal (PAL TTS). Listening /tiago/action")

    def parse_action(self, s: str):
        parts = s.strip().split()
        if not parts:
            return None, {}
        t = parts[0].upper()
        args = {}
        for p in parts[1:]:
            if "=" in p:
                k, v = p.split("=", 1)
                args[k.strip()] = v.strip()
        return t, args

    def _unique_goal_id(self) -> str:
        # Unique, même si appelé plusieurs fois par seconde
        # Exemple: speaker-1700000000-123456
        return f"speaker-{int(time.time())}-{int(time.time_ns() % 1_000_000_000)}"

    def say(self, text: str, lang_id: str = None):
        text = (text or "").strip()
        if not text:
            return

        gid = self._unique_goal_id()

        msg = TtsActionGoal()
        msg.header.stamp = rospy.Time.now()
        msg.goal_id.stamp = rospy.Time.now()
        msg.goal_id.id = gid

        # On utilise rawtext (comme le web)
        msg.goal.rawtext.text = text
        msg.goal.rawtext.lang_id = (lang_id or self.lang_id)

        # Champs optionnels
        msg.goal.speakerName = self.speaker_name
        msg.goal.wait_before_speaking = self.wait_before_speaking

        # (optionnel) remplir msg.goal.text, mais pas nécessaire pour rawtext
        msg.goal.text.section = ""
        msg.goal.text.key = ""
        msg.goal.text.lang_id = ""
        msg.goal.text.arguments = []

        self.tts_goal_pub.publish(msg)
        rospy.loginfo("[SPEAKER] TTS goal sent id=%s text=%s", gid, text)

    def cancel_tts(self):
        # Cancel “tout” (id vide + stamp 0) : pratique si tu veux couper la parole
        cancel = GoalID()
        cancel.stamp = rospy.Time(0)
        cancel.id = ""
        self.tts_cancel_pub.publish(cancel)

    def finish(self, ok: bool, action_str: str, reason: str = ""):
        if ok:
            self.pub_result.publish(String(data=f"SUCCESS action={action_str}"))
        else:
            rs = f"FAILED action={action_str}"
            if reason:
                rs += f" reason={reason}"
            self.pub_result.publish(String(data=rs))

        self.pending_action = None
        self.waiting_llm = False

    def action_cb(self, msg: String):
        if self.pending_action is not None:
            return  # V1: ignore if already busy

        action_str = msg.data.strip()
        if not action_str:
            return

        action_type, args = self.parse_action(action_str)

        # ---- Route 1: SAY ----
        if action_type == "SAY":
            mode = args.get("mode", "tts").lower()
            text = args.get("text", "")

            if mode == "llm":
                if not text:
                    self.finish(False, action_str, "missing_text")
                    return

                self.pending_action = action_str
                self.waiting_llm = True

                self.pub_llm_req.publish(String(data=text))
                rospy.Timer(rospy.Duration(self.llm_timeout), self.llm_timeout_cb, oneshot=True)
                rospy.loginfo("[SPEAKER] -> LLM request: %s", text)
                return

            # mode tts simple (sans LLM)
            if not text:
                self.finish(False, action_str, "missing_text")
                return

            self.pending_action = action_str
            self.say(text)
            self.finish(True, action_str)
            return

        # ---- Route 2: READ_QR ----
        if action_type == "READ_QR":
            self.pending_action = action_str
            rospy.loginfo("[SPEAKER] Waiting QR on %s ...", self.qr_topic)
            try:
                qr_msg = rospy.wait_for_message(self.qr_topic, String, timeout=self.qr_timeout)
                qr_text = qr_msg.data.strip()
                if not qr_text:
                    self.say("Je n'ai rien lu.")
                    self.finish(False, action_str, "empty_qr")
                    return

                self.say(qr_text)
                self.finish(True, action_str)
                return
            except rospy.ROSException:
                self.say("Je n'ai pas réussi à lire le QR code.")
                self.finish(False, action_str, "qr_timeout")
                return

        # Not for me -> ignore
        return

    def llm_timeout_cb(self, _evt):
        if self.waiting_llm and self.pending_action is not None:
            action_str = self.pending_action
            self.say("Je n'ai pas pu répondre à temps.")
            self.finish(False, action_str, "llm_timeout")

    def llm_response_cb(self, msg: String):
        if not self.waiting_llm or self.pending_action is None:
            return

        response = msg.data.strip()
        action_str = self.pending_action

        if not response:
            self.say("Je n'ai pas de réponse.")
            self.finish(False, action_str, "empty_llm_response")
            return

        self.say(response)
        self.finish(True, action_str)


if __name__ == "__main__":
    SpeakerNode()
    rospy.spin()
