#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Bool
from gpt4all import GPT4All
import time

from pal_interaction_msgs.msg import TtsActionGoal


class TiagoChatNode:
    def __init__(self):
        rospy.init_node('tiago_chat_node')

        self.pub = rospy.Publisher('/tiago/output', String, queue_size=10)
        self.listen_pub = rospy.Publisher('/tiago/listen', Bool, queue_size=1)
        rospy.Subscriber('/tiago/voice_input', String, self.callback)

        # TTS via /tts/goal
        self.tts_pub = rospy.Publisher('/tts/goal', TtsActionGoal, queue_size=10)
        self.tts_seq = 0

        rospy.loginfo("Chargement du modèle GPT4All...")
        self.model = GPT4All("ggml-mpt-7b-chat.bin")

        # Prompt système
        self.prompt_intro = (
            "Tu dois parler en français. "
            "Tu es Tiago, un robot assistant intelligent et serviable. "
            "Tu réponds toujours de manière claire, polie et concise. "
            "Tu t'exprimes à la première personne. "
            "Réponse courte."
        )

        rospy.loginfo("🤖 Tiago est prêt à discuter (GPT4All sans chat_session).")

    def _tts_goal(self, text: str, lang_id: str = "fr_FR"):
        text = (text or "").strip()
        if not text:
            return

        self.tts_seq += 1
        msg = TtsActionGoal()
        msg.header.stamp = rospy.Time.now()
        msg.goal_id.stamp = rospy.Time.now()
        msg.goal_id.id = f"/tiago_chat_node-{self.tts_seq}-{int(time.time_ns())}"

        msg.goal.text.section = ""
        msg.goal.text.key = ""
        msg.goal.text.lang_id = ""
        msg.goal.text.arguments = []

        msg.goal.rawtext.text = text
        msg.goal.rawtext.lang_id = lang_id
        msg.goal.speakerName = ""
        msg.goal.wait_before_speaking = 0.0

        self.tts_pub.publish(msg)

    def callback(self, msg):
        user_input = msg.data.strip()
        rospy.loginfo("Vous : %s", user_input)

        try:
            self.listen_pub.publish(False)

            # ✅ prompt + question + réponse courte (20 tokens)
            full_prompt = f"{self.prompt_intro}\nUtilisateur: {user_input}\nTiago:"
            response = self.model.generate(full_prompt, max_tokens=20).strip()

            rospy.loginfo("🤖 Tiago : %s", response)

            self.pub.publish(response)
            self._tts_goal(response, "fr_FR")

            rospy.sleep(0.3)
            self.listen_pub.publish(True)

        except Exception as e:
            rospy.logerr("Erreur GPT : %s", str(e))
            self.pub.publish("Désolé, une erreur est survenue.")
            self._tts_goal("Désolé, une erreur est survenue.", "fr_FR")
            self.listen_pub.publish(True)


if __name__ == '__main__':
    try:
        node = TiagoChatNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
