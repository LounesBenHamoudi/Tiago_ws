#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from gpt4all import GPT4All

class LLMResponderNode:
    def __init__(self):
        rospy.init_node('llm_responder')

        self.pub = rospy.Publisher('/tiago/speaker/llm_response', String, queue_size=10)
        rospy.Subscriber('/tiago/speaker/llm_request', String, self.callback, queue_size=10)

        rospy.loginfo("[LLM] Chargement du modèle GPT4All...")
        self.model = GPT4All("ggml-mpt-7b-chat.bin")

        self.prompt_intro = (
            "Tu dois parler en français. "
            "Tu es Tiago, un robot assistant intelligent et serviable, utilisé dans des environnements humains. "
            "Tu réponds toujours de manière claire, polie et concise. "
            "Tu t'exprimes à la première personne comme un robot assistant réel. "
            "L’année actuelle est 2025. "
            "Si on te demande une action simple, réponds brièvement et confirme que tu t’y mets."
        )

        # Session chat (comme ton code)
        self.session = self.model.chat_session()
        self.session.__enter__()
        self.model.generate(self.prompt_intro, max_tokens=80)

        rospy.loginfo("[LLM] Ready. Waiting /tiago/speaker/llm_request")

    def callback(self, msg: String):
        user_input = msg.data.strip()
        if not user_input:
            self.pub.publish(String(data=""))
            return

        try:
            response = self.model.generate(user_input, max_tokens=60).strip()
            self.pub.publish(String(data=response))
            rospy.loginfo("[LLM] Response: %s", response)
        except Exception as e:
            rospy.logerr("[LLM] Error: %s", str(e))
            self.pub.publish(String(data="Désolé, j'ai eu un problème pour répondre."))

    def __del__(self):
        if hasattr(self, "session"):
            self.session.__exit__(None, None, None)

if __name__ == '__main__':
    try:
        node = LLMResponderNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
