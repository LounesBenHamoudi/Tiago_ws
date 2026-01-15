#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from gpt4all import GPT4All

class TiagoChatNode:
    def __init__(self):
        rospy.init_node('tiago_chat_node')

        self.pub = rospy.Publisher('/tiago/output', String, queue_size=10)
        rospy.Subscriber('/tiago/input', String, self.callback)

        rospy.loginfo("Chargement du modèle GPT4All...")
        self.model = GPT4All("ggml-model-gpt4all-falcon-q4_0.bin")
        #self.model = GPT4All("ggml-mpt-7b-chat.bin")


        # Prompt système court + très contraignant
        self.system_prompt = (
            # "Tu parles uniquement en français. "
            # "Tu es TIAGo, un robot assistant. "
            # "Règles: "
            # "1) Si l'utilisateur dit bonjour/salut/hello, tu réponds toujours par 'Bonjour !' ou 'Salut !' puis une question courte. "
            # "2) Ne répète pas 'Je suis TIAGo' sauf si l'utilisateur te demande qui tu es. "
            # "3) Réponds en 1 à 2 phrases maximum. "
            # "4) Pas de balises, pas de '## Instruction'. "
        )


        self.history = []          # garde les derniers échanges
        self.max_turns = 4         # 2 tours utilisateur/assistant


        # Démarre une session persistante
        self.chat = self.model.chat_session()
        self.chat.__enter__()
        _ = self.model.generate(self.system_prompt, max_tokens=20, temp=0.2)

        rospy.loginfo("Tiago est prêt à discuter !")

    def callback(self, msg):
        user_input = msg.data.strip()
        rospy.loginfo("Message reçu : %s", user_input)

        # met à jour l'historique (format simple)
        self.history.append(("Utilisateur", user_input))
        self.history = self.history[-self.max_turns:]

        # construit le contexte
        history_txt = "\n".join([f"{role}: {text}" for role, text in self.history])

        prompt = (
            f"{self.system_prompt}\n\n"
            f"{history_txt}\n"
            f"Assistant:"
        )

        try:
            response = self.model.generate(
                prompt,
                max_tokens=30,
                temp=0.3,
                top_p=0.9,
                repeat_penalty=1.2
            ).strip()

            # nettoyage rapide
            response = response.replace("## Instruction:", "").replace("##", "").strip()

            # ajoute la réponse à l'historique
            self.history.append(("Assistant", response))
            self.history = self.history[-self.max_turns:]

            rospy.loginfo("Réponse générée : %s", response)
            self.pub.publish(response)

        except Exception as e:
            rospy.logerr("Erreur lors de la génération : %s", str(e))
            self.pub.publish("Désolé, une erreur est survenue.")

    def __del__(self):
        if hasattr(self, "chat"):
            self.chat.__exit__(None, None, None)

if __name__ == '__main__':
    try:
        node = TiagoChatNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
