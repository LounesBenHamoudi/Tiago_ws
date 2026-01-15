#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import os
import time
from dotenv import load_dotenv
from openai import OpenAI

# Chargement de la clé API
load_dotenv()
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

# Anti-boucle
last_processed_time = 0
last_message = ""

# Contexte externe reçu du node tampon
external_context = ""

# Prompt système enrichi avec instructions pour utiliser le contexte
system_prompt = """

Tu es Tiago, un robot humanoïde réel conçu par PAL Robotics. Tu es actuellement en service sur le campus CESI de Nanterre, en France.

Tu as été programmé et intégré à un environnement ROS (Robot Operating System) par un groupe d’étudiants de la filière S3E. Grâce à cette architecture, tu es capable de bouger, écouter, parler, et interagir via des nœuds ROS.

Tu es équipé de bras articulés, d’une base mobile, de capteurs audio et vidéo, et tu peux détecter les visages, les émotions et le nombre de doigts levés. Tu peux également exécuter des gestes en réponse à certaines phrases.

**Toutes les données externes reçues entre crochets, comme [Visage détecté: oui | Émotion: heureux | Doigts levés: 2], sont des observations en temps réel fournies par tes capteurs.** Tu dois les intégrer dans tes réponses, de manière naturelle, sans les répéter. Utilise ces données pour mieux comprendre la situation et répondre comme un assistant humain.

Tu dois garder un ton amical et professionnel. Réponds de manière courte, claire, naturelle, comme si tu étais un vrai robot assistant présent dans la pièce.

"""

def ask_gpt(prompt):
    try:
        response = client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": prompt}
            ]
        )
        return response.choices[0].message.content
    except Exception as e:
        rospy.logerr(f"Erreur GPT : {e}")
        return "Désolé, je n'ai pas compris."

def voice_callback(msg):
    global last_processed_time, last_message, external_context

    user_input = msg.data.strip().lower()
    now = time.time()

    if now - last_processed_time < 2.0 or user_input == last_message:
        return

    last_processed_time = now
    last_message = user_input

    mic_control_pub.publish("pause")
    rospy.sleep(1.0)

    full_prompt = f"{external_context}\n{user_input}"
    rospy.loginfo(f"🗣️ Question reçue : {user_input}")
    reply = ask_gpt(full_prompt)
    rospy.loginfo(f" Réponse GPT : {reply}")

    tts_pub.publish(reply)
    intent_pub.publish(reply)

def context_callback(msg):
    global external_context
    external_context = f"[{msg.data}]"

if __name__ == '__main__':
    rospy.init_node('gpt_node')
    tts_pub = rospy.Publisher('/text_to_speech', String, queue_size=10)
    mic_control_pub = rospy.Publisher('/voice_control', String, queue_size=10)
    intent_pub = rospy.Publisher('/tiago/intent', String, queue_size=10)

    rospy.Subscriber('/tiago/voice_input', String, voice_callback)
    rospy.Subscriber('/tiago/context_info', String, context_callback)

    rospy.loginfo("Node GPT-3.5 avec contexte externe actif.")
    rospy.spin()