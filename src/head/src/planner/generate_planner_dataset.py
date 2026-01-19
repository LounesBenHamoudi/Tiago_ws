#!/usr/bin/env python3
import json
import random
import time
from pathlib import Path
import yaml

# =========================
# Paths
# =========================
DATA_DIR = Path(__file__).resolve().parents[2] / "data" / "planner"
DOMAIN_PATH = DATA_DIR / "world.yaml"
OUT_PATH = DATA_DIR / "planner_dataset.jsonl"

random.seed(42)

# =========================
# Load domain (world.yaml)
# =========================
def load_domain():
    with open(DOMAIN_PATH, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}

# =========================
# State format
# =========================
def build_state_text(goal_intent: str, goal_args: dict, history: list, last_status: str, retry_count: int):
    args_str = " ".join([f"{k}={v}" for k, v in goal_args.items()])
    hist_str = " ; ".join(history) if history else "EMPTY"
    return f"GOAL={goal_intent} {args_str} | LAST_STATUS={last_status} | RETRY={retry_count} | HISTORY={hist_str}"

# =========================
# Helpers: action type
# =========================
def action_kind(action: str) -> str:
    a = action.strip().upper()
    if a.startswith("GO_TO"):
        return "GO_TO"
    if a.startswith("READ_QR"):
        return "READ_QR"
    if a.startswith("SAY"):
        return "SAY"
    if a in ("STOP", "SKIP"):
        return a
    return "OTHER"

# =========================
# Optimal episode generator
# =========================
def episode_to_pairs_optimal(
    goal_intent,
    goal_args,
    plan_actions,
    p_fail_go_to=0.12,
    p_fail_qr=0.25,
    p_fail_say=0.02,
    max_skips_before_stop=2,
):
    """
    Génère (state -> next_action) en simulant une exécution réaliste.
    Règles :
      - FAILED => retry 1 fois
      - FAILED encore => SKIP
      - Trop de SKIP => STOP

    Les probabilités d'échec dépendent du type d'action.
    """
    pairs = []
    history = []
    last_status = "START"
    retry_count = 0

    skips = 0

    def should_fail(a: str) -> bool:
        k = action_kind(a)
        if k == "GO_TO":
            return random.random() < p_fail_go_to
        if k == "READ_QR":
            return random.random() < p_fail_qr
        if k == "SAY":
            return random.random() < p_fail_say
        return False

    for a in plan_actions:
        # Avant de proposer une nouvelle action, si on a trop de SKIP, on STOP.
        if skips > max_skips_before_stop:
            st = build_state_text(goal_intent, goal_args, history, last_status, retry_count)
            pairs.append({"x": st, "y": "STOP"})
            return pairs

        # 1) Proposer l'action
        st = build_state_text(goal_intent, goal_args, history, last_status, retry_count)
        pairs.append({"x": st, "y": a})

        # 2) Simuler résultat
        fail = should_fail(a)

        if not fail:
            history.append(a)
            last_status = "SUCCESS"
            retry_count = 0
            continue

        # FAILED 1ère fois => retry (RETRY=0 dans l'état de fail)
        last_status = "FAILED"
        retry_count = 0
        st = build_state_text(goal_intent, goal_args, history, last_status, retry_count)
        pairs.append({"x": st, "y": a})  # retry même action
        retry_count = 1

        # Simuler le retry : souvent ça marche, mais pas toujours.
        # On garde une proba d'échec plus basse au retry.
        # (tu peux ajuster ces facteurs)
        k = action_kind(a)
        if k == "GO_TO":
            fail_retry = random.random() < (p_fail_go_to * 0.60)
        elif k == "READ_QR":
            fail_retry = random.random() < (p_fail_qr * 0.80)
        elif k == "SAY":
            fail_retry = random.random() < (p_fail_say * 0.50)
        else:
            fail_retry = random.random() < 0.10

        if not fail_retry:
            history.append(a + " [RETRY_OK]")
            last_status = "SUCCESS"
            retry_count = 0
            continue

        # FAILED 2ème fois => SKIP
        last_status = "FAILED"
        st = build_state_text(goal_intent, goal_args, history, last_status, retry_count)
        pairs.append({"x": st, "y": "SKIP"})

        history.append(f"SKIPPED:{a}")
        skips += 1
        last_status = "SUCCESS"   # on continue après un skip
        retry_count = 0

    # Fin => STOP
    st = build_state_text(goal_intent, goal_args, history, last_status, retry_count)
    pairs.append({"x": st, "y": "STOP"})
    return pairs

# =========================
# Plan builders from world.yaml
# =========================
def make_guided_tour(domain, tour_name, shuffle=False, include_qr=True, include_say=False):
    places_map = domain.get("places_map", {}) or {}
    tours = domain.get("tours", {}) or {}
    tour = (tours.get(tour_name, []) or []).copy()

    if shuffle:
        random.shuffle(tour)

    actions = []
    for alias in tour:
        place_key = places_map.get(alias, alias)
        actions.append(f"GO_TO place={place_key}")
        if include_qr:
            actions.append("READ_QR")
        if include_say:
            # Phrase courte, sans espaces pour éviter les soucis si tu testes à la main
            actions.append("SAY text=Voici_la_fiche_du_robot")
    return actions

def make_go_to(domain, place_human):
    place_key = domain.get("places_map", {}).get(place_human, place_human)
    return [f"GO_TO place={place_key}"]

def make_come_back(domain):
    place_key = domain.get("places_map", {}).get("origine", "origine")
    return [f"GO_TO place={place_key}"]

# =========================
# Main
# =========================
def main(n_samples=3000):
    domain = load_domain()
    all_pairs = []

    # Paramètres "optimaux"
    P_FAIL_GO_TO = 0.12
    P_FAIL_QR = 0.25
    P_FAIL_SAY = 0.02
    MAX_SKIPS_BEFORE_STOP = 2

    for _ in range(n_samples):
        r = random.random()

        # 60% GUIDED_TOUR
        if r < 0.60:
            tour_name = "default"
            shuffle = random.random() < 0.30
            include_say = random.random() < 0.20  # 20% avec un SAY après QR (optionnel)
            actions = make_guided_tour(domain, tour_name, shuffle=shuffle, include_qr=True, include_say=include_say)

            goal_intent = "GUIDED_TOUR"
            goal_args = {"tour": tour_name, "shuffle": str(shuffle)}

            all_pairs.extend(
                episode_to_pairs_optimal(
                    goal_intent, goal_args, actions,
                    p_fail_go_to=P_FAIL_GO_TO,
                    p_fail_qr=P_FAIL_QR,
                    p_fail_say=P_FAIL_SAY,
                    max_skips_before_stop=MAX_SKIPS_BEFORE_STOP
                )
            )

        # 30% GO_TO simple
        elif r < 0.90:
            place_human = random.choice(list((domain.get("places_map", {}) or {}).keys()) or ["porte"])
            actions = make_go_to(domain, place_human)

            goal_intent = "GO_TO"
            goal_args = {"place": place_human}

            all_pairs.extend(
                episode_to_pairs_optimal(
                    goal_intent, goal_args, actions,
                    p_fail_go_to=P_FAIL_GO_TO,
                    p_fail_qr=P_FAIL_QR,
                    p_fail_say=P_FAIL_SAY,
                    max_skips_before_stop=MAX_SKIPS_BEFORE_STOP
                )
            )

        # 10% COME_BACK
        else:
            actions = make_come_back(domain)

            goal_intent = "COME_BACK"
            goal_args = {}

            all_pairs.extend(
                episode_to_pairs_optimal(
                    goal_intent, goal_args, actions,
                    p_fail_go_to=P_FAIL_GO_TO,
                    p_fail_qr=P_FAIL_QR,
                    p_fail_say=P_FAIL_SAY,
                    max_skips_before_stop=MAX_SKIPS_BEFORE_STOP
                )
            )

    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    with open(OUT_PATH, "w", encoding="utf-8") as f:
        for ex in all_pairs:
            f.write(json.dumps(ex, ensure_ascii=False) + "\n")

    print(f"✅ Dataset généré: {OUT_PATH}")
    print(f"   Nb exemples (state->action): {len(all_pairs)}")
    print("   Notes: GUIDED_TOUR inclut GO_TO + READ_QR (+ SAY parfois), avec retry/skip/stop.")

if __name__ == "__main__":
    main()
