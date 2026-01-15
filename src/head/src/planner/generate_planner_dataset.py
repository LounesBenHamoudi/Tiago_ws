#!/usr/bin/env python3
import json
import random
from pathlib import Path
import yaml

DATA_DIR = Path(__file__).resolve().parents[2] / "data" / "planner"
DOMAIN_PATH = DATA_DIR / "world.yaml"
OUT_PATH = DATA_DIR / "planner_dataset.jsonl"

random.seed(42)

def load_domain():
    with open(DOMAIN_PATH, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)

def build_state_text(goal_intent: str, goal_args: dict, history: list, last_status: str, retry_count: int):
    args_str = " ".join([f"{k}={v}" for k, v in goal_args.items()])
    hist_str = " ; ".join(history) if history else "EMPTY"
    return f"GOAL={goal_intent} {args_str} | LAST_STATUS={last_status} | RETRY={retry_count} | HISTORY={hist_str}"


def episode_to_pairs_with_fail_policy(goal_intent, goal_args, plan_actions, p_fail=0.15):
    """
    Génère (state -> next_action) en simulant des échecs:
    - sur FAILED: retry 1 fois (RETRY=0 -> proposer la même action)
    - si re-FAILED: SKIP (ajouter SKIPPED dans history)
    - STOP à la fin
    """
    pairs = []
    history = []
    last_status = "START"
    retry_count = 0

    for a in plan_actions:
        # Etat normal -> propose action
        state = build_state_text(goal_intent, goal_args, history, last_status, retry_count)
        pairs.append({"x": state, "y": a})

        # On simule si l’action réussit ou échoue
        fail = (random.random() < p_fail)

        if not fail:
            # SUCCESS
            history.append(a)
            last_status = "SUCCESS"
            retry_count = 0
            continue

        # FAILED 1ère fois -> retry
        last_status = "FAILED"
        retry_count = 0
        state = build_state_text(goal_intent, goal_args, history, last_status, retry_count)
        pairs.append({"x": state, "y": a})  # retry la même action
        retry_count = 1

        # Simuler le résultat du retry
        fail_retry = (random.random() < 0.5)  # 50% des retries échouent (ajuste si tu veux)

        if not fail_retry:
            # SUCCESS après retry
            history.append(a + " [RETRY_OK]")
            last_status = "SUCCESS"
            retry_count = 0
            continue

        # FAILED 2ème fois -> SKIP (on passe à l’action suivante)
        last_status = "FAILED"
        state = build_state_text(goal_intent, goal_args, history, last_status, retry_count)
        pairs.append({"x": state, "y": "SKIP"})  # le planner apprend SKIP ici

        history.append(f"SKIPPED:{a}")
        last_status = "SUCCESS"   # après SKIP, on continue
        retry_count = 0

    # Fin -> STOP
    state = build_state_text(goal_intent, goal_args, history, last_status, retry_count)
    pairs.append({"x": state, "y": "STOP"})
    return pairs


def make_guided_tour(domain, tour_name, shuffle=False):
    places_map = domain["places_map"]
    tour = domain["tours"][tour_name].copy()
    if shuffle:
        random.shuffle(tour)

    # plan = GO_TO pour chaque lieu
    actions = []
    for p in tour:
        place_key = places_map.get(p, p)
        actions.append(f"GO_TO place={place_key}")
    return actions

def make_go_to(domain, place_human):
    place_key = domain["places_map"].get(place_human, place_human)
    return [f"GO_TO place={place_key}"]

def main(n_samples=3000):
    domain = load_domain()
    all_pairs = []

    # On génère plusieurs types d'objectifs (goals)
    for _ in range(n_samples):
        r = random.random()

        # 60% tours
        if r < 0.60:
            tour_name = "default"
            shuffle = random.random() < 0.30  # 30% tours avec ordre mélangé
            actions = make_guided_tour(domain, tour_name, shuffle=shuffle)

            goal_intent = "GUIDED_TOUR"
            goal_args = {"tour": tour_name, "shuffle": str(shuffle)}

            all_pairs.extend(episode_to_pairs_with_fail_policy(goal_intent, goal_args, actions))


        # 30% GO_TO simple (un lieu)
        elif r < 0.90:
            place_human = random.choice(list(domain["places_map"].keys()))
            actions = make_go_to(domain, place_human)

            goal_intent = "GO_TO"
            goal_args = {"place": place_human}

            all_pairs.extend(episode_to_pairs_with_fail_policy(goal_intent, goal_args, actions))


        # 10% COME_BACK (retour origine)
        else:
            actions = [f"GO_TO place={domain['places_map'].get('origine', 'origine')}"]
            goal_intent = "COME_BACK"
            goal_args = {}

            all_pairs.extend(episode_to_pairs_with_fail_policy(goal_intent, goal_args, actions))


    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    with open(OUT_PATH, "w", encoding="utf-8") as f:
        for ex in all_pairs:
            f.write(json.dumps(ex, ensure_ascii=False) + "\n")

    print(f"✅ Dataset généré: {OUT_PATH}")
    print(f"   Nb exemples (state->action): {len(all_pairs)}")

if __name__ == "__main__":
    main()
