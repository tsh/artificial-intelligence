import random
import pickle
from isolation import Isolation
from collections import defaultdict, Counter


NUMBER_OF_SIMULATION = 1000
MAX_DEPTH = 8

def generate_book():
    book = defaultdict(Counter)
    for i in range(NUMBER_OF_SIMULATION):
        initial_state = Isolation()
        build_tree(initial_state, MAX_DEPTH, book)
        print(i, len(book))
    return {k: max(v, key=v.get) for k, v in book.items()}


def build_tree(state, depth, book):
    if depth == 0 or state.terminal_test():
        return -simulate(state)
    action = random.choice(state.actions())
    new_state = state.result(action)
    reward = build_tree(new_state, depth-1, book)
    book[state][action] += reward
    return -reward

def simulate(state):
    while not state.terminal_test():
        action = random.choice(state.actions())
        state = state.result(action)
    return -1 if state.utility(state.player()) < 0 else 1


book = generate_book()

with open("data.pickle", 'wb') as f:
    pickle.dump(book, f)
