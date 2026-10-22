#!/usr/bin/env python3
"""
game_logic.py — Dots and Boxes game state + Minimax AI (ROS node).

Board: 3x3 boxes (4x4 dots), 24 edges total.

Topics:
  Sub: /human_move  std_msgs/String  JSON {"r1":0,"c1":0,"r2":0,"c2":1}
  Pub: /robot_move  std_msgs/String  JSON {"r1":0,"c1":0,"r2":0,"c2":1}
  Pub: /game_state  std_msgs/String  JSON full board state

Params:
  ~max_depth  (int, default 10)  Minimax search depth
"""

import json
import rospy
from std_msgs.msg import String

# ── Board constants ──────────────────────────────────────────────────────────

ROWS, COLS = 3, 3   # 3x3 grid of boxes → 4x4 dots

def _make_all_edges():
    edges = []
    for r in range(ROWS + 1):          # horizontal edges
        for c in range(COLS):
            edges.append((r, c, r, c + 1))
    for r in range(ROWS):              # vertical edges
        for c in range(COLS + 1):
            edges.append((r, c, r + 1, c))
    return tuple(edges)

ALL_EDGES = _make_all_edges()          # 24 edges in fixed order

BOX_EDGES = {
    (r, c): [
        (r,     c,     r,     c + 1),  # top
        (r + 1, c,     r + 1, c + 1),  # bottom
        (r,     c,     r + 1, c    ),  # left
        (r,     c + 1, r + 1, c + 1),  # right
    ]
    for r in range(ROWS) for c in range(COLS)
}

# ── Board utilities ──────────────────────────────────────────────────────────

def count_new_boxes(edge, claimed_after):
    """Count boxes completed by this edge. claimed_after must already include edge."""
    r1, c1, r2, c2 = edge
    total = 0
    if r1 == r2:                           # horizontal edge
        candidates = [(r1 - 1, c1), (r1, c1)]
    else:                                  # vertical edge
        candidates = [(r1, c1 - 1), (r1, c1)]
    for br, bc in candidates:
        if 0 <= br < ROWS and 0 <= bc < COLS:
            if all(e in claimed_after for e in BOX_EDGES[(br, bc)]):
                total += 1
    return total


def _order_moves(available, claimed):
    """Put box-completing moves first to improve alpha-beta pruning."""
    completers, others = [], []
    for edge in available:
        if count_new_boxes(edge, claimed | {edge}) > 0:
            completers.append(edge)
        else:
            others.append(edge)
    return completers + others


def edge_from_dict(d):
    return (d["r1"], d["c1"], d["r2"], d["c2"])


def edge_to_dict(e):
    return {"r1": e[0], "c1": e[1], "r2": e[2], "c2": e[3]}

# ── Minimax with alpha-beta pruning ──────────────────────────────────────────

def _minimax(claimed, r_score, h_score, is_robot, alpha, beta, depth, max_depth):
    available = [e for e in ALL_EDGES if e not in claimed]
    if not available or depth >= max_depth:
        return r_score - h_score

    available = _order_moves(available, claimed)

    if is_robot:
        best = -float("inf")
        for edge in available:
            nc = claimed | {edge}
            nb = count_new_boxes(edge, nc)
            next_is_robot = bool(nb)           # robot goes again if it scored
            score = _minimax(nc, r_score + nb, h_score,
                             next_is_robot, alpha, beta, depth + 1, max_depth)
            best = max(best, score)
            alpha = max(alpha, best)
            if beta <= alpha:
                break
        return best
    else:
        best = float("inf")
        for edge in available:
            nc = claimed | {edge}
            nb = count_new_boxes(edge, nc)
            next_is_robot = not bool(nb)       # human goes again if it scored
            score = _minimax(nc, r_score, h_score + nb,
                             next_is_robot, alpha, beta, depth + 1, max_depth)
            best = min(best, score)
            beta = min(beta, best)
            if beta <= alpha:
                break
        return best


def best_move(claimed, r_score, h_score, max_depth=10):
    """Return the best edge for the robot to play."""
    available = _order_moves(
        [e for e in ALL_EDGES if e not in claimed], claimed
    )
    best_val  = -float("inf")
    best_edge = available[0]

    for edge in available:
        nc = claimed | {edge}
        nb = count_new_boxes(edge, nc)
        next_is_robot = bool(nb)
        val = _minimax(nc, r_score + nb, h_score, next_is_robot,
                       -float("inf"), float("inf"), 1, max_depth)
        if val > best_val:
            best_val  = val
            best_edge = edge

    return best_edge

# ── Game state ────────────────────────────────────────────────────────────────

class GameState:
    def __init__(self):
        self.claimed = set()
        self.r_edges = set()
        self.h_edges = set()
        self.r_score = 0
        self.h_score = 0

    def apply(self, edge, is_robot):
        """
        Apply a move. Returns number of boxes completed (-1 if invalid move).
        Caller uses return value: 0 = turn passes, >0 = same player goes again.
        """
        if edge in self.claimed:
            return -1
        self.claimed.add(edge)
        nb = count_new_boxes(edge, self.claimed)
        if is_robot:
            self.r_edges.add(edge)
            self.r_score += nb
        else:
            self.h_edges.add(edge)
            self.h_score += nb
        return nb

    def is_over(self):
        return len(self.claimed) == len(ALL_EDGES)

    def winner(self):
        if self.r_score > self.h_score:
            return "robot"
        if self.h_score > self.r_score:
            return "human"
        return "draw"

    def to_dict(self):
        return {
            "r_score":   self.r_score,
            "h_score":   self.h_score,
            "r_edges":   [list(e) for e in self.r_edges],
            "h_edges":   [list(e) for e in self.h_edges],
            "game_over": self.is_over(),
            "winner":    self.winner() if self.is_over() else None,
        }

# ── ROS node ─────────────────────────────────────────────────────────────────

class GameLogicNode:
    def __init__(self):
        rospy.init_node("game_logic_node")
        self.state      = GameState()
        self.robot_turn = False        # human always goes first
        self.max_depth  = rospy.get_param("~max_depth", 10)

        self.pub_move  = rospy.Publisher("/robot_move", String, queue_size=1)
        self.pub_state = rospy.Publisher("/game_state", String, queue_size=1)
        rospy.Subscriber("/human_move", String, self._on_human_move)

        rospy.loginfo("Game logic ready — human goes first.")
        rospy.loginfo("Board: %dx%d boxes, max_depth=%d", ROWS, COLS, self.max_depth)

    def _publish_state(self):
        self.pub_state.publish(json.dumps(self.state.to_dict()))

    def _do_robot_turn(self):
        edge = best_move(frozenset(self.state.claimed),
                         self.state.r_score, self.state.h_score,
                         self.max_depth)
        nb = self.state.apply(edge, is_robot=True)
        rospy.loginfo("Robot plays %s  (boxes scored: %d)", edge_to_dict(edge), nb)
        self.pub_move.publish(json.dumps(edge_to_dict(edge)))
        self._publish_state()

        if self.state.is_over():
            rospy.loginfo("Game over — winner: %s  (%d vs %d)",
                          self.state.winner(), self.state.r_score, self.state.h_score)
            return

        if nb > 0:
            self._do_robot_turn()      # robot completed a box, goes again
        else:
            self.robot_turn = False

    def _on_human_move(self, msg):
        if self.robot_turn:
            rospy.logwarn("Ignoring human move — it is the robot's turn.")
            return
        if self.state.is_over():
            rospy.logwarn("Game is already over.")
            return

        try:
            edge = edge_from_dict(json.loads(msg.data))
        except (KeyError, ValueError) as exc:
            rospy.logerr("Bad /human_move message: %s", exc)
            return

        nb = self.state.apply(edge, is_robot=False)
        if nb == -1:
            rospy.logwarn("Edge %s is already claimed.", edge)
            return

        rospy.loginfo("Human plays %s  (boxes scored: %d)", edge_to_dict(edge), nb)
        self._publish_state()

        if self.state.is_over():
            rospy.loginfo("Game over — winner: %s  (%d vs %d)",
                          self.state.winner(), self.state.r_score, self.state.h_score)
            return

        if nb > 0:
            rospy.loginfo("Human completed a box — play again!")
            return                     # stay in human turn

        self.robot_turn = True
        self._do_robot_turn()

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    GameLogicNode().spin()
