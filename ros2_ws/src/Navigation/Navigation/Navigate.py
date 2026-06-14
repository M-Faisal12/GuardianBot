
import copy
import random
 
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
 
# Import A* functions from the same package
from Navigation.Astar import astar, generate_commands, WALL, FREE
 
# ---------------------------------------------------------------------------
# CONFIG
# ---------------------------------------------------------------------------
 
# Grid definition (0 = free, 1 = wall) — edit to match your environment
INITIAL_GRID = [
    [0, 0, 0, 0, 0],
    [0, 1, 1, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 1, 0, 1, 0],
    [0, 0, 0, 0, 0],
]
 
# Named patrol stops for the mall — add as many as needed
# These are the only locations the robot will patrol between
STOPS = {
    "A": (0, 4),
    "B": (2, 0),
    "C": (2, 2),
}
 
# Starting stop
START_STOP = "A"
 
# ROS topics
HURDLE_CMD_TOPIC  = "/move"           # CV hurdle node publishes here
COMMUNICATE_TOPIC = "/robot_command"  # final commands go here
ALERT_TOPIC       = "/danger"          # danger/danger node publishes here
CURRENT_POS_TOPIC = "/Current_Pos"    # this node publishes position here
 
# How often (seconds) to publish current position during normal patrol
POS_PUBLISH_RATE = 1.0
 
# ---------------------------------------------------------------------------
 
HURDLE_SIGNAL = {"Move left", "Move right"}
CLEAR_SIGNAL  = {"Move forward"}
 
CMD_DELTA = {
    "UP":            (-1,  0),
    "DOWN":          ( 1,  0),
    "LEFT":          ( 0, -1),
    "RIGHT":         ( 0,  1),
    "UP_LEFT":       (-1, -1),
    "UP_RIGHT":      (-1,  1),
    "DOWN_LEFT":     ( 1, -1),
    "DOWN_RIGHT":    ( 1,  1),
    "Move forward":  (-1,  0),
    "Move left":     ( 0, -1),
    "Move right":    ( 0,  1),
    "Move backward": ( 1,  0),
}
 
 
class AStarNavNode(Node):
 
    def __init__(self):
        super().__init__('astar_nav_node')
 
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
 
        # ── Subscribers ──────────────────────────────────────────────────────
 
        # CV hurdle commands
        self.hurdle_sub = self.create_subscription(
            String,
            HURDLE_CMD_TOPIC,
            self._hurdle_callback,
            qos,
        )
 
        # Alert topic — highest priority, pauses normal patrol logic
        self.alert_sub = self.create_subscription(
            String,
            ALERT_TOPIC,
            self._alert_callback,
            10,                         # reliable QoS — alerts must not be dropped
        )
 
        # ── Publishers ───────────────────────────────────────────────────────
 
        self.cmd_pub = self.create_publisher(String, COMMUNICATE_TOPIC, 10)
        self.pos_pub = self.create_publisher(String, CURRENT_POS_TOPIC, 10)
 
        # ── Internal state ───────────────────────────────────────────────────
 
        self._grid              = copy.deepcopy(INITIAL_GRID)
        self._position          = STOPS[START_STOP]
        self._goal              = self._pick_next_goal(exclude=STOPS[START_STOP])
        self._astar_cmds        = []
        self._current_astar_cmd = None
 
        # Alert state tracking
        self._last_alert_data   = None
 
        # Periodic position publisher timer
        self.pos_timer = self.create_timer(POS_PUBLISH_RATE, self._publish_position)
 
        # Plan the first patrol path
        self._replan()
 
        self.get_logger().info(
            f"AStarNavNode started | pos={self._position} | "
            f"first goal={self._goal} | patrolling {list(STOPS.keys())}"
        )
 
    # =========================================================================
    # PATROL — AUTONOMOUS GOAL SELECTION
    # =========================================================================
 
    def _pick_next_goal(self, exclude=None):
        """
        Randomly pick a new patrol stop, never the same as `exclude`
        (which is the current position / just-reached goal).
        Falls back to any stop if only one exists.
        """
        candidates = [pos for label, pos in STOPS.items() if pos != exclude]
        if not candidates:
            candidates = list(STOPS.values())
        chosen = random.choice(candidates)
        label  = [k for k, v in STOPS.items() if v == chosen][0]
        self.get_logger().info(f"[PATROL] New goal assigned → Stop {label} {chosen}")
        return chosen
 
    def _on_goal_reached(self):
        """
        Called whenever the robot arrives at the current goal.
        Assigns a new random patrol goal and replans immediately.
        """
        self.get_logger().info(
            f"[PATROL] *** Reached goal {self._goal} — selecting next patrol stop ***"
        )
        self._goal = self._pick_next_goal(exclude=self._position)
        self._replan()
 
    # =========================================================================
    # ALERT CALLBACK — highest priority
    # =========================================================================
 
    def _alert_callback(self, msg: String):
        """
        Triggered whenever /alert receives data (fight/gun detected).
        - Immediately publishes current robot position on /Current_Pos.
        - Robot continues patrolling toward goal uninterrupted.
        - Position is also published periodically by the timer, but this
          gives an instant publish the moment the alert fires.
        """
        alert_data            = msg.data.strip()
        self._last_alert_data = alert_data
 
        self.get_logger().warn(
            f"[ALERT] *** PRIORITY ALERT RECEIVED: {alert_data} ***"
        )
        self.get_logger().warn(
            f"[ALERT] Position reported — robot continues patrol toward {self._goal}"
        )
 
        # Publish position instantly — movement is never paused
        self._publish_position(force=True, reason=f"ALERT: {alert_data}")
 
    # =========================================================================
    # PLANNING
    # =========================================================================
 
    def _replan(self):
        """Run A* from current position to goal and load the command queue."""
        path = astar(self._grid, self._position, self._goal)
 
        if not path:
            self.get_logger().warn(
                f"A* found no path from {self._position} to {self._goal}!"
            )
            self._astar_cmds        = []
            self._current_astar_cmd = None
            return
 
        self._astar_cmds = generate_commands(path)
        self._advance_astar_step()
 
        self.get_logger().info(
            f"[A*] Replanned: {len(self._astar_cmds) + 1} steps remaining "
            f"→ goal {self._goal}"
        )
 
    def _advance_astar_step(self):
        """Pop the next command from the A* queue and make it the active step."""
        if self._astar_cmds:
            self._current_astar_cmd = self._astar_cmds.pop(0)
            self.get_logger().info(
                f"[A*] Next step → {self._current_astar_cmd['command']} "
                f"angle={self._current_astar_cmd['angle']}°"
            )
        else:
            self._current_astar_cmd = None
            self.get_logger().info("[A*] No more steps in queue.")
 
    # =========================================================================
    # HURDLE (CV) CALLBACK — one message = validate one command
    # =========================================================================
 
    def _hurdle_callback(self, msg: String):
 
        cv_cmd = msg.data.strip()
 
        # ── No A* plan available ─────────────────────────────────────────────
        if self._current_astar_cmd is None:
            self.get_logger().warn("No active A* command. Trying to replan...")
            self._replan()
            if self._current_astar_cmd is None:
                return
 
        astar_name = self._current_astar_cmd["command"]
 
        if cv_cmd in CLEAR_SIGNAL:
            # ── NO HURDLE: path clear → execute A* step ───────────────────────
            self.get_logger().info(
                f"[CLEAR]  CV={cv_cmd}  A*={astar_name}  → executing A* step"
            )
            self._publish(astar_name)
            self._apply_move(astar_name)
            self._advance_astar_step()
 
        elif cv_cmd in HURDLE_SIGNAL:
            # ── HURDLE: execute CV command, mark wall, replan ─────────────────
            self.get_logger().warn(
                f"[HURDLE] CV={cv_cmd} → obstacle ahead of A* step "
                f"'{astar_name}' → executing CV command and replanning"
            )
            self._publish(cv_cmd)
            self._mark_obstacle_ahead(astar_name)
            self._apply_move(cv_cmd)
            self._replan()
 
        else:
            self.get_logger().warn(
                f"[UNKNOWN] CV sent unrecognised command '{cv_cmd}' — ignoring"
            )
 
    # =========================================================================
    # GRID HELPERS
    # =========================================================================
 
    def _mark_obstacle_ahead(self, astar_cmd_name: str):
        """Mark the grid cell the A* step would have entered as WALL."""
        dr, dc = CMD_DELTA.get(astar_cmd_name, (0, 0))
        r, c   = self._position
        nr, nc = r + dr, c + dc
 
        rows = len(self._grid)
        cols = len(self._grid[0]) if rows else 0
 
        if 0 <= nr < rows and 0 <= nc < cols:
            if self._grid[nr][nc] != WALL:
                self._grid[nr][nc] = WALL
                self.get_logger().info(
                    f"[GRID] Marked ({nr},{nc}) as WALL (obstacle detected)"
                )
 
    def _apply_move(self, cmd_name: str):
        """Advance self._position by one step in the given direction."""
        dr, dc = CMD_DELTA.get(cmd_name, (0, 0))
        r, c   = self._position
        nr, nc = r + dr, c + dc
 
        rows = len(self._grid)
        cols = len(self._grid[0]) if rows else 0
 
        if 0 <= nr < rows and 0 <= nc < cols and self._grid[nr][nc] != WALL:
            self._position = (nr, nc)
            self.get_logger().info(f"[POS] Updated position → {self._position}")
        else:
            self.get_logger().warn(
                f"[POS] Move '{cmd_name}' would go out-of-bounds or into wall; "
                f"staying at {self._position}"
            )
 
        # ── Check if goal reached → assign next patrol stop ──────────────────
        if self._position == self._goal:
            self._on_goal_reached()
 
    # =========================================================================
    # PUBLISHERS
    # =========================================================================
 
    def _publish(self, command: str):
        msg      = String()
        msg.data = command
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"[PUB] → '{command}' on {COMMUNICATE_TOPIC}")
 
    def _publish_position(self, force: bool = False, reason: str = ""):
        """
        Publish current position on /Current_Pos.
        Called by timer every POS_PUBLISH_RATE seconds,
        and immediately (force=True) when an alert fires.
        """
        label = next(
            (k for k, v in STOPS.items() if v == self._position), "UNKNOWN"
        )
        pos_str = f"row={self._position[0]},col={self._position[1]},stop={label}"
 
        if force and reason:
            pos_str += f" | PRIORITY: {reason}"
 
        msg      = String()
        msg.data = pos_str
        self.pos_pub.publish(msg)
 
        if force:
            self.get_logger().warn(f"[POS ALERT] {pos_str}")
        else:
            self.get_logger().info(f"[POS] Published → {pos_str}")
 
    # =========================================================================
 
    def destroy_node(self):
        super().destroy_node()
 
 
# ---------------------------------------------------------------------------
# ENTRY POINT
# ---------------------------------------------------------------------------
 
def main(args=None):
    rclpy.init(args=args)
    node = AStarNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()