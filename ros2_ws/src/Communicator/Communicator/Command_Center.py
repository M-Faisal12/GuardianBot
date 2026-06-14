"""
command_center.py
==================
ROS2 node: CommandCenterNode
 
Subscribes to /robot_command and forwards every command to the MQTT topic
/esp32 so that an ESP32 microcontroller can read and execute it.
 
Design goals
------------
  • Reliable delivery  — QoS 1 (at-least-once) on MQTT
  • Deduplication      — identical consecutive commands within a short window
                         are coalesced (configurable) to avoid ESP32 flooding
  • Structured payload — JSON envelope with sequence counter, timestamp, and
                         the raw command string
  • Secure transport   — optional TLS + username/password (same flags as
                         alert_center.py)
  • Non-blocking       — MQTT loop runs in its own thread; ROS callbacks never
                         block waiting for the broker
 
MQTT topic published
--------------------
  /esp32  – one message per robot command
 
Payload schema (JSON)
---------------------
  {
    "seq":       <int>,        // monotonically increasing sequence number
    "cmd":       "<string>",   // the raw command, e.g. "RIGHT", "LEFT"
    "timestamp": <float>       // Unix epoch seconds (float)
  }
 
Dependencies
------------
  pip install paho-mqtt
  ROS2 (rclpy, std_msgs)
"""
 
import json
import threading
import time
 
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from std_msgs.msg import String
 
import paho.mqtt.client as mqtt
 
# ---------------------------------------------------------------------------
# MQTT CONFIG  —  edit to match your broker
# ---------------------------------------------------------------------------
MQTT_BROKER_HOST = "localhost"       # e.g. "192.168.1.100" or cloud endpoint
MQTT_BROKER_PORT = 1883
MQTT_KEEPALIVE   = 60
MQTT_USE_TLS     = False             # set True + fill TLS_* below for TLS
MQTT_USERNAME    = ""                # leave empty if broker has no auth
MQTT_PASSWORD    = ""
 
# TLS options (only used when MQTT_USE_TLS = True)
TLS_CA_CERTS = "/path/to/ca.crt"
TLS_CERTFILE = "/path/to/client.crt"
TLS_KEYFILE  = "/path/to/client.key"
 
# MQTT QoS for command messages
#   0 = at most once  |  1 = at least once  |  2 = exactly once
MQTT_QOS    = 1
MQTT_RETAIN = False   # commands are transient — do NOT retain
 
# MQTT client ID — must be unique per broker connection
MQTT_CLIENT_ID = "ros2_command_center"
 
# MQTT topic the ESP32 subscribes to
ESP32_TOPIC = "/esp32"
 
# ---------------------------------------------------------------------------
# DEDUPLICATION CONFIG
# ---------------------------------------------------------------------------
# If the same command arrives multiple times within this window (seconds),
# only the first occurrence is forwarded.  Set to 0.0 to disable.
DEDUP_WINDOW_SEC = 0.1
 
# ---------------------------------------------------------------------------
# ROS CONFIG
# ---------------------------------------------------------------------------
ROBOT_CMD_TOPIC = "/robot_command"
 
# ---------------------------------------------------------------------------
 
 
class CommandCenterNode(Node):
    """
    Bridges /robot_command → MQTT /esp32.
    """
 
    def __init__(self):
        super().__init__("command_center_node")
 
        # ── Sequence counter (for ESP32-side ordering / dedup) ───────────────
        self._seq      = 0
        self._seq_lock = threading.Lock()
 
        # ── Deduplication state ──────────────────────────────────────────────
        self._last_cmd      = None
        self._last_cmd_time = 0.0
        self._dedup_lock    = threading.Lock()
 
        # ── MQTT setup ───────────────────────────────────────────────────────
        self._mqtt = mqtt.Client(client_id=MQTT_CLIENT_ID, clean_session=True)
 
        if MQTT_USERNAME:
            self._mqtt.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
 
        if MQTT_USE_TLS:
            self._mqtt.tls_set(
                ca_certs=TLS_CA_CERTS,
                certfile=TLS_CERTFILE,
                keyfile=TLS_KEYFILE,
            )
            self._mqtt.tls_insecure_set(False)
 
        self._mqtt.on_connect    = self._on_mqtt_connect
        self._mqtt.on_disconnect = self._on_mqtt_disconnect
        self._mqtt.on_publish    = self._on_mqtt_publish
 
        self._mqtt_connected = False
        self._mqtt_lock      = threading.Lock()
 
        self._connect_mqtt()         # non-blocking; retries in background
 
        # ── ROS subscription ─────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=50,               # generous queue — never miss a command
        )
        self._cmd_sub = self.create_subscription(
            String,
            ROBOT_CMD_TOPIC,
            self._cmd_callback,
            qos,
        )
 
        self.get_logger().info(
            f"CommandCenterNode ready — "
            f"listening on '{ROBOT_CMD_TOPIC}', "
            f"forwarding to MQTT '{ESP32_TOPIC}'"
        )
 
    # =========================================================================
    # MQTT HELPERS
    # =========================================================================
 
    def _connect_mqtt(self):
        """
        Try to connect to the broker in a background thread.
        Retries every 5 seconds on failure so the node stays alive.
        """
        def _try_connect():
            while not self._mqtt_connected:
                try:
                    self._mqtt.connect(
                        MQTT_BROKER_HOST,
                        MQTT_BROKER_PORT,
                        MQTT_KEEPALIVE,
                    )
                    self._mqtt.loop_start()
                    return
                except Exception as exc:
                    self.get_logger().warn(
                        f"[MQTT] Connection failed ({exc}). Retrying in 5 s…"
                    )
                    time.sleep(5)
 
        threading.Thread(target=_try_connect, daemon=True).start()
 
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self._mqtt_connected = True
            self.get_logger().info(
                f"[MQTT] Connected to broker "
                f"{MQTT_BROKER_HOST}:{MQTT_BROKER_PORT}"
            )
        else:
            self.get_logger().error(
                f"[MQTT] Connection refused — rc={rc}"
            )
 
    def _on_mqtt_disconnect(self, client, userdata, rc):
        self._mqtt_connected = False
        if rc != 0:
            self.get_logger().warn(
                f"[MQTT] Unexpected disconnection (rc={rc}). "
                "Paho will attempt automatic reconnect."
            )
 
    def _on_mqtt_publish(self, client, userdata, mid):
        self.get_logger().debug(f"[MQTT] Delivery confirmed (mid={mid})")
 
    def _next_seq(self) -> int:
        with self._seq_lock:
            self._seq += 1
            return self._seq
 
    # =========================================================================
    # DEDUPLICATION
    # =========================================================================
 
    def _is_duplicate(self, cmd: str) -> bool:
        """
        Return True if `cmd` is identical to the last forwarded command and
        was received within DEDUP_WINDOW_SEC.  Thread-safe.
        """
        if DEDUP_WINDOW_SEC <= 0.0:
            return False
 
        now = time.monotonic()
        with self._dedup_lock:
            if (
                cmd == self._last_cmd
                and (now - self._last_cmd_time) < DEDUP_WINDOW_SEC
            ):
                return True
            self._last_cmd      = cmd
            self._last_cmd_time = now
            return False
 
    # =========================================================================
    # ROS CALLBACK
    # =========================================================================
 
    def _cmd_callback(self, msg: String):
        cmd = msg.data.strip()
 
        if not cmd:
            self.get_logger().warn("[CMD] Empty command received — ignoring.")
            return
 
        # ── Deduplication ─────────────────────────────────────────────────────
        if self._is_duplicate(cmd):
            self.get_logger().debug(
                f"[CMD] Duplicate '{cmd}' within dedup window — skipped."
            )
            return
 
        # ── Build structured payload ──────────────────────────────────────────
        seq     = self._next_seq()
        payload = {
            "seq":       seq,
            "cmd":       cmd,
            "timestamp": time.time(),
        }
        json_payload = json.dumps(payload, ensure_ascii=False)
 
        # ── Publish to MQTT ───────────────────────────────────────────────────
        if not self._mqtt_connected:
            self.get_logger().warn(
                f"[MQTT] Broker not connected — command '{cmd}' (seq={seq}) "
                "dropped! Check broker and restart node."
            )
            return
 
        with self._mqtt_lock:
            result = self._mqtt.publish(
                ESP32_TOPIC,
                payload=json_payload,
                qos=MQTT_QOS,
                retain=MQTT_RETAIN,
            )
 
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            self.get_logger().error(
                f"[MQTT] Publish failed for cmd='{cmd}' seq={seq} "
                f"rc={result.rc}"
            )
        else:
            self.get_logger().info(
                f"[CMD→MQTT] seq={seq:04d}  cmd='{cmd}'  "
                f"→ topic='{ESP32_TOPIC}'"
            )
 
    # =========================================================================
 
    def destroy_node(self):
        self._mqtt.loop_stop()
        self._mqtt.disconnect()
        super().destroy_node()
 
 
# ---------------------------------------------------------------------------
# ENTRY POINT
# ---------------------------------------------------------------------------
 
def main(args=None):
    rclpy.init(args=args)
    node = CommandCenterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
 