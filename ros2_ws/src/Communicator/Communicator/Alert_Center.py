"""
alert_center.py
================
ROS2 node: AlertCenterNode
 
Subscribes to /Current_Pos and inspects every message for embedded alert
keywords (FIRE, WEAPON / GUN, FIGHT).  Normal position messages are silently
ignored.  When an alert is detected the payload is forwarded to the matching
MQTT topic so that any downstream subscriber (dashboard, security team, etc.)
is notified immediately.
 
MQTT topics published
---------------------
  /fire    – fire / smoke detected
  /weapon  – firearm / weapon detected
  /fight   – physical altercation detected
 
Dependencies
------------
  pip install paho-mqtt
  ROS2 (rclpy, std_msgs)
"""
 
import json
import re
import time
import threading
 
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
TLS_CA_CERTS   = "/path/to/ca.crt"
TLS_CERTFILE   = "/path/to/client.crt"
TLS_KEYFILE    = "/path/to/client.key"
 
# MQTT QoS for alert messages
#   0 = at most once  |  1 = at least once  |  2 = exactly once
MQTT_QOS    = 1
MQTT_RETAIN = True   # broker retains last alert so new subscribers see it
 
# MQTT client ID — must be unique per broker connection
MQTT_CLIENT_ID = "ros2_alert_center"
 
# ---------------------------------------------------------------------------
# ROS CONFIG
# ---------------------------------------------------------------------------
CURRENT_POS_TOPIC = "/Current_Pos"
 
# ---------------------------------------------------------------------------
# ALERT KEYWORD MAPS
#   Each entry:  keyword_regex  →  mqtt_topic
#
#   Keywords are matched case-insensitively against the raw message string.
# ---------------------------------------------------------------------------
ALERT_RULES = [
    (re.compile(r"\bFIRE\b",             re.IGNORECASE), "/fire"),
    (re.compile(r"\b(WEAPON|GUN|KNIFE)\b", re.IGNORECASE), "/weapon"),
    (re.compile(r"\bFIGHT\b",            re.IGNORECASE), "/fight"),
]
 
# "ALERT" or "PRIORITY" marker that the main nav node injects — used to
# quickly distinguish alert messages from ordinary position messages.
ALERT_MARKER = re.compile(r"\bPRIORITY\b", re.IGNORECASE)
 
# ---------------------------------------------------------------------------
 
 
class AlertCenterNode(Node):
    """
    Listens to /Current_Pos, detects alert payloads, and forwards them to
    the appropriate MQTT topic.
    """
 
    def __init__(self):
        super().__init__("alert_center_node")
 
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
            depth=10,
        )
        self._pos_sub = self.create_subscription(
            String,
            CURRENT_POS_TOPIC,
            self._pos_callback,
            qos,
        )
 
        self.get_logger().info(
            f"AlertCenterNode ready — listening on '{CURRENT_POS_TOPIC}'"
        )
 
    # =========================================================================
    # MQTT HELPERS
    # =========================================================================
 
    def _connect_mqtt(self):
        """
        Try to connect to the broker.  If it fails, a background thread
        retries every 5 seconds so the node doesn't crash on startup.
        """
        def _try_connect():
            while not self._mqtt_connected:
                try:
                    self._mqtt.connect(
                        MQTT_BROKER_HOST,
                        MQTT_BROKER_PORT,
                        MQTT_KEEPALIVE,
                    )
                    self._mqtt.loop_start()   # background network thread
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
                f"[MQTT] Connected to broker {MQTT_BROKER_HOST}:{MQTT_BROKER_PORT}"
            )
        else:
            self.get_logger().error(f"[MQTT] Connection refused — rc={rc}")
 
    def _on_mqtt_disconnect(self, client, userdata, rc):
        self._mqtt_connected = False
        if rc != 0:
            self.get_logger().warn(
                f"[MQTT] Unexpected disconnection (rc={rc}). "
                "Paho will attempt automatic reconnect."
            )
 
    def _on_mqtt_publish(self, client, userdata, mid):
        self.get_logger().info(f"[MQTT] Message delivered (mid={mid})")
 
    def _publish_alert(self, topic: str, payload: dict):
        """
        Serialize payload to JSON and publish on the given MQTT topic.
        Drops the message with a warning if the broker is unreachable.
        """
        if not self._mqtt_connected:
            self.get_logger().warn(
                f"[MQTT] Broker not connected — alert to '{topic}' dropped!"
            )
            return
 
        json_payload = json.dumps(payload, ensure_ascii=False)
 
        with self._mqtt_lock:
            result = self._mqtt.publish(
                topic,
                payload=json_payload,
                qos=MQTT_QOS,
                retain=MQTT_RETAIN,
            )
 
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            self.get_logger().error(
                f"[MQTT] Publish failed on '{topic}' — rc={result.rc}"
            )
        else:
            self.get_logger().warn(
                f"[MQTT] *** ALERT published → topic='{topic}' payload={json_payload}"
            )
 
    # =========================================================================
    # ROS CALLBACK
    # =========================================================================
 
    def _pos_callback(self, msg: String):
        raw = msg.data.strip()
 
        # ── Fast-path: skip normal position messages ──────────────────────────
        # The main nav node appends " | PRIORITY: <alert_text>" to alert msgs.
        # Ordinary messages look like: "row=2,col=0,stop=B"
        if not ALERT_MARKER.search(raw):
            return   # normal position publish — nothing to do
 
        self.get_logger().warn(f"[ALERT_CENTER] Alert message received: {raw}")
 
        # ── Parse position fields ─────────────────────────────────────────────
        position = self._parse_position(raw)
 
        # ── Match alert keywords → MQTT topics ───────────────────────────────
        matched = False
        for pattern, mqtt_topic in ALERT_RULES:
            m = pattern.search(raw)
            if m:
                payload = {
                    "alert_type":  mqtt_topic.lstrip("/").upper(),
                    "keyword":     m.group(0).upper(),
                    "raw_message": raw,
                    "position":    position,
                    "timestamp":   time.time(),
                }
                self._publish_alert(mqtt_topic, payload)
                matched = True
 
        if not matched:
            # PRIORITY marker present but no known keyword — publish a generic
            # alert so nothing is silently lost.
            self.get_logger().warn(
                "[ALERT_CENTER] PRIORITY marker found but no keyword matched — "
                "publishing to /alert/unknown"
            )
            payload = {
                "alert_type":  "UNKNOWN",
                "keyword":     None,
                "raw_message": raw,
                "position":    position,
                "timestamp":   time.time(),
            }
            self._publish_alert("/alert/unknown", payload)
 
    # =========================================================================
    # UTILITIES
    # =========================================================================
 
    @staticmethod
    def _parse_position(raw: str) -> dict:
        """
        Extract row / col / stop from a position string.
        Returns a dict; unknown fields default to None.
        """
        row  = re.search(r"row=(-?\d+)",  raw)
        col  = re.search(r"col=(-?\d+)",  raw)
        stop = re.search(r"stop=(\S+)",   raw)
        return {
            "row":  int(row.group(1))  if row  else None,
            "col":  int(col.group(1))  if col  else None,
            "stop": stop.group(1)      if stop else None,
        }
 
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
    node = AlertCenterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()