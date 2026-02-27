import json, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def norm_label(s: str) -> str:
    s = (s or "").strip().lower()
    for ch in ["_", "-", "/"]:
        s = s.replace(ch, " ")
    return " ".join(s.split())

def get_bbox(det: dict):
    for k in ["bbox", "xyxy"]:
        if k in det and isinstance(det[k], (list, tuple)) and len(det[k]) == 4:
            return det[k]
    for a,b,c,d in [("x1","y1","x2","y2"), ("xmin","ymin","xmax","ymax")]:
        if a in det and b in det and c in det and d in det:
            return [det[a], det[b], det[c], det[d]]
    return None

def get_conf(det: dict) -> float:
    for k in ["conf", "confidence", "score", "prob", "p"]:
        if k in det and det[k] is not None:
            try: return float(det[k])
            except: pass
    return 1.0

def build_names_map(payload: dict):
    for k in ["names", "class_names", "labels", "class_map"]:
        if k in payload and isinstance(payload[k], dict):
            out = {}
            for kk, vv in payload[k].items():
                try: out[int(kk)] = str(vv)
                except:
                    try: out[int(kk)] = str(vv)
                    except: pass
            return out
    return {}

def det_label(det: dict, names_map: dict) -> str:
    for k in ["name", "label", "class_name"]:
        if k in det and det[k] is not None:
            return str(det[k])
    for k in ["class_id", "class", "cls", "id"]:
        if k in det and det[k] is not None:
            try:
                cid = int(det[k])
                return names_map.get(cid, str(cid))
            except:
                return str(det[k])
    return ""

class Supervisor(Node):
    def __init__(self):
        super().__init__("traffic_rules_supervisor_v5_stopfix")

        # Topics
        self.declare_parameter("cmd_in", "/cmd_vel_bc")
        self.declare_parameter("cmd_out", "/cmd_vel_nav")
        self.declare_parameter("dets_json", "/yolo/detections_json")

        # Caps
        self.declare_parameter("max_lin", 0.30)
        self.declare_parameter("max_ang", 2.0)

        # Yield (same as v5 behavior: close-gated + hold)
        self.declare_parameter("yield_scale", 0.25)
        self.declare_parameter("yield_hold_s", 4.0)
        self.declare_parameter("yield_h", 0.08)
        self.declare_parameter("yield_ymax", 0.82)
        self.declare_parameter("sign_min_conf", 0.55)

        # Stop (same trigger as before) + NEW: stop rearm ignores far detections
        self.declare_parameter("stop_duration_s", 3.0)
        self.declare_parameter("stop_cooldown_s", 8.0)
        self.declare_parameter("stop_rearm_s", 1.5)
        self.declare_parameter("stop_h", 0.12)
        self.declare_parameter("stop_ymax", 0.88)

        # NEW: what counts as "still seeing stop sign" for rearm logic
        self.declare_parameter("stop_seen_h", 0.05)
        self.declare_parameter("stop_seen_ymax", 0.65)

        # Traffic lights (same as v5 stable selection)
        self.declare_parameter("green_scale", 2.0)
        self.declare_parameter("yellow_clear_scale", 2.5)
        self.declare_parameter("red_hold_s", 2.5)
        self.declare_parameter("yellow_hold_s", 1.5)
        self.declare_parameter("green_hold_s", 0.8)
        self.declare_parameter("light_min_conf", 0.60)

        self.declare_parameter("cw_stop_ymax", 0.70)
        self.declare_parameter("cw_stop_h", 0.10)
        self.declare_parameter("cw_crossed_ymax", 0.93)
        self.declare_parameter("cw_crossed_h", 0.22)

        self.declare_parameter("light_roi_x", 0.28)
        self.declare_parameter("light_roi_ymax", 0.70)

        self.declare_parameter("debug", True)

        self.cmd_in = self.get_parameter("cmd_in").value
        self.cmd_out = self.get_parameter("cmd_out").value
        self.det_topic = self.get_parameter("dets_json").value

        self.sub_cmd = self.create_subscription(Twist, self.cmd_in, self.on_cmd, 10)
        self.sub_det = self.create_subscription(String, self.det_topic, self.on_det, 10)
        self.pub = self.create_publisher(Twist, self.cmd_out, 10)

        self.last_cmd = Twist()
        self.last_cmd_time = 0.0

        self.last_seen = {"red": 0.0, "yellow": 0.0, "green": 0.0, "yield": 0.0}
        self.latest_state = None
        self.latest_cw = None
        self.latest_light = None

        # stop state
        self.stop_until = 0.0
        self.last_stop_trigger = 0.0
        self.stop_armed = True
        self.last_stop_seen_close = 0.0  # <-- NEW

        self.best_stop = None
        self.best_yield = None

        self._dbg_t = 0.0
        self.timer = self.create_timer(1.0/20.0, self.tick)

    def on_cmd(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = time.time()

    def _normalize_boxes(self, dets, img_w, img_h):
        out = []
        for det in dets:
            bb = get_bbox(det)
            if not bb:
                continue
            x1,y1,x2,y2 = [float(v) for v in bb]
            if max(x2,y2) > 2.0:
                w = float(img_w or 1280.0); h = float(img_h or 720.0)
                x1/=w; x2/=w; y1/=h; y2/=h
            x1 = max(0.0, min(1.0, x1)); x2 = max(0.0, min(1.0, x2))
            y1 = max(0.0, min(1.0, y1)); y2 = max(0.0, min(1.0, y2))
            d2 = dict(det)
            d2["_bb"] = (x1,y1,x2,y2)
            out.append(d2)
        return out

    def on_det(self, msg: String):
        now = time.time()
        try:
            payload = json.loads(msg.data)
        except Exception:
            return

        dets = None
        if isinstance(payload, dict):
            for k in ["detections","dets","objects","predictions"]:
                if k in payload and isinstance(payload[k], list):
                    dets = payload[k]
                    break
        if dets is None and isinstance(payload, list):
            dets = payload
        if dets is None:
            dets = []

        img_w = payload.get("image_w") if isinstance(payload, dict) else None
        img_h = payload.get("image_h") if isinstance(payload, dict) else None
        if isinstance(payload, dict):
            img_w = img_w or payload.get("width")
            img_h = img_h or payload.get("height")

        names_map = build_names_map(payload) if isinstance(payload, dict) else {}
        dets = self._normalize_boxes(dets, img_w, img_h)

        sign_min_conf = float(self.get_parameter("sign_min_conf").value)
        light_min_conf = float(self.get_parameter("light_min_conf").value)
        roi_x = float(self.get_parameter("light_roi_x").value)
        roi_ymax = float(self.get_parameter("light_roi_ymax").value)

        stop_h_thr = float(self.get_parameter("stop_h").value)
        stop_ymax_thr = float(self.get_parameter("stop_ymax").value)
        stop_seen_h = float(self.get_parameter("stop_seen_h").value)
        stop_seen_ymax = float(self.get_parameter("stop_seen_ymax").value)

        yield_h_thr = float(self.get_parameter("yield_h").value)
        yield_ymax_thr = float(self.get_parameter("yield_ymax").value)

        self.best_stop = None
        self.best_yield = None

        crosswalks = []
        housings = []
        red_lens = []
        yellow_lens = []
        green_lens = []

        stop_trigger_close = False
        yield_trigger_close = False

        for d in dets:
            raw = det_label(d, names_map)
            nl = norm_label(raw)

            x1,y1,x2,y2 = d["_bb"]
            xc = 0.5*(x1+x2); yc = 0.5*(y1+y2)
            h = (y2-y1)
            conf = get_conf(d)
            area = max(0.0,(x2-x1))*max(0.0,h)

            if nl == "crosswalk":
                crosswalks.append((area, xc, yc, (x1,y1,x2,y2)))

            # STOP sign: NEW rearm logic (ignore far detections)
            if nl in ["stop sign", "stop"] and conf >= sign_min_conf:
                cand = (conf, h, y2)
                if self.best_stop is None or cand > self.best_stop:
                    self.best_stop = cand

                # only update "still seeing sign" if it’s at least moderately close
                if (h >= stop_seen_h) or (y2 >= stop_seen_ymax):
                    self.last_stop_seen_close = now

                # actual stop trigger (close)
                if (h >= stop_h_thr) or (y2 >= stop_ymax_thr):
                    stop_trigger_close = True

            # YIELD sign (close gated + hold)
            if nl in ["yield sign", "yield", "yield sign."] and conf >= sign_min_conf:
                cand = (conf, h, y2)
                if self.best_yield is None or cand > self.best_yield:
                    self.best_yield = cand
                if (h >= yield_h_thr) or (y2 >= yield_ymax_thr):
                    yield_trigger_close = True

            # Traffic lights (ROI + conf)
            in_roi = (abs(xc-0.5) < roi_x) and (yc < roi_ymax)
            if in_roi and conf >= light_min_conf:
                if nl in ["traffic light", "trafficlight", "traffic light."]:
                    score = area - 1.5*abs(xc-0.5)
                    housings.append((score, (x1,y1,x2,y2)))
                elif nl in ["red light", "red"]:
                    red_lens.append((x1,y1,x2,y2))
                elif nl in ["yellow light", "yellow", "amber light", "amber"]:
                    yellow_lens.append((x1,y1,x2,y2))
                elif nl in ["green light", "green"]:
                    green_lens.append((x1,y1,x2,y2))

        # Rearm stop when "close-seen" disappears
        if (now - self.last_stop_seen_close) > float(self.get_parameter("stop_rearm_s").value):
            self.stop_armed = True

        # Trigger stop once per approach (armed + cooldown)
        stop_duration = float(self.get_parameter("stop_duration_s").value)
        stop_cooldown = float(self.get_parameter("stop_cooldown_s").value)
        if stop_trigger_close and self.stop_armed and (now - self.last_stop_trigger) > stop_cooldown:
            self.last_stop_trigger = now
            self.stop_until = now + stop_duration
            self.stop_armed = False

        # Yield hold
        if yield_trigger_close:
            self.last_seen["yield"] = now

        # Choose best housing
        chosen_light = None
        if housings:
            housings.sort(reverse=True)
            chosen_light = housings[0][1]
        self.latest_light = chosen_light

        def inside(hbb, bb):
            if hbb is None:
                return True
            hx1,hy1,hx2,hy2 = hbb
            x1,y1,x2,y2 = bb
            cx = 0.5*(x1+x2); cy = 0.5*(y1+y2)
            return (hx1 <= cx <= hx2) and (hy1 <= cy <= hy2)

        red_seen = any(inside(chosen_light, bb) for bb in red_lens)
        yellow_seen = any(inside(chosen_light, bb) for bb in yellow_lens)
        green_seen = any(inside(chosen_light, bb) for bb in green_lens)

        if red_seen: self.last_seen["red"] = now
        if yellow_seen: self.last_seen["yellow"] = now
        if green_seen: self.last_seen["green"] = now

        red_hold = float(self.get_parameter("red_hold_s").value)
        yellow_hold = float(self.get_parameter("yellow_hold_s").value)
        green_hold = float(self.get_parameter("green_hold_s").value)

        state = None
        if (now - self.last_seen["red"]) < red_hold:
            state = "red"
        elif (now - self.last_seen["yellow"]) < yellow_hold:
            state = "yellow"
        elif (now - self.last_seen["green"]) < green_hold:
            state = "green"
        self.latest_state = state

        # Crosswalk: align with light (handles double crosswalk)
        chosen_cw = None
        if crosswalks:
            if chosen_light is not None:
                lx1,ly1,lx2,ly2 = chosen_light
                best = None
                for (a,xc,yc,bb) in crosswalks:
                    x1,y1,x2,y2 = bb
                    gap = y1 - ly2
                    x_align = abs(xc - 0.5*(lx1+lx2))
                    if gap <= 0:
                        gap = abs(gap) + 1.0
                    score = gap + 0.75*x_align
                    if best is None or score < best[0]:
                        best = (score, bb)
                chosen_cw = best[1]
            else:
                crosswalks.sort(key=lambda t: t[3][3], reverse=True)
                chosen_cw = crosswalks[0][3]
        self.latest_cw = chosen_cw

    def tick(self):
        now = time.time()
        if (now - self.last_cmd_time) > 0.5:
            self.pub.publish(Twist())
            self._dbg(now, stale=True)
            return

        out = Twist()
        out.linear.x = float(self.last_cmd.linear.x)
        out.angular.z = float(self.last_cmd.angular.z)

        # Stop override
        if now < self.stop_until:
            out.linear.x = 0.0
            out.angular.z = 0.0
            self.pub.publish(out)
            self._dbg(now)
            return

        # Crosswalk proximity
        cw_stop = False
        cw_crossed = False
        if self.latest_cw is not None:
            x1,y1,x2,y2 = self.latest_cw
            h = (y2-y1)
            cw_stop = (y2 >= float(self.get_parameter("cw_stop_ymax").value)) or (h >= float(self.get_parameter("cw_stop_h").value))
            cw_crossed = (y2 >= float(self.get_parameter("cw_crossed_ymax").value)) or (h >= float(self.get_parameter("cw_crossed_h").value))

        # Traffic light behavior
        st = self.latest_state
        if st == "red":
            if cw_stop:
                out.linear.x = 0.0; out.angular.z = 0.0
            else:
                out.linear.x *= 0.35
        elif st == "yellow":
            if cw_stop and (not cw_crossed):
                out.linear.x = 0.0; out.angular.z = 0.0
            else:
                out.linear.x *= float(self.get_parameter("yellow_clear_scale").value)
        elif st == "green":
            out.linear.x *= float(self.get_parameter("green_scale").value)

        # Yield hold
        if (now - self.last_seen["yield"]) < float(self.get_parameter("yield_hold_s").value):
            out.linear.x *= float(self.get_parameter("yield_scale").value)

        # Caps
        max_lin = float(self.get_parameter("max_lin").value)
        max_ang = float(self.get_parameter("max_ang").value)
        out.linear.x = max(-max_lin, min(max_lin, out.linear.x))
        out.angular.z = max(-max_ang, min(max_ang, out.angular.z))

        self.pub.publish(out)
        self._dbg(now)

    def _dbg(self, now, stale=False):
        if not bool(self.get_parameter("debug").value):
            return
        if (now - self._dbg_t) < 0.8:
            return
        self._dbg_t = now
        self.get_logger().info(
            f"stale={stale} stop_active={now<self.stop_until} stop_armed={self.stop_armed} "
            f"TL={self.latest_state} yield_age={(now-self.last_seen['yield']):.2f}s "
            f"best_stop={self.best_stop} best_yield={self.best_yield}"
        )

def main():
    rclpy.init()
    n = Supervisor()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
