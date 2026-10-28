"""Sensor-only red-team baseline: frontier exploration, mapping and placement.

Each drone keeps its own map.  Only observations received through the official
short-range communicator are merged; no simulator map or true pose is read.
"""

from __future__ import annotations

import heapq
import math

import cv2
import numpy as np

from swarm_rescue.simulation.drone.controller import CommandsDict
from swarm_rescue.simulation.drone.drone_abstract import DroneAbstract


class MyDronePlaceBaseline(DroneAbstract):
    CELL = 12
    WALL_BIN = 6
    SCAN_PERIOD = 4
    PLAN_PERIOD = 35
    MAX_RANGE = 300.0
    SUBMIT_PEER_TTL = 12
    GOAL_PROGRESS_TIMEOUT = 140

    def __init__(self, identifier=None, misc_data=None, **kwargs):
        super().__init__(identifier=identifier, misc_data=misc_data, **kwargs)
        self._step = 0
        self._pos = None
        self._heading = None
        self._start = None
        self._initial_inventory = None
        self._last_inventory = None
        self._last_place_step = -1000
        self._placed_positions = []
        self._peer = {}
        self._peer_ready = {}
        self._completed = set()
        self._wall_votes = {}
        self._wall_set = set()
        self._wall_list = []
        self._send_cursor = 0
        self._goal = None
        self._waypoints = []
        self._last_plan_step = -100
        self._blocked_until = {}
        self._last_progress_pos = None
        self._last_progress_step = 0
        self._progress_goal = None
        self._goal_best_distance = float("inf")
        self._goal_progress_step = 0
        self._goal_timeouts = 0
        self._escape_until = 0
        self._escape_sign = 1.0
        self._submitted = False
        self._ready_since = None
        if self.size_area is not None:
            w, h = (int(v) for v in self.size_area)
            shape = ((h + self.CELL - 1) // self.CELL,
                     (w + self.CELL - 1) // self.CELL)
            self._free = np.zeros(shape, dtype=np.bool_)
            self._hits = np.zeros(shape, dtype=np.uint8)
            self._visits = np.zeros(shape, dtype=np.uint8)
        else:
            self._free = self._hits = self._visits = None

    def define_message_for_all(self):
        # Messages are collected before control() in the current simulator step.
        n = min(72, len(self._wall_list))
        chunk = []
        if n:
            for i in range(n):
                chunk.append(self._wall_list[(self._send_cursor + i) % len(self._wall_list)])
            self._send_cursor = (self._send_cursor + n) % len(self._wall_list)
        return {"id": int(self.identifier), "step": self._step,
                "pos": self._pos, "goal": self._goal,
                "inv": self.carried_bombs_count(),
                "ready": self._submission_due(),
                "done": sorted(self._completed),
                "walls": chunk, "bombs": self._placed_positions[-10:]}

    def control(self) -> CommandsDict:
        self._step += 1
        cmd = {"forward": 0.0, "lateral": 0.0, "rotation": 0.0,
               "grasper": 0, "place_bomb": 0}
        self._update_pose()
        self._read_messages()
        inventory = self.carried_bombs_count()
        if inventory == 0:
            self._completed.add(int(self.identifier))
        if self._initial_inventory is None:
            self._initial_inventory = inventory
        if self._last_inventory is not None and inventory < self._last_inventory and self._pos:
            self._remember_bomb(self._pos)
        self._last_inventory = inventory

        lidar = self.lidar_values()
        angles = self.lidar_rays_angles()
        if (self._pos is not None and self._heading is not None and
                lidar is not None and self._free is not None and
                self._step % self.SCAN_PERIOD == 0):
            self._scan(lidar, angles)

        if self._pos is not None and self._free is not None:
            cell = self._cell(self._pos)
            if cell is not None:
                self._visits[cell] = min(255, int(self._visits[cell]) + 1)
        self._check_stuck()
        self._check_goal_progress()
        if self._step < self._escape_until:
            cmd["forward"] = -0.4 if self._escape_until - self._step > 14 else 0.2
            cmd["rotation"] = 0.7 * self._escape_sign
        else:
            if (self._step - self._last_plan_step >= self.PLAN_PERIOD or
                    (not self._waypoints and self._step - self._last_plan_step >= 12)):
                self._plan()
                self._last_plan_step = self._step
            self._drive(cmd)
        self._avoid(cmd, lidar, angles)
        self._maybe_place(cmd, lidar, angles)
        self._maybe_submit()
        return cmd

    def _update_pose(self):
        gps = self.measured_gps_position()
        compass = self.measured_compass_angle()
        odom = self.odometer_values()
        old_heading = self._heading
        if gps is not None and np.all(np.isfinite(gps)):
            self._pos = (float(gps[0]), float(gps[1]))
            if self._start is None:
                self._start = self._pos
        elif (self._pos is not None and old_heading is not None and
              odom is not None and np.all(np.isfinite(odom))):
            distance, alpha = float(odom[0]), float(odom[1])
            self._pos = (self._pos[0] + distance * math.cos(old_heading + alpha),
                         self._pos[1] + distance * math.sin(old_heading + alpha))
        if compass is not None and math.isfinite(compass):
            self._heading = float(compass)
        elif old_heading is not None and odom is not None and np.all(np.isfinite(odom)):
            self._heading = self._wrap(old_heading + float(odom[2]))

    def _read_messages(self):
        for _, msg in self.communicator.received_messages:
            if not isinstance(msg, dict) or not isinstance(msg.get("id"), int):
                continue
            peer_id = msg["id"]
            if peer_id == int(self.identifier):
                continue
            self._peer[peer_id] = (self._step, msg.get("inv"),
                                   msg.get("pos"), msg.get("goal"))
            self._peer_ready[peer_id] = (self._step, msg.get("ready") is True)
            if msg.get("inv") == 0:
                self._completed.add(peer_id)
            # Completion can travel through intermediate drones after direct
            # contact with the submitting drone has been lost.
            done = msg.get("done")
            if isinstance(done, (list, tuple)):
                count = self._misc_data.number_drones if self._misc_data else 0
                self._completed.update(i for i in done
                                       if isinstance(i, int) and 0 <= i < count)
            for point in msg.get("walls", ()):
                if (isinstance(point, (tuple, list)) and len(point) == 2 and
                        all(isinstance(v, int) for v in point)):
                    self._add_wall((point[0], point[1]))
            for point in msg.get("bombs", ()):
                if (isinstance(point, (tuple, list)) and len(point) == 2 and
                        all(isinstance(v, (int, float)) for v in point)):
                    self._remember_bomb((float(point[0]), float(point[1])))

    def _scan(self, lidar, angles):
        w, h = (int(v) for v in self.size_area)
        px, py = self._pos
        origin = self._cell(self._pos)
        if origin is None:
            return
        semantic = self.semantic_values()
        excluded = []
        if semantic is not None:
            for obj in semantic:
                if (math.isfinite(obj.distance) and obj.distance > 0 and
                        obj.entity_type.name in ("BOMB", "DRONE", "DISPOSAL_CENTER")):
                    excluded.append((float(obj.distance), float(obj.angle)))
        scan_bins = set()
        for i in range(0, min(len(lidar), len(angles)), 2):
            distance = float(lidar[i])
            if not math.isfinite(distance) or distance <= 3:
                continue
            distance = min(self.MAX_RANGE, distance)
            angle = float(angles[i])
            direction = self._heading + angle
            c, s = math.cos(direction), math.sin(direction)
            # A margin before a noisy endpoint keeps a wall from being cleared.
            clear_length = max(0.0, distance - 13.0)
            if i % 4 == 0:
                for r in range(0, int(clear_length), self.CELL):
                    cell = self._cell((px + r * c, py + r * s))
                    if cell is not None:
                        self._free[cell] = True
            if distance >= self.MAX_RANGE - 8:
                continue
            if any(abs(self._wrap(angle - a)) < 0.11 and abs(distance - d) < 24
                   for d, a in excluded):
                continue
            x, y = px + distance * c, py + distance * s
            if not (0 <= x + w / 2 < w and 0 <= h / 2 - y < h):
                continue
            cell = self._cell((x, y))
            if cell is not None:
                self._hits[cell] = min(255, int(self._hits[cell]) + 1)
            key = (int(round((x + w / 2) / self.WALL_BIN)),
                   int(round((h / 2 - y) / self.WALL_BIN)))
            scan_bins.add(key)
        for key in scan_bins:
            count = self._wall_votes.get(key, 0) + 1
            self._wall_votes[key] = count
            if count == 2:
                self._add_wall(key)
        self._free[origin] = True

    def _add_wall(self, key):
        if key not in self._wall_set:
            self._wall_set.add(key)
            self._wall_list.append(key)

    def _cell(self, pos):
        if self._free is None or self.size_area is None:
            return None
        w, h = self.size_area
        c = int((pos[0] + w / 2) // self.CELL)
        r = int((h / 2 - pos[1]) // self.CELL)
        if 0 <= r < self._free.shape[0] and 0 <= c < self._free.shape[1]:
            return r, c
        return None

    def _world(self, cell):
        w, h = self.size_area
        return ((cell[1] + 0.5) * self.CELL - w / 2,
                h / 2 - (cell[0] + 0.5) * self.CELL)

    def _plan(self):
        if self._free is None or self._pos is None:
            return
        start = self._cell(self._pos)
        if start is None:
            return
        # Two-cell inflation leaves room for the 10 px drone and pose noise.
        blocked = cv2.dilate((self._hits >= 2).astype(np.uint8),
                             np.ones((3, 3), np.uint8)).astype(bool)
        allowed = self._free & ~blocked
        allowed[start] = True
        dist = {start: 0.0}
        parent = {}
        queue = [(0.0, start)]
        best = None
        best_score = -1e9
        rows, cols = allowed.shape
        peers = [p for t, _, p, _ in self._peer.values()
                 if self._step - t < 80 and p is not None]
        while queue:
            cost, cell = heapq.heappop(queue)
            if cost != dist[cell]:
                continue
            r, c = cell
            if cost > 4:
                near_unknown = (not self._free[max(0, r - 1):min(rows, r + 2),
                                                max(0, c - 1):min(cols, c + 2)].all())
                if near_unknown and self._blocked_until.get(cell, 0) < self._step:
                    gain = int(np.count_nonzero(
                        ~self._free[max(0, r - 5):min(rows, r + 6),
                                    max(0, c - 5):min(cols, c + 6)]))
                    wx, wy = self._world(cell)
                    peer_penalty = sum(max(0.0, 110.0 - math.hypot(wx - p[0], wy - p[1])) / 5
                                       for p in peers)
                    revisit = float(self._visits[r, c]) * 0.3
                    score = gain - 0.27 * cost - revisit - peer_penalty
                    if score > best_score:
                        best_score, best = score, cell
            for dr, dc, weight in ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0),
                                   (0, -1, 1.0), (1, 1, 1.42), (1, -1, 1.42),
                                   (-1, 1, 1.42), (-1, -1, 1.42)):
                nxt = (r + dr, c + dc)
                if (0 <= nxt[0] < rows and 0 <= nxt[1] < cols and allowed[nxt] and
                        (dr == 0 or dc == 0 or
                         (allowed[r + dr, c] and allowed[r, c + dc]))):
                    new_cost = cost + weight
                    if new_cost < dist.get(nxt, float("inf")):
                        dist[nxt] = new_cost
                        parent[nxt] = cell
                        heapq.heappush(queue, (new_cost, nxt))
        self._goal = best
        self._waypoints = []
        if best is not None:
            cell = best
            while cell != start:
                self._waypoints.append(cell)
                cell = parent[cell]
            self._waypoints.reverse()

    def _drive(self, cmd):
        if self._pos is None or self._heading is None or not self._waypoints:
            cmd["forward"] = 0.4
            cmd["rotation"] = 0.3 if int(self.identifier) % 2 == 0 else -0.3
            return
        while len(self._waypoints) > 1:
            x, y = self._world(self._waypoints[0])
            if math.hypot(x - self._pos[0], y - self._pos[1]) >= 20:
                break
            self._waypoints.pop(0)
        target = self._world(self._waypoints[min(3, len(self._waypoints) - 1)])
        desired = math.atan2(target[1] - self._pos[1], target[0] - self._pos[0])
        error = self._wrap(desired - self._heading)
        cmd["rotation"] = max(-0.75, min(0.75, 1.5 * error))
        cmd["forward"] = 0.75 if abs(error) < 0.55 else 0.2
        if self._goal and math.hypot(*(a - b for a, b in zip(self._world(self._goal), self._pos))) < 22:
            self._blocked_until[self._goal] = self._step + 100
            self._waypoints = []

    def _avoid(self, cmd, lidar, angles):
        if lidar is None or angles is None or cmd["forward"] < 0:
            return
        arr = np.asarray(lidar)
        aa = np.asarray(angles)
        valid = np.isfinite(arr)
        front = arr[valid & (np.abs(aa) < 0.28)]
        if not len(front):
            return
        clearance = float(np.min(front))
        if clearance >= 47:
            return
        left = arr[valid & (aa > 0.16) & (aa < 0.95)]
        right = arr[valid & (aa < -0.16) & (aa > -0.95)]
        left_clear = float(np.percentile(left, 25)) if len(left) else 0.0
        right_clear = float(np.percentile(right, 25)) if len(right) else 0.0
        sign = 1.0 if left_clear >= right_clear else -1.0
        cmd["rotation"] = sign * 0.8
        cmd["forward"] = 0.0 if clearance < 35 else min(cmd["forward"], 0.25)

    def _check_stuck(self):
        if self._pos is None:
            return
        if self._last_progress_pos is None:
            self._last_progress_pos = self._pos
            self._last_progress_step = self._step
        if self._step - self._last_progress_step < 38:
            return
        movement = math.hypot(self._pos[0] - self._last_progress_pos[0],
                              self._pos[1] - self._last_progress_pos[1])
        if movement < 17:
            if self._goal is not None:
                self._blocked_until[self._goal] = self._step + 180
            self._waypoints = []
            self._escape_until = self._step + 30
            self._escape_sign = -1.0 if int(self.identifier) % 2 else 1.0
        self._last_progress_pos = self._pos
        self._last_progress_step = self._step

    def _check_goal_progress(self):
        # 只跟踪本机估计位姿和已规划路径。使用剩余路径长度，允许正常
        # 绕障时暂时远离终点；24 px 的改善门槛防止定位抖动不断刷新计时。
        if (self._pos is None or self._goal is None or not self._waypoints or
                self._step < self._escape_until):
            self._progress_goal = None
            return
        if self._step % self.SCAN_PERIOD:
            return
        previous = self._pos
        remaining = 0.0
        for cell in self._waypoints:
            point = self._world(cell)
            remaining += math.hypot(point[0] - previous[0], point[1] - previous[1])
            previous = point
        same_region = (self._progress_goal is not None and
                       max(abs(a - b) for a, b in
                           zip(self._goal, self._progress_goal)) <= 2)
        if not same_region or remaining <= self._goal_best_distance - 2 * self.CELL:
            self._progress_goal = self._goal
            self._goal_best_distance = remaining
            self._goal_progress_step = self._step
            return
        if self._step - self._goal_progress_step < self.GOAL_PROGRESS_TIMEOUT:
            return
        # 禁选小邻域，避免下一次规划只把目标挪一格又回到同一困境。
        row, col = self._goal
        rows, cols = self._free.shape
        for r in range(max(0, row - 2), min(rows, row + 3)):
            for c in range(max(0, col - 2), min(cols, col + 3)):
                self._blocked_until[(r, c)] = max(
                    self._blocked_until.get((r, c), 0), self._step + 180)
        self._goal_timeouts += 1
        print(f"[place-baseline] goal timeout id={self.identifier}, step={self._step}")
        self._goal = None
        self._waypoints = []
        self._progress_goal = None
        self._last_plan_step = self._step - self.PLAN_PERIOD

    def _maybe_place(self, cmd, lidar, angles):
        inventory = self.carried_bombs_count()
        if inventory <= 0 or self._pos is None:
            return
        limit = self._limit()
        placed_self = max(0, (self._initial_inventory or 0) - inventory)
        earliest = (0.15 if placed_self == 0 else 0.35) * limit + 12 * int(self.identifier)
        if (self._step < earliest or self._step - self._last_place_step < 35 or
                self._step % 10 != int(self.identifier) % 10):
            return
        if (self._step < 0.45 * limit and self._start and
                math.hypot(self._pos[0] - self._start[0],
                           self._pos[1] - self._start[1]) < 70):
            return
        separation = min((math.hypot(self._pos[0] - p[0], self._pos[1] - p[1])
                          for p in self._placed_positions), default=float("inf"))
        if separation < (145 if self._step < 0.52 * limit else 35):
            return
        near_wall = False
        if lidar is not None:
            vals = np.asarray(lidar)
            good = vals[np.isfinite(vals) & (vals > 30) & (vals < 145)]
            near_wall = len(good) > 7
        if not near_wall and self._step < 0.58 * limit:
            return
        # Approximate the marginal blast coverage from observed wall points.
        # The published scoring radius defaults to 12% of the shorter side.
        radius = 0.12 * min(self.size_area)
        gain = 0
        w, h = self.size_area
        for col, row in self._wall_set:
            x, y = col * self.WALL_BIN - w / 2, h / 2 - row * self.WALL_BIN
            if math.hypot(x - self._pos[0], y - self._pos[1]) > radius:
                continue
            if any(math.hypot(x - p[0], y - p[1]) <= radius
                   for p in self._placed_positions):
                continue
            gain += 1
        if gain < 18 and self._step < 0.55 * limit:
            return
        cmd["place_bomb"] = 1
        self._last_place_step = self._step

    def _remember_bomb(self, pos):
        if all(math.hypot(pos[0] - p[0], pos[1] - p[1]) > 20
               for p in self._placed_positions):
            self._placed_positions.append(pos)

    def _submission_due(self):
        limit = self._limit()
        count = self._misc_data.number_drones if self._misc_data else None
        all_done = (count is not None and self.carried_bombs_count() == 0 and
                    all(i in self._completed for i in range(count)))
        return ((all_done and self._step >= 0.58 * limit) or
                self._step >= 0.82 * limit)

    def _maybe_submit(self):
        if self._submitted or self._free is None or not self._submission_due():
            return
        if self._ready_since is None:
            self._ready_since = self._step
        # 备用机首次就绪后先传播完成消息，避免比 0 号机早收到最后一条
        # done 时抢先结束正常局。等待有上限，完全失联仍能自行提交。
        delay = (min(self.SUBMIT_PEER_TTL + 4 * int(self.identifier),
                     int(0.04 * self._limit())) if int(self.identifier) else 0)
        if self._step - self._ready_since < delay:
            return
        # 仅向通过官方通信刚刚宣布 ready 的低编号让位；损毁或断联后
        # 消息最多保留 12 步，不能永久锁住提交。所有状态均为实例私有。
        if any(peer_id < int(self.identifier) and ready and
               self._step - heard < self.SUBMIT_PEER_TTL
               for peer_id, (heard, ready) in self._peer_ready.items()):
            return
        count = self._misc_data.number_drones if self._misc_data else None
        w, h = (int(v) for v in self.size_area)
        grid = np.zeros((h, w), dtype=np.uint8)
        for col, row in self._wall_set:
            x, y = col * self.WALL_BIN, row * self.WALL_BIN
            if 0 <= x < w and 0 <= y < h:
                # A confirmed bin represents a complete cell. Drawing circles
                # leaves gaps between adjacent bins even on a continuous wall.
                half = self.WALL_BIN // 2
                grid[max(0, y - half):min(h, y - half + self.WALL_BIN),
                     max(0, x - half):min(w, x - half + self.WALL_BIN)] = 1
        # The public size describes a ClosedPlayground with a 6 px wall frame.
        cv2.rectangle(grid, (2, 2), (w - 3, h - 3), 1, 5)
        print(f"[place-baseline] submit id={self.identifier}, step={self._step}, walls={len(self._wall_set)}, "
              f"inventory={self.carried_bombs_count()}, peers={len(self._peer)}, "
              f"done={len(self._completed)}/{count}")
        self.submit_exploration_map(grid)
        self._submitted = True

    def _limit(self):
        value = self._misc_data.max_timestep_limit if self._misc_data else None
        return int(value) if value else 2000

    @staticmethod
    def _wrap(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi
