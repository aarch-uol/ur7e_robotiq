"""SETUP > Flask Geometry and the flask scrape generator: the web commands (a WebService mixin).

Nothing here moves the robot. Capture reads where the TCP is; Save writes the "flask" fixture and a flask profile;
Check reach and Make program only compute (the program opens unsaved in PROGRAMS for the operator to review).
Logic without the web is in flask_setup.py, flask_geometry.py and flask_scrape.py.
"""
from __future__ import annotations

from typing import Dict, List, Optional

import numpy as np

from .. import cart_speed
from .. import flask_scrape as fs
from .. import flask_setup as FS
from .. import transforms as tf
from ..calibration import FixtureDef, config_dir
from ..flask_geometry import FlaskGeometry, FlaskGeometryError

FLASK_ADMIN = ("flask_capture", "flask_remove", "flask_reset", "flask_fake_touches", "flask_save",
               "flask_profile_delete")


def _plan_band(plan):
    """The widest run of the plan's polar angles reached at every angle round (None: none)."""
    thetas = sorted({p.theta_deg for p in plan.reachable} | {u.theta_deg for u in plan.unreachable})
    bad = {u.theta_deg for u in plan.unreachable}
    runs, cur = [], []
    for t in thetas:
        if t in bad:
            cur = []
            continue
        if not cur:
            runs.append(cur)
        cur.append(t)
    best = max(runs, key=lambda r: (r[-1] - r[0], len(r)), default=None)
    return (best[0], best[-1]) if best else None


class FlaskCommands:
    def _flask_init(self):
        self.flask_points: Dict[str, List[List[float]]] = {k: [] for k in FS.KINDS}
        self.flask_settings = FS.fit_settings()
        self.flasks = FS.FlaskLibrary(config_dir() / "flasks" / f"{self.core.profile.name}.yaml",
                                      self.core.profile.base_link)

    # ------------------------------------------------------------ capture and fit
    def _flask_set(self, tip_mm=None, wall_mm=None, neck_id_mm=None):
        self.flask_settings = FS.fit_settings(tip_mm, wall_mm, neck_id_mm, self.flask_settings)

    def _flask_fit(self) -> Optional[FlaskGeometry]:
        p = self.flask_points
        if len(p["rim"]) < FS.MIN_POINTS["rim"] or len(p["bulb"]) < FS.MIN_POINTS["bulb"]:
            return None
        return FS.fit(p, self.flask_settings, self.core.profile.base_link)

    def flask_view(self) -> dict:
        out = {"points": self.flask_points, "settings": self.flask_settings, "result": None, "error": "",
               "hint": None, "profiles": self.flasks.view(self.core.store.fixtures), "fixture": self._flask_fixture()}
        g = None
        try:
            g = self._flask_fit()
            if g is not None:
                out["result"] = FS.geometry_view(g)
        except FlaskGeometryError as e:
            out["error"] = str(e)[:1].upper() + str(e)[1:] + "."
        out["hint"] = FS.next_touch(self.flask_points, g)
        return out

    def _flask_fixture(self) -> Optional[dict]:
        """The saved "flask" fixture as the 3D view and the generator see it, or None."""
        fx = self.core.store.fixtures.get(FS.FLASK_FIXTURE)
        if fx is None or not isinstance(fx.meta.get("flask_geometry"), dict):
            return None
        try:
            g = FlaskGeometry.from_dict(fx.meta["flask_geometry"])
            T = self.core.geo.T_base_frame(FS.FLASK_FIXTURE)
        except Exception as e:                       # noqa: BLE001 - shown on the page, not a crash
            return {"error": f"the flask fixture is unusable: {e}"}
        return {"profile": fx.meta.get("flask_profile", ""), "size": g.standard_size,
                "od_mm": round(2000.0 * g.R_outer, 2), "draw": FS.draw_shapes(g, T)}

    def cmd_flask_get(self, tip_mm=None, wall_mm=None, neck_id_mm=None):
        """The touch points, the fit for these numbers (they are kept for the next capture) and the next-touch hint.
        Read only: the numbers only change what the page is shown."""
        self._flask_set(tip_mm, wall_mm, neck_id_mm)
        return self._ok(flask=self.flask_view())
    cmd_flask_get.read_only = True

    def cmd_flask_capture(self, kind, tip_mm=None, wall_mm=None, neck_id_mm=None):
        """Add where the active TCP is now (base frame) as a rim, bulb or neck-side touch."""
        if kind not in FS.KINDS:
            raise FS.FlaskSetupError(f"kind must be one of {', '.join(FS.KINDS)}")
        self._flask_set(tip_mm, wall_mm, neck_id_mm)
        if len(self.flask_points[kind]) >= FS.MAX_POINTS:
            raise FS.FlaskSetupError(f"{FS.MAX_POINTS} {kind} points are enough: remove some first")
        T = self.core.geo.current_pose(self.core.profile.base_link, self.core.store.active_tcp).T()
        self.flask_points[kind].append([float(v) for v in T[:3, 3]])
        return self._ok(f"{kind} point {len(self.flask_points[kind])} captured", flask=self.flask_view())

    def cmd_flask_remove(self, kind, index, **fit):
        self._flask_set(**fit)
        try:
            self.flask_points[kind].pop(int(index))
        except (KeyError, IndexError, TypeError, ValueError):
            raise FS.FlaskSetupError("no such touch point") from None
        return self._ok("removed", flask=self.flask_view())

    def cmd_flask_reset(self, **fit):
        self._flask_set(**fit)
        self.flask_points = {k: [] for k in FS.KINDS}
        return self._ok("cleared", flask=self.flask_view())

    def cmd_flask_fake_touches(self, noise_mm=0.2, **fit):
        """Simulated robot only: touches on a 250 ml flask whose rim is 5 cm below the TCP (to try the page)."""
        if not self.core.fake:
            raise FS.FlaskSetupError("fake touches are only for the simulated robot")
        self._flask_set(**fit)
        p = self.core.geo.current_T_base_tcp(self.core.store.active_tcp)[:3, 3]
        rim_z = (85.0 ** 2 / 4 - 13.5 ** 2) ** 0.5 / 1000.0 + 0.05
        pts = FS.synthetic_touches(p - [0.0, 0.0, 0.05 + rim_z], tip_mm=self.flask_settings["tip_mm"] or 0.0,
                                   noise_mm=float(noise_mm))
        self.flask_points = pts
        return self._ok("fake touches on a 250 ml flask", flask=self.flask_view())

    # ------------------------------------------------------------- save and profiles
    def _flask_put(self, g: FlaskGeometry, profile: str, source: str) -> None:
        if not self.core.idle() or self.core.runner.state.name != "IDLE":
            raise FS.FlaskSetupError("the robot is busy: change the flask while idle")
        meta = {"flask_profile": profile, "source": source, "flask_geometry": g.to_dict()}
        store = self.core.store
        store.put_fixture(FixtureDef(FS.FLASK_FIXTURE, self.core.profile.base_link, FS.T_base_flask(g), meta),
                          replace=FS.FLASK_FIXTURE in store.fixtures)
        c = ", ".join(f"{1000 * v:.1f}" for v in g.sphere_center)
        self.core.bridge.log(f"fixture {FS.FLASK_FIXTURE} = flask profile {profile} ({g.standard_size}): "
                             f"bulb centre {c} mm in {self.core.profile.base_link}")

    def cmd_flask_save(self, name, tip_mm=None, wall_mm=None, neck_id_mm=None):
        """Fit, then save it as the flask profile `name` and make it the fixture "flask" programs use."""
        self._flask_set(tip_mm, wall_mm, neck_id_mm)
        g = self._flask_fit()
        if g is None:
            raise FS.FlaskSetupError("touch 3 rim points and 4 bulb points first")
        n = len(self.flask_points["neck"])
        name = self.flasks.save(name, g, f"touched {len(self.flask_points['rim'])} rim, "
                                         f"{len(self.flask_points['bulb'])} bulb" + (f", {n} neck" if n else ""))
        self._flask_put(g, name, f"flask profile {name}")
        return self._ok(f"saved flask {name} ({g.standard_size}) as fixture {FS.FLASK_FIXTURE}",
                        flask=self.flask_view())

    def cmd_flask_profiles(self):
        return self._ok(profiles=self.flasks.view(self.core.store.fixtures))
    cmd_flask_profiles.read_only = True

    def cmd_flask_profile_use(self, name, **fit):
        """Make a saved flask the fixture "flask" (like vial_profile_use: refused while a program runs)."""
        self._flask_set(**fit)
        g = self.flasks.get(name)
        self._flask_put(g, str(name), f"flask profile {name}")
        return self._ok(f"{FS.FLASK_FIXTURE} is now flask {name} ({g.standard_size})", flask=self.flask_view())

    def cmd_flask_profile_delete(self, name, **fit):
        self._flask_set(**fit)
        self.flasks.delete(name)
        return self._ok(f"flask profile {name} deleted (the fixture made from it stays)", flask=self.flask_view())

    # ----------------------------------------------------------------- scrape
    def _flask_saved(self) -> FlaskGeometry:
        fx = self.core.store.fixtures.get(FS.FLASK_FIXTURE)
        if fx is None or not isinstance(fx.meta.get("flask_geometry"), dict):
            raise FS.FlaskSetupError("save a flask first (SETUP › Flask Geometry)")
        return FlaskGeometry.from_dict(fx.meta["flask_geometry"])

    def _flask_plan(self, tool=None, tcp=None, theta_min_deg=0.0, theta_max_deg=15.0, rings=6, step_deg=15.0,
                    press_mm=0.5, margin_mm=1.0, pattern="down", pitch_top_deg=None, pitch_bottom_deg=None):
        g = self._flask_saved()
        tcp = str(tcp or self.core.store.active_tcp)
        if tcp not in self.core.store.tcps:
            raise FS.FlaskSetupError(f"unknown TCP '{tcp}'")
        t = FS.make_tool(tool, tcp)
        a = FS.scrape_args(theta_min_deg, theta_max_deg, rings, step_deg, press_mm, margin_mm, pattern,
                           pitch_top_deg, pitch_bottom_deg)
        T = self.core.geo.T_base_frame(FS.FLASK_FIXTURE)
        try:              # a bent tool keeps its roll as held now (a straight one: X faces the wall)
            ref = self.core.geo.current_pose(FS.FLASK_FIXTURE, tcp).T()[:3, :3]
        except Exception:                          # noqa: BLE001 - no TF yet: the flask's own axes
            ref = None
        try:
            plan = fs.generate_scrape_poses(g, t, a["theta_min_deg"], a["theta_max_deg"], a["rings"], a["step_deg"],
                                            press_depth_mm=a["press_depth_mm"], margin_m=a["margin_m"],
                                            reference_orientation=ref, pattern=a["pattern"], pitch_deg=a["pitch_deg"])
        except fs.FlaskScrapeError as e:
            raise FS.FlaskSetupError(str(e)) from None
        if a["pitch_deg"] is not None:          # a set pitch: the band is what this plan reaches all the way round
            return plan, _plan_band(plan), T
        # the bent tool's check is ~100x slower: a coarser band (1°, every 30° around) keeps it to a few seconds
        coarse = isinstance(t, fs.BentTool)
        band = fs.reachable_band(g, t, margin_m=a["margin_m"], resolution_deg=1.0 if coarse else 0.25,
                                 azimuth_step_deg=30.0 if coarse else 15.0)
        return plan, band, T

    def cmd_flask_reach(self, **kw):
        """Which of the poses the tool reaches through the neck, and the polar band reached all the way round."""
        plan, band, T = self._flask_plan(**kw)
        r = FS.reach_view(plan, band, T)
        b = (f"reached all the way round from {band[0]:.1f} to {band[1]:.1f}° from the bottom" if band
             else "no polar angle is reached all the way round")
        return self._ok(f"{r['reachable']} of {r['reachable'] + r['unreachable']} poses reachable; {b}", reach=r)

    def cmd_flask_suggest_bend(self, shaft_radius_mm=3.0, shaft_length_mm=250.0, theta_min_deg=0.0,
                               theta_max_deg=70.0, tip_radius_mm=0.0, margin_mm=1.0):
        """A bend (length, angle) that passes the neck of the saved flask, and the band it reaches."""
        g = self._flask_saved()
        try:
            sg = fs.suggest_bend(g, target_band_deg=(float(theta_min_deg), float(theta_max_deg)),
                                 shaft_radius=float(shaft_radius_mm) / 1000.0, shaft_length=float(shaft_length_mm) / 1000.0,
                                 tip_radius=float(tip_radius_mm or 0.0) / 1000.0, margin_m=float(margin_mm) / 1000.0)
        except (TypeError, ValueError) as e:
            raise FS.FlaskSetupError(str(e)) from None
        v = FS.suggestion_view(sg)
        return self._ok(f"bend {v['length_mm']:.1f} mm at {v['angle_deg']:.1f}°", suggestion=v)

    def cmd_program_flask_scrape(self, speed_mm_s=10.0, force_n=None, approach_mm=50.0, **kw):
        """A new, unsaved program that scrapes the flask's reachable poses (flask_scrape.build_program), linear
        moves at speed_mm_s. Never runs it."""
        self._editable()
        plan, band, T = self._flask_plan(**kw)
        try:
            speed = cart_speed.check_mm_s(float(speed_mm_s), cart_speed.max_mm_s(self.core.profile))
        except (TypeError, ValueError) as e:
            raise FS.FlaskSetupError(str(e)) from None
        modes = self.core.profile.modes
        motion = "position" if "position" in modes else self.core.profile.default_mode
        force = None if force_n in (None, "", 0) else float(force_n)
        force_mode = motion
        if force is not None:
            force_mode = next((n for n, m in modes.items() if m.wrench.get("topic") and m.supports_moveit), None)
            if force_mode is None:
                raise FS.FlaskSetupError("no control mode of this robot takes both a force and MoveIt moves")
            if force > float(self.core.force_cfg["max_force_n"]):
                raise FS.FlaskSetupError(f"force over the profile limit {self.core.force_cfg['max_force_n']:g} N")
        prog = fs.build_program(plan, "flask_scrape", self.core.profile.name, approach_m=float(approach_mm) / 1000.0,
                                force_n=force, motion_mode=motion, force_mode=force_mode)
        FS.add_speed(prog, speed)
        self.program, self.program_path = prog, None
        r = FS.reach_view(plan, band, T)
        return self._ok(f"generated {len(prog.steps)} steps for {r['reachable']} poses: check, then save",
                        reach=r, **self.program_view())
