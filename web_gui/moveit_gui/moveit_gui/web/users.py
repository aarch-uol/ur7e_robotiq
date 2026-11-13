"""Who has the web GUI open: one user per browser, however many tabs, even when several browsers come from the same
IP (one PC, or everyone behind `tailscale serve`, which connects from 127.0.0.1).

Each browser keeps a random id in its localStorage and sends it when it opens the live state stream. That id is never
shown to other browsers: they see a short public id derived from it, the name typed in that browser and where it
connects from (Tailscale user or IP). The control lease (service.py) records the public id of its holder, so the list
can say who has control."""
import hashlib
import re
import threading
from typing import Callable, Dict, List, Optional

GONE_S = 4.0          # a browser whose last tab closed stays listed this long (a reload must not flicker)
BID = re.compile(r"^[A-Za-z0-9_-]{16,64}$")


def public_id(bid: str) -> str:
    return hashlib.sha256(("moveit_gui-user:" + bid).encode()).hexdigest()[:10]


def clean_name(name) -> str:
    return " ".join(str(name or "").split())[:24]


class Users:
    def __init__(self, clock: Callable[[], float]):
        self.clock = clock
        self._lock = threading.Lock()
        self._b: Dict[str, dict] = {}            # browser id -> {id, name, where, tabs, since, gone}

    @staticmethod
    def valid(bid) -> bool:
        return isinstance(bid, str) and bool(BID.match(bid))

    def open(self, bid: str, name: str = "", where: str = "", key: str = "", role: str = "") -> Optional[str]:
        """A tab of this browser opened the state stream. Returns its public id (None: no usable id)."""
        if not self.valid(bid):
            return None
        now = self.clock()
        with self._lock:
            u = self._b.get(bid)
            if u is None or (u["gone"] and now - u["gone"] > GONE_S):
                u = self._b[bid] = {"id": public_id(bid), "name": "", "where": where, "tabs": 0, "since": now, "gone": 0.0,
                                    "key": key, "role": role}
            u["tabs"] += 1
            u["gone"] = 0.0
            u["where"] = where or u["where"]
            u["key"], u["role"] = key or u["key"], role or u["role"]
            if clean_name(name):
                u["name"] = clean_name(name)
            return u["id"]

    def close(self, bid: str):
        with self._lock:
            u = self._b.get(bid)
            if u is None:
                return
            u["tabs"] = max(0, u["tabs"] - 1)
            if not u["tabs"]:
                u["gone"] = self.clock()

    def rename(self, bid: str, name: str) -> bool:
        with self._lock:
            u = self._b.get(bid)
            if u is None:
                return False
            u["name"] = clean_name(name)
            return True

    def label(self, bid: str) -> str:
        """The name others see: the typed name, else "Browser" + a few letters of its public id."""
        with self._lock:
            u = self._b.get(bid)
        if u is None:
            return f"Browser {public_id(bid)[:4]}" if self.valid(bid) else ""
        return u["name"] or f"Browser {u['id'][:4]}"

    def view(self, holder: Optional[str] = None) -> List[dict]:
        """Everyone with the page open: control first, then in the order they came. Same names get (2), (3)."""
        now = self.clock()
        with self._lock:
            for bid in [b for b, u in self._b.items() if u["gone"] and now - u["gone"] > GONE_S]:
                del self._b[bid]
            us = sorted(self._b.values(), key=lambda u: (u["id"] != holder, u["since"]))
            out, seen = [], {}
            for u in us:
                if u["gone"]:
                    continue
                name = u["name"] or f"Browser {u['id'][:4]}"
                seen[name] = seen.get(name, 0) + 1
                out.append({"id": u["id"], "name": name if seen[name] == 1 else f"{name} ({seen[name]})",
                            "where": u["where"], "tabs": u["tabs"], "control": u["id"] == holder,
                            "for_s": round(now - u["since"]), "key": u["key"], "role": u["role"]})
            return out
