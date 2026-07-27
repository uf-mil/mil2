"""Static lower bound on a behaviour tree's declared sim-time budget.

Walks the mission XML summing every declared timeout, so the harness budget in
tasks.py can be checked against what the tree actually asks for. Rules:

  * A wrapping timeout node (Timeout / RosTimeout) contributes its own cap and
    its children are NOT also counted -- the cap already bounds them.
  * Any other `*_msec` attribute (timeout_msec, move_timeout_msec, delay_msec,
    grip_settle_msec, ...) is additive: it is time the node can consume.
  * RetryUntilSuccessful(num_attempts) and Repeat(num_cycles) multiply their
    subtree's sum when the count is a literal.
  * <SubTree ID="X" a="1" b="{ref}"> recurses into X with the caller's literals
    bound to X's ports. A {ref} that cannot be resolved contributes 0 and is
    recorded in `unresolved`.

Because unresolved values contribute 0, the result is a LOWER bound -- which is
exactly what a "your budget must be at least this" guard needs.

Known blind spots, all of which make the number a floor rather than a ceiling
except where noted:

  * `<Parallel>` children are SUMMED, not maxed. That over-counts a tree whose
    branches run concurrently, so it is safe for a `budget >= bound` guard but
    it does contradict the lower-bound contract -- do not read a Parallel-
    containing bound as "at least this long".
  * Only `msec` and `*_msec` attributes count. A node that spells its duration
    some other way (`milliseconds=`, `momentum_ms=`) or that holds a duration
    only in its C++ default contributes 0. `<Action ID="YawStyle">` in
    orient_to_octagon_image.xml is reachable from OctagonMission and is
    invisible for exactly this reason.
  * An infinite loop (`num_cycles="-1"`, `KeepRunningUntilFailure`) counts as
    one pass. Such a tree is not bounded by its declared timeouts at all, so it
    is flagged in `unresolved` -- see `Bound.complete`.
"""

from __future__ import annotations

import glob
import os
import re
from dataclasses import dataclass, field
from xml.etree import ElementTree

WRAPPING_TIMEOUTS = {"Timeout", "RosTimeout"}
MULTIPLIERS = {
    "RetryUntilSuccessful": "num_attempts",
    "Repeat": "num_cycles",
}
INFINITE_LOOPS = {"KeepRunningUntilFailure"}  # loops until its child fails
PORT_REF = re.compile(r"^\{([^{}]+)\}$")


@dataclass
class Bound:
    msec: float = 0.0
    unresolved: list[str] = field(default_factory=list)

    def __post_init__(self) -> None:
        # The same port is often unresolvable at several call sites; report each
        # distinct cause once, in the order first hit.
        self.unresolved = list(dict.fromkeys(self.unresolved))

    @property
    def seconds(self) -> float:
        return round(self.msec / 1000.0, 3)

    @property
    def complete(self) -> bool:
        """True when every declared duration in the tree actually resolved.

        A caller DERIVING a budget must check this: a subtree walked on its own
        takes its timeouts from caller-supplied ports, so `ApproachAndGrasp`
        alone is 0.0 s and complete=False, not a 0-second tree. A guard merely
        asserting `budget >= seconds` must NOT check it -- an incomplete bound
        is still a valid floor, and requiring completeness would fail on trees
        whose ports are legitimately set at runtime by <Script>.
        """
        return not self.unresolved


def _parse_all(xml_dir: str):
    """Yield (path, root, error) for every XML file, parsed or not."""
    for path in sorted(glob.glob(os.path.join(xml_dir, "*.xml"))):
        try:
            yield path, ElementTree.parse(path).getroot(), None
        except ElementTree.ParseError as exc:
            yield path, None, exc


def unparsable(xml_dir: str) -> dict[str, str]:
    """Mission XML that BT.CPP's tinyxml2 accepts but ElementTree rejects.

    Their trees are invisible to declared_bound(), so a stage that came to
    depend on one would silently under-report its budget.
    """
    return {os.path.basename(p): str(e) for p, _, e in _parse_all(xml_dir) if e}


def load_trees(xml_dir: str) -> dict[str, ElementTree.Element]:
    """Map every BehaviorTree ID in a directory of mission XML to its element."""
    trees: dict[str, ElementTree.Element] = {}
    for _path, root, error in _parse_all(xml_dir):
        if error:
            continue  # reported by unparsable(); pinned by the canary test
        for bt in root.iter("BehaviorTree"):
            tree_id = bt.get("ID")
            if tree_id:
                trees[tree_id] = bt
    return trees


def _resolve(
    value: str | None,
    env: dict,
    unresolved: list[str],
    what: str,
) -> float | None:
    """Resolve a literal or a {port} reference to a float, or None."""
    if value is None:
        return None
    ref = PORT_REF.match(value.strip())
    if ref:
        name = ref.group(1)
        resolved = env.get(name)
        if resolved is None:
            unresolved.append(f"{what}={{{name}}}")
            return None
        value = str(resolved)
    try:
        return float(value)
    except ValueError:
        unresolved.append(f"{what}={value}")
        return None


def _child_env(elem: ElementTree.Element, env: dict) -> dict:
    """Ports passed at a SubTree call site become the child's environment."""
    child = dict(env) if elem.get("_autoremap") == "true" else {}
    for key, value in elem.attrib.items():
        if key in ("ID", "_autoremap", "name"):
            continue
        ref = PORT_REF.match(value.strip())
        # Deliberately unguarded: in BT.CPP an explicit remap WINS over
        # _autoremap, so an unresolvable {ref} must shadow the autoremapped
        # value of the same name. Resolving it instead would invent a number
        # the tree never reads.
        child[key] = env.get(ref.group(1)) if ref else value
    return child


def _walk(
    elem: ElementTree.Element,
    trees: dict[str, ElementTree.Element],
    env: dict,
    unresolved: list[str],
    seen: tuple[str, ...],
) -> float:
    tag = elem.tag

    if tag in WRAPPING_TIMEOUTS:
        cap = _resolve(elem.get("msec"), env, unresolved, f"{tag}.msec")
        # The cap bounds everything below it: do not descend.
        return cap or 0.0

    if tag == "SubTree":
        tree_id = elem.get("ID")
        if tree_id in seen:  # recursive call site; one pass is the lower bound
            return 0.0
        if not tree_id or tree_id not in trees:
            # Same silent-zero hazard as an unresolvable port: a subtree whose
            # XML is missing (or failed to parse) drops its whole budget.
            unresolved.append(f"SubTree.ID={tree_id}")
            return 0.0
        child_env = _child_env(elem, env)
        return _walk(trees[tree_id], trees, child_env, unresolved, (*seen, tree_id))

    total = 0.0
    for key, value in elem.attrib.items():
        if key.endswith("_msec") or key == "msec":
            total += _resolve(value, env, unresolved, f"{tag}.{key}") or 0.0

    children = sum(_walk(c, trees, env, unresolved, seen) for c in elem)

    if tag in MULTIPLIERS:
        port = MULTIPLIERS[tag]
        count = _resolve(elem.get(port), env, unresolved, f"{tag}.count")
        if count is not None and count < 0:
            # BT.CPP's "loop forever". One pass is the honest lower bound, but a
            # tree containing one is not bounded by its declared timeouts at all.
            unresolved.append(f"{tag}.{port}=forever")
        children *= 1.0 if count is None or count < 0 else count
    elif tag in INFINITE_LOOPS:
        unresolved.append(f"{tag}=forever")

    return total + children


def declared_bound(trees: dict[str, ElementTree.Element], tree_id: str) -> Bound:
    """Lower bound, in declared time, on how long `tree_id` can run."""
    if tree_id not in trees:
        raise KeyError(f"no BehaviorTree with ID '{tree_id}'")
    unresolved: list[str] = []
    msec = _walk(trees[tree_id], trees, {}, unresolved, (tree_id,))
    return Bound(msec=msec, unresolved=unresolved)
