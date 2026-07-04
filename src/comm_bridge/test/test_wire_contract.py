"""Wire-contract guard: comm_bridge relay table vs shared interface_manifest.yaml.

The manifest (SSOT, shipped inside the g1_onboard_msgs submodule) is the canonical
PC<->NX wire contract. This test asserts that every /bridge/* topic comm_bridge
relays carries the message type the manifest requires. It catches topic<->type
drift such as C-1 (PC publishes JointCmdChunk on /bridge/cmd/arm while comm_bridge
still relays JointCmd).

Paths can be overridden for local runs:
  WIRE_MANIFEST -> interface_manifest.yaml
  WIRE_RELAY    -> comm_bridge_params.yaml
"""
import os
from pathlib import Path

import pytest
import yaml

_HERE = Path(__file__).resolve()
_REPO = _HERE.parents[3]  # src/comm_bridge/test/<file> -> repo root

MANIFEST = Path(os.environ.get(
    "WIRE_MANIFEST", _REPO / "src" / "g1_onboard_msgs" / "interface_manifest.yaml"))
RELAY = Path(os.environ.get(
    "WIRE_RELAY", _REPO / "src" / "comm_bridge" / "config" / "comm_bridge_params.yaml"))


def _manifest_by_topic():
    data = yaml.safe_load(MANIFEST.read_text())
    return {i["topic"]: i for i in data["interfaces"]}


def _bridge_relays():
    """(bridge_topic, relayed_type) for every comm_bridge relay entry."""
    data = yaml.safe_load(RELAY.read_text())
    entries = []
    for section in ("inbound_relay", "outbound_relay"):
        for r in data[section]["ros__parameters"]["relays"]:
            # the /bridge side is `src` for inbound, `dst` for outbound
            topic = r["src"] if r["src"].startswith("/bridge/") else r["dst"]
            entries.append((topic, r["type"]))
    return entries


@pytest.mark.parametrize("topic,relay_type", _bridge_relays())
def test_relay_type_matches_manifest(topic, relay_type):
    manifest = _manifest_by_topic()
    assert topic in manifest, (
        f"{topic} is relayed by comm_bridge but is absent from interface_manifest.yaml")
    expected = manifest[topic]["type"]
    assert relay_type == expected, (
        f"WIRE CONTRACT DRIFT on {topic}: comm_bridge relays '{relay_type}' "
        f"but manifest (SSOT) requires '{expected}'")
