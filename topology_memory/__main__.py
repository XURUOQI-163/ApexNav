"""Offline inspection: python -m topology_memory [--node ID] [--export]."""
import argparse
import json
from pathlib import Path

from .store import UnifiedTopologyMemory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', default=str(Path(__file__).resolve().parents[1] / 'data/topology'))
    parser.add_argument('--node', help='List one node\'s original RGB observations')
    parser.add_argument('--export', action='store_true', help='Rebuild the derived combined snapshot (stop recorder first)')
    args = parser.parse_args()
    memory = UnifiedTopologyMemory(args.directory)
    issues = memory.load_memory()
    if args.node:
        result = memory.get_node_observations(args.node)
    else:
        graph = memory.get_topology_graph(include_inactive=True)
        result = dict(directory=str(memory.root), map_key=memory.manifest.get('map_key'),
                      session_id=memory.manifest.get('session_id'), issues=issues,
                      nodes=len(graph['nodes']), edges=len(graph['edges']),
                      active_frontiers=len(memory.get_active_frontiers()),
                      node_ids=[n['node_id'] for n in graph['nodes']])
    if args.export:
        # Avoid racing the live writer from an inspector terminal.
        import fcntl
        memory.root.mkdir(parents=True, exist_ok=True)
        with (memory.root / '.writer.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            memory.export_unified_graph()
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
