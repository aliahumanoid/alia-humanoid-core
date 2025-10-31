"""
Lightweight sanity check for the Flask/SocketIO host app.
- Imports the app and registers routes
- Prints URL map and exits with code 0 on success
"""

import sys

try:
    from main import app, init_app
except Exception as exc:
    print(f"[sanity] import failure: {exc}")
    sys.exit(2)


def main() -> int:
    try:
        init_app()
        # Show a couple of endpoints to confirm routing is live
        urls = sorted({str(r) for r in app.url_map.iter_rules()})
        print("[sanity] URL map (sample):")
        for r in urls[:10]:
            print("  ", r)
        print("[sanity] OK")
        return 0
    except Exception as exc:
        print(f"[sanity] runtime failure: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

