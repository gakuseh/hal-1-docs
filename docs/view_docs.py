#!/usr/bin/env python3
"""Open the HAL-1 Doxygen docs in a browser.

Serves docs/doxygen/html on a local port and opens it. Needs only Python 3
(standard library), so it works on Linux, macOS and Windows:

    python3 docs/view_docs.py            # Linux / macOS
    py docs\\view_docs.py                 # Windows

Options:
    --build        Regenerate the docs with Doxygen first (needs doxygen and
                   Graphviz installed).
    --port N       Serve on port N instead of picking a free one.
    --no-browser   Don't open a browser; just print the URL.

Stop the server with Ctrl+C.
"""
import argparse
import functools
import http.server
import shutil
import subprocess
import sys
import webbrowser
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
HTML = REPO / "docs" / "doxygen" / "html"


def build():
    doxygen = shutil.which("doxygen")
    if not doxygen:
        sys.exit("doxygen is not installed. Install Doxygen and Graphviz, "
                 "or run without --build to view the committed docs.")
    print("Running doxygen...")
    subprocess.run([doxygen], cwd=REPO, check=True)
    log = REPO / "docs" / "doxygen-warnings.log"
    if log.exists() and log.stat().st_size:
        print(f"Doxygen reported warnings; see {log}")


def main():
    p = argparse.ArgumentParser(description="Serve the HAL-1 Doxygen docs locally.")
    p.add_argument("--build", action="store_true", help="regenerate the docs with doxygen first")
    p.add_argument("--port", type=int, default=0, help="port to serve on (default: any free port)")
    p.add_argument("--no-browser", action="store_true", help="don't open a browser")
    args = p.parse_args()

    if args.build:
        build()
    if not (HTML / "index.html").exists():
        sys.exit(f"{HTML / 'index.html'} not found. Run with --build to generate it.")

    handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=str(HTML))
    handler.log_message = lambda *a: None  # keep the terminal quiet
    with http.server.ThreadingHTTPServer(("127.0.0.1", args.port), handler) as server:
        url = f"http://127.0.0.1:{server.server_address[1]}/index.html"
        print(f"Serving the docs at {url}  (Ctrl+C to stop)", flush=True)
        if not args.no_browser:
            webbrowser.open(url)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
