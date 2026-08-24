"""Select either unattended capture or the browser bench/viewer application."""

import os


mode = os.environ.get("SCANNER_MODE", "bench")

if mode == "deployed":
    from app.main import main

    main()
elif mode in {"bench", "viewer"}:
    from app.web import main

    main(mode)
else:
    raise SystemExit("SCANNER_MODE must be bench, viewer, or deployed")
