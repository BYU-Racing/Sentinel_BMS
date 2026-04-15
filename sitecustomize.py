"""Repo-local Python startup shim for broken PlatformIO penv installs.

This project is sometimes opened with a PlatformIO virtualenv missing the
top-level ``requests`` package even though pip's vendored copy is present.
Python imports ``sitecustomize`` automatically on startup when it exists on
``sys.path``; because the project root is on ``sys.path`` for local commands,
we can map the missing imports before PlatformIO initializes.
"""

from __future__ import annotations

import importlib
import sys


def _alias_package(name: str, vendor_name: str) -> None:
    if name in sys.modules:
        return
    try:
        importlib.import_module(name)
        return
    except ModuleNotFoundError:
        pass

    sys.modules[name] = importlib.import_module(vendor_name)


_alias_package("requests", "pip._vendor.requests")
_alias_package("certifi", "pip._vendor.certifi")
