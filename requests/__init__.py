"""Compatibility shim for a broken PlatformIO virtualenv.

The local PlatformIO Python environment is missing the top-level ``requests``
package, but pip still ships a vendored copy. Expose that copy under the
expected import name so ``platformio`` can start.
"""

from __future__ import annotations

from importlib import import_module

_vendor = import_module("pip._vendor.requests")

globals().update(_vendor.__dict__)
__file__ = _vendor.__file__
__path__ = _vendor.__path__
