"""The README's example of the bindings, run as written.

The README shows the module serving the synthetic pattern and fetching one
half, and says what that prints. This runs the example from the README itself,
so a change to the module that breaks it, or to the example, fails here.
"""

import contextlib
import io
import pathlib
import re

REPO = pathlib.Path(__file__).resolve().parents[3]
README = REPO / "README.md"
SECTION = "## Python bindings"


def bindings_section():
    """The README's bindings section: from its heading to the next one."""
    text = README.read_text(encoding="utf-8")
    start = text.index(SECTION)
    end = text.find("\n## ", start + len(SECTION))
    return text[start:] if end < 0 else text[start:end]


def test_readme_example_prints_what_it_says():
    section = bindings_section()
    examples = re.findall(r"```python\n(.*?)```", section, re.DOTALL)
    assert len(examples) == 1, "the bindings section has one Python example"
    promised = re.search(r"That prints `([^`]*)`", section)
    assert promised, "the README says what the example prints"

    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        exec(compile(examples[0], str(README), "exec"), {"__name__": "readme_example"})
    assert out.getvalue() == promised.group(1) + "\n"
