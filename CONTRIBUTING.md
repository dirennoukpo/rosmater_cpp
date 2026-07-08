# Contributing & Developer Guide

This guide is for developers working *on* the Rosmaster C++ driver (as opposed to
*using* it — for that, read [README.md](README.md)). It covers the build, the
compile/verification checks, the documentation toolchain, and the house style.

---

## 1. Repository layout

| Path                 | Purpose                                                             |
| -------------------- | ------------------------------------------------------------------- |
| `Rosmaster.hpp`      | The entire driver — single header, header-only, fully `inline`.     |
| `README.md`          | User-facing guide (API, protocol, PID theory, ros2_control, tuning).|
| `CONTRIBUTING.md`    | This file — developer/build/docs guide.                             |
| `Doxyfile`           | Doxygen configuration for the HTML API reference.                   |
| `scripts/build_docs.sh` | Convenience wrapper around `doxygen`.                            |
| `LICENSE`            | License.                                                            |

There is deliberately **no build system** (no CMake/Makefile): the header is
compiled directly by whatever consumes it. See below for a standalone compile check.

---

## 2. Building & compile-checking

The driver needs **C++17**, POSIX, and `-pthread` (it starts a receive thread and
a PID thread).

### Fast syntax check (no hardware needed)

```bash
printf '#include "Rosmaster.hpp"\nint main(){ return 0; }\n' > /tmp/check.cpp
g++ -std=c++17 -fsyntax-only -Wall -Wextra -I. /tmp/check.cpp && echo OK
```

Run this after **every** change to `Rosmaster.hpp`. It must compile clean with no
warnings.

### Building a program against the header

```bash
g++ -std=c++17 -O2 -Wall -Wextra -pthread your_program.cpp -o your_program
```

A ready-to-run test bench (calibrate → enable PID → convergence log) is provided
inline in [README.md §13](README.md#13-test-bench-standalone-example) — copy it into
`main.cpp` and build it with the command above. Running it needs the real hardware on
`/dev/ttyUSB0` with the wheels free to spin.

---

## 3. Documentation toolchain (Doxygen)

The public/private API is documented **in-source** with Doxygen `/** … */` comment
blocks (JavaDoc style, `@brief`/`@param`/`@return`/`@throws`/`@note`/`@warning`).

### Generate and view the HTML reference

Install the tools once (`doxygen` required, `graphviz` optional for the graphs):

| Platform            | Command                                          |
| ------------------- | ------------------------------------------------ |
| Debian / Ubuntu / WSL | `sudo apt-get install doxygen graphviz`        |
| Fedora / RHEL       | `sudo dnf install doxygen graphviz`              |
| Arch                | `sudo pacman -S doxygen graphviz`                |
| macOS (Homebrew)    | `brew install doxygen graphviz`                  |

```bash
./scripts/build_docs.sh --serve   # generate + serve on http://localhost:8000 + open browser
./scripts/build_docs.sh --open    # generate + open the static file
./scripts/build_docs.sh           # generate only
doxygen Doxyfile                  # raw equivalent (no serving)
```

Full viewing instructions (including WSL 2 specifics) are in
[README.md → “Browsing the API reference”](README.md#browsing-the-api-reference-doxygen).

Output lands in `docs/html/index.html` (git-ignored). The landing page is
`docs/mainpage.md` (`USE_MDFILE_AS_MAINPAGE`); the repo's README/CONTRIBUTING are the
GitHub-facing guides and are intentionally **not** fed to Doxygen (their GitHub-style
anchor TOCs would emit spurious `\ref` warnings). `graphviz` produces the
class/collaboration/call graphs. Key `Doxyfile` settings: `EXTRACT_ALL`,
`EXTRACT_PRIVATE`, `EXTRACT_STATIC` (the private members and the PID internals are
part of the reference), `SOURCE_BROWSER` (clickable annotated source), and
`PREDEFINED = __linux__ __unix__` (so the POSIX serial path is the one documented).

### Documentation conventions

- **Document the declaration.** Put the full `/** … */` API block on the *declaration*
  in the class body. Implementations carry concise "how/why" comments only — do not
  duplicate the `@param`/`@return` list on the definition.
- **Explain units and thread-safety.** Motor speeds/targets are percent in `[-100, 100]`;
  `scale` is ticks/s at 100% command. State which thread calls a method and which
  atomic/mutex protects each shared field.
- **Prefer "why" over "what".** The non-obvious invariants (wrap-safe encoder deltas,
  the timestamp-based `window_dt`, the sign-aware anti-windup, the FIX-1…7 serial
  steps) are the ones worth a comment.
- **Keep the history.** The `SERIAL PORT LIFECYCLE` and `SOFTWARE PID — CHANGELOG`
  blocks at the top of `Rosmaster.hpp` are institutional memory — extend them, don't
  delete them.

### The documentation invariant

Documentation changes must be **comments-only**: adding or editing comments must never
change a single token of code. To prove it after a doc pass:

```bash
# Only additions, zero deletions/modifications of code lines:
git diff -- Rosmaster.hpp | grep '^-' | grep -v '^---'    # should be empty
# And it must still compile clean:
g++ -std=c++17 -fsyntax-only -Wall -Wextra -I. /tmp/check.cpp && echo OK
```

---

## 4. House style

- **C++17**, POSIX, no third-party dependencies (no Boost, no libserial). Keep it that
  way — the header must stay drop-in.
- Everything is `inline` (header-only). New free functions and class methods included.
- **Concurrency:** cross-thread scalars are `std::atomic`; anything larger is guarded by
  the existing mutex for its domain (`pid_gains_mutex_`, `ff_mutex_`, `arm_mutex_`).
  Take a snapshot under the lock in `pidLoop()`; never hold a lock across I/O.
- **Serial writes** go through `writeCmd()` (appends checksum, honours `delay_time_`).
  Frame length byte is `size − 1` (checksum excluded); the checksum is
  `(COMPLEMENT + Σbytes) & 0xFF`.
- **Encoder math** is always the wrap-safe form
  `int32_t(uint32_t(now) − uint32_t(old))` — never a plain `int` subtraction.
- **Tuning constants** (`kPidHz`, `kVelWindow`, `kDerivAlpha`) are coupled; changing one
  means recomputing the others (see [README §15](README.md#15-tuning-the-pid-gains)).

---

## 5. Commit checklist

- [ ] `Rosmaster.hpp` compiles clean (`-fsyntax-only -Wall -Wextra`).
- [ ] Any behaviour change is reflected in `README.md` and the in-source docs.
- [ ] Doc-only commits satisfy the comments-only invariant above.
- [ ] The top-of-file CHANGELOG / FIX history is updated for behavioural changes.
