# cppautogui

`cppautogui` is a small native macOS replacement for the local `pyautogui`
test helper. It records mouse/key actions, captures app screenshots, replays a
recording, and compares BMP screenshots without Python, pyautogui, pynput, or
Pillow.

The implementation intentionally keeps the same text format as
`../pyautogui`:

- `m x y` mouse move
- `p x y Button.left` mouse press
- `r x y Button.left` mouse release
- `k Key.esc` key press
- `screenshot n x y width height mouse_x mouse_y`
- `quit x x`

Old recordings from `../pyautogui/datas` can therefore be replayed by this
tool.

## macOS permissions

Open `System Settings > Privacy & Security` and allow the Terminal or IDE that
runs `cppautogui` for:

- Accessibility
- Input Monitoring
- Screen Recording
- Developer Tools

Restart the terminal after changing permissions.

## Build

From `smala/cppautogui`:

```sh
make
```

Executables are created in `bin/`:

- `bin/record_cppautogui`
- `bin/replay_cppautogui`
- `bin/launch_tests_cppautogui`

No third-party dependency is required. The Makefile links only macOS system
frameworks.

## Directory layout

Run commands from `smala/cppautogui`, like the Python helper was run from
`smala/pyautogui`.

- `datas/`: stable replay files, for example `simplest_data.txt`
- `references/`: stable reference screenshots, for example
  `simplest_1_Ref.bmp`
- `results/`: screenshots produced by replay
- `results/diffs/`: diff images when comparison fails
- `new_records/`: temporary recordings and reference screenshots

To reuse the existing Python baselines:

```sh
cp ../pyautogui/datas/* datas/
cp ../pyautogui/references/* references/
```

## Record

```sh
bin/record_cppautogui -i simplest
bin/record_cppautogui -i simplest --debug
bin/record_cppautogui -i simplest --retina
```

The tool launches `make -j simplest_test` from `smala/`, then records global
input events.

- Press `CTRL+ALT` to capture the current `<test>_app` window.
- Press `ESC` to stop recording.

Files are written in `new_records/`. Once a recording is good, copy:

- `new_records/<test>_data.txt` to `datas/`
- `new_records/<test>_*_Ref.bmp` to `references/`

## Replay one test

```sh
bin/replay_cppautogui -i simplest
bin/replay_cppautogui -i simplest --test
bin/replay_cppautogui -i simplest --retina
bin/replay_cppautogui -i simplest --no_interpolation
```

Default mode reads `datas/<test>_data.txt` and compares against
`references/<test>_*_Ref.bmp`.

`--test` reads both data and references from `new_records/`, useful just after
recording.

If `datas/<test>_data_nointer.txt` exists, it is used automatically and mouse
interpolation is disabled.

Useful tuning flags:

```sh
bin/replay_cppautogui -i simplest --wait 3
bin/replay_cppautogui -i simplest --ratio 0.5
```

- `--wait`: seconds to wait after launching the test app.
- `--ratio`: accepted image difference in percent. Default is `1.0`.

## Replay all tests

```sh
bin/launch_tests_cppautogui
bin/launch_tests_cppautogui --retina
```

The launcher scans `datas/*_data*.txt`, replays every test, and returns a
non-zero status if at least one comparison fails.

## Notes

- The app window name is expected to be `<test>_app`, as in the Python helper.
- The replay command launches `make -j <test>_test` from the parent directory.
- Only macOS is implemented for now. Linux/Windows support would need separate
  backends for input injection, global input recording, screen capture, and
  window discovery.
