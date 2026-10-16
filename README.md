# ApexSim — Nordschleife GT3

A sim-racing game in modern C++20 / OpenGL 4.5, in the spirit of iRacing: one car (a GT3), one track
(the 20.8 km Nürburgring Nordschleife), and a focus on realistic handling, graphics and sound.

![Fuchsröhre](docs/fuchsroehre.png)

| | |
|---|---|
| ![menu](docs/menu.png) | ![cockpit](docs/cockpit.png) |
| ![car](docs/car_side.png) | ![Karussell](docs/karussell.png) |

*Screenshots are from the software renderer used in CI-style testing (llvmpipe, 2× MSAA, 720p). On a
real GPU, use 4–8× MSAA, Ultra shadows and vegetation, and higher resolutions.*

## Features

**Track: the real Nordschleife.** The track is built from data, not drawn by hand.
- The centreline and all 42 named sections (Hatzenbach, Flugplatz, Fuchsröhre, Bergwerk, Karussell,
  Pflanzgarten, Döttinger Höhe, …) come from OpenStreetMap. Length is 20.77 km.
- Elevation comes from DEM terrain tiles: 300 m of climb from Breidscheid to Hohe Acht, including the
  Flugplatz crest and the Fuchsröhre compression.
- The real OSM forest/field/village map decides where ~100k spruce and beech trees grow.
- Road features: kerbs at every corner, the banked Karussell, double armco, grass verges, white lines,
  a rubbered-in racing line and the start/finish line.
- `tools/build_track.py` regenerates everything from source data.

**Physics** (fixed 1 kHz step, everything in double precision)
- 6-DOF chassis with four unsprung masses, springs, slow/fast bump & rebound dampers, bump stops,
  anti-roll bars, roll centres, camber and toe.
- Combined-slip Magic-Formula tyres with load sensitivity, a camber window, relaxation lengths
  (stable down to standstill) and self-aligning torque.
- 5.4 l V8 torque curve, engine inertia and engine braking. Clutch and diff are solved as friction
  constraints. Sequential 6-speed with ignition cut, auto-blip and money-shift protection.
  Clutch-pack LSD (preload + power/coast ramps).
- Aero: CdA 1.25, ClA 3.0 with balance; ~600 kg of downforce at 200 km/h.
- ABS, 6-level traction control and adjustable brake bias. Brake-disc temperature makes the discs glow.
- Validated by `apexsim_tests`: 0–100 km/h in 3.6 s, 0–200 in 9.2 s, 271 km/h top speed, 200–0 in
  ~80 m, ~1.7 g mid-speed grip, and a clean AI lap of the Nordschleife in ~7:44 (real GT3: ~7:00).

**Graphics** (forward HDR renderer)
- PBR (GGX / Smith / Schlick), with a clear-coat lobe for car paint and carbon.
- Physically based atmosphere sky (Rayleigh/Mie) with a cumulus layer. Image-based lighting from the
  sky: prefiltered specular plus SH irradiance. Sun position for the real latitude and any time of day.
- Stabilised cascaded shadow maps (up to 4×4096²), height fog / aerial perspective, MSAA with
  tonemap-aware resolve, alpha-to-coverage foliage, bloom, ACES filmic tonemapping.
- Reversed-Z infinite projection and camera-relative rendering, so nothing jitters or z-fights
  anywhere on the 20 km circuit.
- CC0 photogrammetry PBR textures from Poly Haven (tarmac, grass, forest floor, gravel, bark).
- GPU heightmap terrain with LODs, a distant forest canopy shell and a 24 km horizon.
- Procedural GT3 coupé: wing, splitter, diffuser, livery with race number, glowing brakes, shift
  lights, a cockpit with a moving steering wheel and roll cage.

**Sound** (procedural, nothing sampled)
- Cross-plane V8: individual exhaust pulses per cylinder, using the real firing order, so each bank
  gets uneven pulses — the "burble" — feeding exhaust-pipe waveguides. Intake roar, mechanical noise,
  overrun crackle, limiter stutter and shift pops.
- Straight-cut gear whine and sequential clunks.
- Tyre squeal and scrub driven by the slip model, lock-ups, kerb rumble at the true ridge frequency,
  gravel/grass, road and wind noise, impacts and floor scrapes.
- Separate cockpit and exterior mixes, with a forest reverb.

**Game**
- Main menu with a live AI car lapping behind it.
- Practice sessions: lap timing, live delta to your best lap, four-wheels-off lap invalidation.
- HUD with rev lights, gear and speed, pedal and steering inputs, track map and the current corner name.
- Settings, saved to `settings.ini`:
  - graphics (resolution, window mode, MSAA, render scale, shadows, vegetation, FOV…)
  - audio mix
  - controls: rebind any key, button or axis; calibrate pedals; set wheel rotation for 1:1 steering
  - assists
- Cameras: cockpit (with g-force head movement), hood, chase, far chase and TV.

## Building (Windows, for your RTX 5070 Ti)

**Easiest:** open the repository's **Actions** tab on GitHub, pick the latest run, and download the
`ApexSim-windows-x64` artifact. Unzip it and run `apexsim.exe`.

**From source:**
1. Install Visual Studio 2022 (with "Desktop development with C++") and CMake 3.21+ (VS includes one).
2. Run:
   ```
   git clone https://github.com/NoahMohsin44/iracing-clone.git
   cd iracing-clone
   cmake -S . -B build -A x64
   cmake --build build --config Release --parallel
   build\Release\apexsim.exe
   ```
   Dependencies (GLFW, GLM, Dear ImGui, miniaudio) are downloaded automatically by CMake on the first
   configure. Or open the folder in Visual Studio ("Open Folder") and pick the `apexsim.exe` target.

**Linux:** `sudo apt install libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libgl-dev`, then
`cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release && cmake --build build`.

**Physics tests:** `build/Release/apexsim_tests` (add `quick` to skip the full lap).
**Audio preview:** `apexsim_tests audio out.wav 1` renders a scripted run of the car audio to a WAV
file (`1` = cockpit mix, `0` = exterior).

## Controls (defaults, all rebindable)

| Action | Keyboard | Gamepad |
|---|---|---|
| Steer | ← → | left stick |
| Throttle / brake | ↑ / ↓ | RT / LT |
| Shift up / down | E / Q | A or RB / X or LB |
| Clutch | Z | – |
| Camera | C | Y |
| Look left / right / back | , / . / B | d-pad / R3 |
| Pause | Esc | Start |
| Reset car to track | R | Back |
| ABS / TC level | F7 / F8 | – |
| Brake bias + / − | ] / [ | d-pad up / down |
| HUD | H | – |
| Screenshot | F12 | – |

**Racing wheels** (Logitech, Thrustmaster, Fanatec, …):
1. Open *Settings → Controls*.
2. Click the *Wheel / controller* slot for **Steering**, turn the wheel fully right, then let go.
3. For each pedal, press it fully and release.
4. Set *Wheel rotation* to your wheel's range so steering is 1:1 with the car's 540° wheel.

Force feedback isn't implemented yet (see the roadmap).

**Field of view:** a realistic vertical FOV for one monitor is 35–45°. Change it in *Settings → Graphics*.

## Project layout

```
src/core      settings, input, logging, paths, JSON, noise
src/world     track data and queries, track/terrain meshes, vegetation
src/car       tyre model, drivetrain, chassis physics, GT3 parameters, procedural car model
src/audio     DSP blocks, engine synth, car audio mix, audio device
src/gfx       GL wrappers, sky/IBL, shadows, post-processing, renderer
src/game      app loop, session, cameras, AI driver, lap timer
src/ui        theme, main menu, settings, HUD
assets/       shaders, textures, fonts, track data
tools/        build_track.py (OSM + DEM -> track data), fetch_textures.py
tests/        headless physics tests + audio preview renderer
```

## Data sources & licences

- Track layout, corner names and land cover: © OpenStreetMap contributors, ODbL 1.0.
- Elevation: AWS Terrain Tiles (Mapzen), derived from SRTM, EU-DEM and GMTED.
- Textures: Poly Haven, CC0.
- Font: Titillium Web, SIL OFL.
- Libraries: GLFW, Dear ImGui, GLM, miniaudio, stb, glad.

## Roadmap ideas

- Force feedback (DirectInput) driven by the tyres' self-aligning torque, which is already computed.
- Tyre temperatures and wear, fuel load, car setup screen.
- Replays, ghost car, AI opponents, the GP circuit / 24h combined layout.
- Hero car model from Blender (glTF import), recorded engine samples as an option, rain.
