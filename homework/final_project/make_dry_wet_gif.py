"""
make_dry_wet_gif.py
Generate a 5-frame animated PNG that cross-fades from the dry seal to the wet seal.
Frames are evenly spaced blends:
  frame 0 = 100% dry
  frame 1 =  75% dry / 25% wet
  frame 2 =  50% dry / 50% wet
  frame 3 =  25% dry / 75% wet
  frame 4 = 100% wet

Output: dry_to_wet.apng  (in the same directory as this script)
Note: APNG (Animated PNG) is used instead of GIF to preserve full 24-bit color.
GIF is limited to 256 colors which causes severe color distortion in photo-realistic renders.
APNG is supported by all modern browsers (Chrome, Firefox, Safari, Edge).
"""

from pathlib import Path
from PIL import Image
import numpy as np

HERE   = Path(__file__).parent
OUT    = HERE / "dry_to_wet.apng"

# Use the exact same PNGs as the final report so frames 1 and 5 are identical
# to the submitted report images.
DRY_PNG = HERE / "seal_dry.png"
WET_PNG = HERE / "seal_wet.png"

# ----------- load ----------------------------------------------------------------
dry = Image.open(DRY_PNG).convert("RGB")
wet = Image.open(WET_PNG).convert("RGB")

# Make same size (resize wet to dry's size if they differ)
if dry.size != wet.size:
    wet = wet.resize(dry.size, Image.LANCZOS)

dry_np = np.asarray(dry, dtype=np.float32)
wet_np = np.asarray(wet, dtype=np.float32)

# ----------- blend ---------------------------------------------------------------
N_FRAMES = 5
frame_alphas    = [i / (N_FRAMES - 1) for i in range(N_FRAMES)]  # 0.0→1.0 (wet weight)
frame_durations = []

frames = []
for i, alpha in enumerate(frame_alphas):
    blended = (1.0 - alpha) * dry_np + alpha * wet_np
    blended = np.clip(blended, 0, 255).astype(np.uint8)
    frames.append(Image.fromarray(blended, "RGB"))
    # Hold the first and last frame longer
    if i == 0 or i == N_FRAMES - 1:
        frame_durations.append(1200)   # 1.2 s
    else:
        frame_durations.append(600)    # 0.6 s

# ----------- save ----------------------------------------------------------------
frames[0].save(
    OUT,
    format="PNG",     # APNG — full 24-bit, no palette quantization
    save_all=True,
    append_images=frames[1:],
    duration=frame_durations,
    loop=0,           # loop forever
)

print(f"Saved {N_FRAMES}-frame GIF to: {OUT}")
print(f"Frame sizes: {frames[0].size}")
print(f"Frame durations (ms): {frame_durations}")
