# Drone — Project Jager source

Camera-based glider recognition experiments, glider CAD-generation scripts,
and experimental ESP32 stabilization firmware for a classroom prototype.

This repository is a source snapshot. It does not contain a working autonomous
following integration. No Swarm Interceptor source has been copied into it.

## Source layout

- `trackinh.py`: stationary-camera motion regions, bounding boxes and center coordinates.
- `cnn.py`: image classifier architecture and inference.
- `cnn.ipynb`: notebook with saved outputs removed and portable path placeholders.
- `train_glider.py`, `make_glider_training_data.py`: synthetic glider recognition experiments.
- `collect_hard_negatives.py`, `retrain_classifier.py`, `train_round2.py`, `train_round3.py`: earlier image-classifier data and training experiments.
- `glider_design/`: printable airframe and paired concealed-ballast glider source generators.
- `firmware/StabilizeNowV4/`: latest local self-leveling controller source snapshot.
- Other `firmware/` folders: earlier controller versions and sensor/ESC bench sketches, retained separately.
- `SOURCE_MANIFEST.json`: file inventory and checksums for this upload.

## Vision setup

Install the Python dependencies with `python -m pip install -r requirements.txt`.
Dependency versions are intentionally not pinned: this source snapshot has not
been tested in a fresh environment.

Trained checkpoints and datasets are not included. Copy your existing
`glider_model.pth` next to `cnn.py` before using `python trackinh.py --glider`.
The default non-glider mode expects `missile_model.pth`, also omitted.
Training scripts need their original datasets/manifests and starting weights.
Machine-specific dataset paths in the exported code have been replaced by
relative `data/` placeholders; configure these for your own data.
Run the notebook from the repository root. It also needs the matching model/data.

The existing region detector assumes a stationary camera. Camera motion,
stationary objects, lighting and occlusion can invalidate its measurements.
Screen coordinates and classifier scores do not establish distance, physical
speed, reliable identities, or distinction between concealed ballast variants.

## ESP32 firmware

Each firmware folder is a separate Arduino sketch. For V4, copy
`NetworkConfig.example.h` to `NetworkConfig.h` locally and enter your network
settings. The real configuration is deliberately excluded from Git.
The exported access-point password is a placeholder; configure it locally.
Board pin mappings, IMU orientation and motor wiring must match the selected
sketch and your hardware. Hardware/library settings are not a portable build
configuration and a clean firmware build has not been verified for this snapshot.

V4 is experimental self-level control with no altitude or position hold.
Communication timeout stops motor output; it is not an autonomous hover or
landing feature. Its tuning/test routines are fixture-only. Free-flight
stability and autonomous following have not been established by this upload.

## Glider source

The gliders are experimental unpowered designs. Keep each generator at its
existing relative path so its local imports resolve. Generated CAD, images,
models and print packages are excluded from this code-only snapshot. See the
included assembly notes; geometry checks do not establish flight performance.

## What is excluded

Wi-Fi credentials, checkpoint binaries, training images and data manifests,
recordings, personal presentation material, notebook outputs, caches, compiled
firmware, generated build/vendor trees, duplicate nested sketches, and the
machine-specific original-file updater are excluded. The older third-party
ESP-FC source archives are excluded; the standalone project sketches are here.

Verification covers Python syntax, notebook structure, source inventory and a
credential/path scan only. It does not claim successful training, recognition
on printed gliders, a fresh dependency installation, firmware compilation,
hardware testing or automatic flight.
