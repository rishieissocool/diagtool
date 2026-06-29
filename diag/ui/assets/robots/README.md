# Robot photos (optional)

Drop a photo of each robot here and the **Robots** tab will show it instead of
the drawn placeholder. No restart logic is needed beyond reopening the tool.

* Name each file by the robot's label: `Y0.png`, `Y1.png`, `B1.png`, …
  (team letter `Y`/`B` + shell id, exactly as shown in the app).
* Supported: `.png`, `.jpg`, `.jpeg`, `.webp`, `.bmp`.
* Square-ish images look best; they are centre-cropped to a square.

Want them somewhere else (e.g. a shared drive)? Set `robot_photo_dir:` in
`diag_settings.yaml` to that folder. Files there win; this folder is the
fallback. Any robot still without a photo gets a clean team-coloured placeholder.
