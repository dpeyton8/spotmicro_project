# KDY0523 Thingiverse download audit

Source intended by repo: https://www.thingiverse.com/thing:3445283

Local ZIPs inspected:

- `Spotmicro - robot dog - 3445283 - part 1 of 2.zip`
- `Spotmicro - robot dog - 3445283 - part 2 of 2.zip`

Result:

- Both ZIPs are valid archives.
- `part 1 of 2` contains 26 files, including 4 STL files.
- `part 2 of 2` contains 62 files, including 30 STL files.
- The archives are no longer duplicates.

Conclusion:

The local KDY0523 Thingiverse ZIP set is now complete enough to resolve the previously missing rear/back cover files.

STL files in `part 2 of 2`:

- `files/plate.stl`
- `files/L_side_plate.stl`
- `files/non-mega_L_side_plate.stl`
- `files/R_side_plate.stl`
- `files/I_shoulder_mg.stl`
- `files/I_shoulder_cls.stl`
- `files/O_shoulder.stl`
- `files/L_arm_joint_mg.stl`
- `files/R_arm_joint_mg.stl`
- `files/L_arm_joint_cls.stl`
- `files/R_arm_joint_cls.stl`
- `files/L_arm_mg.stl`
- `files/R_arm_mg.stl`
- `files/L_arm_cls.stl`
- `files/R_arm_cls.stl`
- `files/L_arm_cover.stl`
- `files/R_arm_cover.stl`
- `files/L_wrist_mg.stl`
- `files/R_wrist_mg.stl`
- `files/L_wrist_cls.stl`
- `files/R_wrist_cls.stl`
- `files/foot.stl`
- `files/F_cover.stl`
- `files/L_ultra_sonic.stl`
- `files/R_ultra_sonic.stl`
- `files/T_cover_mg.stl`
- `files/T_cover_cls.stl`
- `files/non-mega_T_cover_mg.stl`
- `files/non-mega_T_cover_cls.stl`
- `files/B_cover_mg.stl`

STL files in `part 1 of 2`:

- `files/B_cover_cls.stl`
- `files/non-mega_B_cover_mg.stl`
- `files/non-mega_B_cover_cls.stl`
- `files/R_cover.stl`

Why this matters:

- The repo/Mike build uses HV5523MG servos, and Mike says the KDY `cls6336hv` files fit those servos.
- KDY says to print `_cls` files for CLS6336HV-style servos, and `_mg` files for MG995/MG996R servos.
- The corrected ZIPs include `non-mega_B_cover_cls.stl`, which is the likely rear/bottom-cover choice for the Raspberry Pi/PCA9685 + `_cls` Mike-style build.

Next action:

Use `docs/SPOTMICRO_KDY0523_EXACT_STL_SELECTION.csv` as the print/skip checklist before slicing.
