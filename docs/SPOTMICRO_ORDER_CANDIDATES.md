# SpotMicro order candidates

Build choice:

- Target: full hardware capability eventually.
- First physical milestone: basic walking dog first.
- Initial physical stack: Raspberry Pi + PCA9685 + 12 servos + 2S LiPo + UBEC.
- Add later: LCD, RPLidar A1/SLAM, extra sensors.

## Servo candidates

Important: the final robot should use 12 matching servos. Do not mix servo models across the four legs except for bench testing.

If buying samples first, buy one of each candidate and test fit against the printed `_cls`/`_mg` parts. If buying a full set now, the safest repo-aligned choice is the JX CLS6336HV-style servo because KDY provides `_cls` files and Mike says the CLS6336HV print files fit his HV5523MG servos.

| Rank | Servo | Buy/source link | Print-file family | Why |
|---|---|---|---|---|
| 1 | JX CLS6336HV high-voltage coreless servo | https://hobbyking.com/en_us/jx-cls6336hv-high-voltage-coreless-metal-gear-high-torque-servo-35-6kg-0-11sec-63g.html | `_cls` | Best match to KDY's upgraded servo files. Specs list 6.0-7.4V, 35.6 kg-cm at 7.4V, 25T spline, and roughly standard-servo dimensions. |
| 2 | JX/PDI-HV5523MG servo | https://servodatabase.com/servo/jx-servo/pdi-hv5523mg | `_cls` | This is the servo model Mike's repo explicitly says he used. It may be harder to source, but it is the closest historical match. |
| 3 | DS3235 / DS3235SG 35 kg servo | https://plexrobotics.com/products/dsservo-ds3235sg-35kg-servo | Human fit check; probably `_cls` first | Strong modern high-torque option: 5.0-7.4V, 35 kg-cm at 7.4V, 25T spline. Fit must be checked before committing to 12. |

Fallback only:

- MG996R can be used with KDY's `_mg` files, but it is weaker than the high-voltage options and it is not the servo Mike documented for his repo build.
- If you choose MG996R, the entire print-selection sheet changes from `_cls` to `_mg`.

## Battery candidates

Use 2S / 7.4V LiPo packs. Prefer XT60 connectors so the power harness stays sane.

It is reasonable to order more than one battery. It is not necessary to order every charger.

| Rank | Battery | Buy/source link | Why |
|---|---|---|---|
| 1 | Turnigy 4000mAh 2S 7.4V 40C LiPo w/ XT60 | https://hobbyking.com/en_us/turnigy-4000mah-2s-40c-lipo-pack-xt-60.html | Closest to Mike's documented 2S 4000mAh LiPo. |
| 2 | PowerHobby 5200mAh 2S 7.4V 50C LiPo w/ XT60 | https://www.powerhobby.com/products/powerhobby-2s-7-4v-5200mah-50c-lipo-battery-w-xt60-plug-adapter | More capacity and discharge headroom; good full-build candidate if it physically fits. |
| 3 | Ovonic 5200mAh 2S 7.4V 50C LiPo w/ XT60 | https://us.ovonicshop.com/products/ovonic-50c-7-4v-5200mah-2s1p-xt60-lipo-battery | Budget-friendly 5200mAh option with XT60; check dimensions against printed battery location. |

## Charger candidates

Buy one good balance charger. You do not need three chargers.

| Rank | Charger | Buy/source link | Why |
|---|---|---|---|
| 1 | Genuine SkyRC iMAX B6AC V2 | https://www.pololu.com/product/2588 | Good beginner-safe choice with AC input and included XT60 cable; Pololu notes they source genuine SkyRC units. |
| 2 | ToolkitRC M6DAC | https://www.team-blacksheep.com/products/prod:toolkitrc_m6dac | More powerful dual-channel charger; useful if you end up with several packs. |
| 3 | ISDT 608AC | https://www.isdt.co/608ac.html?lang=en | Compact AC/DC charger option; supports 1-6S lithium packs. |

## Must-buy safety/support items

- LiPo safe bag.
- XT60 connectors/adapters matching the batteries.
- Proper-gauge silicone wire for the servo power rail.
- Fuse or easy main disconnect.
- 5V/5A or better UBEC/buck regulator for Raspberry Pi + logic power.

## ZIP status

The corrected local KDY0523 ZIPs are valid and no longer duplicates. See `docs/SPOTMICRO_KDY0523_DOWNLOAD_AUDIT.md`.

The previously missing files are now present:

- `B_cover_cls.stl`
- `non-mega_B_cover_mg.stl`
- `non-mega_B_cover_cls.stl`
- `R_cover.stl`
