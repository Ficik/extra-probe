# Klipper Extra probe

Allows define named probes in klipper. I use it for autoZ offset, while still using tap to home.

My setup is sexbolt and tap. I probe towards sexbolt, when that triggers I probe at the same place using tap.
When that triggers z offset is `last probe z of tap - last probe z of sexbolt + omron switch overtravel (usually 0.25mm)`


## install

symlink it to klippers extras:

```
ln -sf "~/extra-probe/extra_probe.py" "~/klipper/klippy/extras/extra_probe.py"
```
