# How the install page ships

This page (`index.html` + `manifest.json`) flashes the compass firmware to an
M5Stack AtomS3 over Web Serial using
[ESP Web Tools](https://esphome.github.io/esp-web-tools/). Everything on the
page is static except the firmware image and the version number, both produced
by CI.

## The pipeline (two workflows)

Firmware builds and page deploys are decoupled. The firmware binary is handed
off between them as a **GitHub Release asset**.

**`.github/workflows/firmware.yml`** — runs only on a pushed tag `v*`:

1. builds the firmware with `pio run -e M5AtomS3` (the web pages in `data/`
   are embedded into the firmware by `scripts/embed_web.py`);
2. attaches the merged image `firmware.factory.bin` to the GitHub Release for
   that tag (creating the release if needed).

It does not touch Pages. Run it only for real firmware releases.

**`.github/workflows/pages.yml`** — deploys `docs/` to GitHub Pages. Triggers:

- a `push` to `main` touching `docs/**` (edit the page, just push);
- `workflow_dispatch` (manual button);
- `workflow_run` after `firmware.yml` succeeds (so a release refreshes the page).

It downloads `firmware.factory.bin` from the **latest** release, reads the
version from that release tag (`v1.2.3` -> `1.2.3`), stamps it into
`manifest.json`, and deploys `docs/` + `firmware/`.

No binaries are committed to the repo — they live only as release assets and in
the Pages artifact.

## Why one merged file

`firmware.factory.bin` is a full-flash image starting at offset `0`
(bootloader, partition table, `boot_app0` and the app combined), so ESP Web
Tools flashes it as a single part.

`new_install_prompt_erase` in `manifest.json` makes the installer ask whether
to erase the device. Erasing wipes NVS: saved WiFi networks and the sensor
calibration. Updates should not erase.

## Reproducible builds

`platformio.ini` pins the platform (pioarduino fork, exact tag) and every
library version, so CI builds the same firmware as a local build. Update the
pins deliberately and test on the device before tagging.

## Versioning

Cut a release by pushing a tag: `git tag v1.2.3 && git push origin v1.2.3`.
The page fetches `manifest.json` at load time and shows the version next to the
install button.

## Hosting

Served from **https://ok1cdj.github.io/M5AtomS3-compass/** via GitHub Pages
(Source = *GitHub Actions*). Web Serial requires HTTPS, which Pages provides.

When the custom domain is ready:

- **DNS:** `<domain>` CNAME → `ok1cdj.github.io.`
- add `docs/CNAME` containing the domain (pages.yml copies it when present);
- **Repo Settings → Pages:** set the custom domain and enable *Enforce HTTPS*
  once the certificate is issued.
