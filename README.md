# restock-bot

Teach an arm a motion once, relative to a fiducial marker on a fixture (a drawer, a bin, a shelf), and replay it on any machine that can see the same marker. Motions are stored as [Viam Data](https://docs.viam.com/data-ai/) sequences, so every machine in the organization can fetch them, and the dense recordings can later be used to fit motion primitives.

## Models

- [`viam-labs:restock-bot:fixture-teacher`](viam-labs_restock-bot_fixture-teacher.md): generic service; configure one per fixture.

## Development

```sh
make test                 # unit tests
make module.tar.gz        # build the module archive for this platform
viam module reload --part-id <part-id>   # build and hot-load onto a machine
```

## Releasing

Pushing a semver tag (`v0.1.0`, `v0.1.0-rc1`) runs `.github/workflows/deploy.yml`, which tests, cloud-builds for every platform in `meta.json`, and uploads that version to the registry. The repository needs `VIAM_KEY_ID` and `VIAM_KEY_VALUE` secrets holding an organization API key for the `viam-labs` namespace.

```sh
git tag v0.1.0 && git push origin v0.1.0
```

After adding or renaming models, run `viam module update-models --binary bin/restock-bot` so `meta.json` lists them.
