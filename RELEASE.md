# Releasing

## Release a new version

Bump the version in `Cargo.toml` (and let it propagate to `pyproject.toml` via maturin) and commit, then tag and push:

```bash
git commit -am "Release v1.0.1"
git push
git tag -a v1.0.1 -m "Release v1.0.1"
git push origin v1.0.1
```