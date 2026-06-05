.PHONY: help build release release-full test check clippy fmt fmt-check install install-full clean

# Show the available targets (default).
help:
	@echo "convobs build targets:"
	@echo "  make build         debug build (fast compile)"
	@echo "  make release       optimized build -> target/release/convobs, target/release/diffobs"
	@echo "  make release-full  optimized build with the external RINEX backend (adds CRINEX input)"
	@echo "  make install       install convobs and diffobs into ~/.cargo/bin"
	@echo "  make install-full  install with the external RINEX backend"
	@echo "  make test          run all tests"
	@echo "  make check         type-check without building binaries (fastest feedback)"
	@echo "  make clippy        lint"
	@echo "  make fmt           format the code"
	@echo "  make clean         remove build artifacts"

# Debug build (fast compile).
build:
	cargo build

# Optimized release build. Binaries land in target/release/convobs and
# target/release/diffobs.
release:
	cargo build --release

# Optimized release build including the external RINEX backend (the bundled
# `rinex` crate), which adds CRINEX (Hatanaka) input support.
release-full:
	cargo build --release --features convobs-cli/rinex-crate

# Install the convobs and diffobs binaries into ~/.cargo/bin.
install:
	cargo install --path cli

# Install with the external RINEX backend.
install-full:
	cargo install --path cli --features rinex-crate

# Run all tests.
test:
	cargo test --workspace

# Type-check without producing binaries (fastest feedback).
check:
	cargo check --workspace

# Lint.
clippy:
	cargo clippy --workspace --all-targets -- -D warnings

# Format code.
fmt:
	cargo fmt

# Check formatting without modifying files.
fmt-check:
	cargo fmt -- --check

# Remove build artifacts.
clean:
	cargo clean
