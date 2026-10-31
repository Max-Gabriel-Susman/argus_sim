//! One version for the package, the crates and the wheel.
//!
//! colcon reads the version from package.xml, Cargo from
//! `[workspace.package]`, and maturin gives the wheel Cargo's (pyproject.toml
//! declares it dynamic). Nothing makes the first two agree, so this test
//! does. The bindings' own tests check the built wheel's `__version__`
//! against the same two files.

use std::error::Error;
use std::fs;
use std::path::Path;

type TestResult = Result<(), Box<dyn Error>>;

/// The version Cargo built this package with, which every crate inherits
/// from `[workspace.package]`.
const CARGO_VERSION: &str = env!("CARGO_PKG_VERSION");

/// A file at the repository root, which is this package's directory.
fn read(name: &str) -> Result<String, Box<dyn Error>> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join(name);
    fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()).into())
}

/// The text between `<version>` and `</version>` in package.xml.
fn package_xml_version(xml: &str) -> Option<&str> {
    let (_, rest) = xml.split_once("<version>")?;
    let (version, _) = rest.split_once("</version>")?;
    Some(version.trim())
}

#[test]
fn the_version_is_2_0_0() {
    // The first release of the Rust relay; 1.x was the C++ one.
    assert_eq!(CARGO_VERSION, "2.0.0");
}

#[test]
fn package_xml_carries_cargos_version() -> TestResult {
    let xml = read("package.xml")?;
    assert_eq!(package_xml_version(&xml), Some(CARGO_VERSION));
    Ok(())
}

#[test]
fn the_bindings_take_their_version_from_cargo() -> TestResult {
    let manifest = read("crates/argus-sim-py/Cargo.toml")?;
    assert!(
        manifest
            .lines()
            .any(|line| line.trim() == "version.workspace = true"),
        "argus-sim-py does not inherit the workspace version"
    );
    let pyproject = read("crates/argus-sim-py/pyproject.toml")?;
    assert!(
        pyproject
            .lines()
            .any(|line| line.trim() == r#"dynamic = ["version"]"#),
        "pyproject.toml does not take the wheel's version from Cargo"
    );
    assert!(
        !pyproject
            .lines()
            .any(|line| line.trim_start().starts_with("version")),
        "pyproject.toml sets a version of its own"
    );
    Ok(())
}
