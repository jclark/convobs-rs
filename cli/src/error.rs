//! The CLI error type.

use std::fmt;

/// A convobs/diffobs error. Carries enough structure to distinguish a usage
/// problem from an I/O failure from a conversion failure, while still rendering
/// to a single human-readable line.
#[derive(Debug)]
pub enum Error {
    /// Bad command line: unknown option, conflicting flags, invalid value.
    Usage(String),
    /// An I/O failure against a named path (or `stdin`/`stdout`).
    Io { path: String, source: std::io::Error },
    /// A failure parsing or converting input, with context already folded in.
    Conversion(String),
}

impl Error {
    pub fn usage(msg: impl Into<String>) -> Error {
        Error::Usage(msg.into())
    }

    pub fn conversion(msg: impl Into<String>) -> Error {
        Error::Conversion(msg.into())
    }

    /// Wraps an I/O error with the path it occurred on (`-` / empty ⇒ stdin).
    pub fn io(path: &str, source: std::io::Error) -> Error {
        Error::Io {
            path: path.to_string(),
            source,
        }
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Usage(m) | Error::Conversion(m) => f.write_str(m),
            Error::Io { path, source } => match path.as_str() {
                "" => write!(f, "{source}"),
                "-" => write!(f, "stdin: {source}"),
                p => write!(f, "{p}: {source}"),
            },
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::Io { source, .. } => Some(source),
            _ => None,
        }
    }
}

/// Conversion-layer errors (currently `String`) fold into [`Error::Conversion`],
/// so `?` keeps working while the boundary carries a typed error.
impl From<String> for Error {
    fn from(s: String) -> Error {
        Error::Conversion(s)
    }
}

pub type Result<T> = std::result::Result<T, Error>;
