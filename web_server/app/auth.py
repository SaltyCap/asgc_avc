import base64
import hashlib
import hmac
import os
import secrets
from pathlib import Path
from urllib.parse import urlparse

from flask import jsonify, redirect, request, session, url_for

from .config import Config

_runtime_password_hash = None
_runtime_password_source = None
_password_announced = False

PUBLIC_PATHS = frozenset({"/login", "/logout", "/setup", "/favicon.ico"})
PUBLIC_PREFIXES = ("/static/",)
AUTH_SESSION_KEY = "auth_ok"


def _b64encode_no_padding(raw_bytes):
    return base64.urlsafe_b64encode(raw_bytes).decode("ascii").rstrip("=")


def _b64decode_no_padding(encoded_text):
    padding = "=" * (-len(encoded_text) % 4)
    return base64.urlsafe_b64decode(encoded_text + padding)


def generate_password_hash(password, iterations=None, salt=None):
    if not password:
        raise ValueError("Password cannot be empty.")

    if iterations is None:
        iterations = Config.PASSWORD_HASH_ITERATIONS
    iterations = int(iterations)

    if salt is None:
        salt = secrets.token_bytes(16)

    derived_key = hashlib.pbkdf2_hmac(
        "sha256", password.encode("utf-8"), salt, iterations
    )
    return (
        f"{Config.PASSWORD_HASH_ALGORITHM}"
        f"${iterations}"
        f"${_b64encode_no_padding(salt)}"
        f"${_b64encode_no_padding(derived_key)}"
    )


def _verify_password_hash(candidate, password_hash):
    if not password_hash:
        return False

    try:
        algorithm, iterations_str, salt_b64, digest_b64 = password_hash.split("$", 3)
        if algorithm != Config.PASSWORD_HASH_ALGORITHM:
            return False

        iterations = int(iterations_str)
        if iterations <= 0:
            return False

        salt = _b64decode_no_padding(salt_b64)
        expected_digest = _b64decode_no_padding(digest_b64)
        computed_digest = hashlib.pbkdf2_hmac(
            "sha256",
            str(candidate).encode("utf-8"),
            salt,
            iterations,
            dklen=len(expected_digest),
        )
        return hmac.compare_digest(expected_digest, computed_digest)
    except Exception:
        return False


def _is_password_hash_format_valid(password_hash):
    try:
        algorithm, iterations_str, salt_b64, digest_b64 = password_hash.split("$", 3)
        if algorithm != Config.PASSWORD_HASH_ALGORITHM:
            return False
        iterations = int(iterations_str)
        if iterations <= 0:
            return False
        if not _b64decode_no_padding(salt_b64):
            return False
        if not _b64decode_no_padding(digest_b64):
            return False
        return True
    except Exception:
        return False


def _get_hash_file_path():
    configured_path = (Config.AUTH_PASSWORD_HASH_FILE or "").strip()
    if not configured_path:
        return None
    return Path(configured_path).expanduser()


def _read_hash_from_file():
    hash_file = _get_hash_file_path()
    if hash_file is None or not hash_file.exists():
        return None

    try:
        file_hash = hash_file.read_text(encoding="utf-8").strip()
        if _is_password_hash_format_valid(file_hash):
            return file_hash
    except Exception:
        return None
    return None


def _write_hash_to_file(password_hash):
    hash_file = _get_hash_file_path()
    if hash_file is None:
        return False

    try:
        hash_file.parent.mkdir(parents=True, exist_ok=True)
        hash_file.write_text(f"{password_hash}\n", encoding="utf-8")
        try:
            os.chmod(hash_file, 0o600)
        except OSError:
            # Best-effort permission hardening on non-POSIX filesystems.
            pass
        return True
    except Exception:
        return False


def set_password_from_plaintext(password):
    """Set/replace web password by hashing and persisting to local hash file."""
    if not password:
        return False

    password_hash = generate_password_hash(password)
    if not _write_hash_to_file(password_hash):
        return False

    global _runtime_password_hash
    global _runtime_password_source

    _runtime_password_hash = password_hash
    _runtime_password_source = "hash_file"
    os.environ.pop(Config.AUTH_PASSWORD_ENV, None)
    return True


def get_auth_password_hash():
    configured_hash = os.getenv(Config.AUTH_PASSWORD_HASH_ENV, "").strip()
    if configured_hash and _is_password_hash_format_valid(configured_hash):
        return configured_hash, "configured_hash"

    global _runtime_password_hash
    global _runtime_password_source

    if _runtime_password_hash:
        return _runtime_password_hash, _runtime_password_source or "runtime"

    configured_password = os.getenv(Config.AUTH_PASSWORD_ENV, "")
    if configured_password:
        _runtime_password_hash = generate_password_hash(configured_password)
        if _write_hash_to_file(_runtime_password_hash):
            _runtime_password_source = "persisted_from_plaintext_env"
        else:
            _runtime_password_source = "derived_from_plaintext_env"
        # Remove plaintext from process environment after deriving hash in memory.
        os.environ.pop(Config.AUTH_PASSWORD_ENV, None)
        return _runtime_password_hash, _runtime_password_source

    file_hash = _read_hash_from_file()
    if file_hash:
        _runtime_password_hash = file_hash
        _runtime_password_source = "hash_file"
        return _runtime_password_hash, _runtime_password_source

    return None, "unconfigured"


def is_auth_configured():
    password_hash, _ = get_auth_password_hash()
    return bool(password_hash)


def announce_password_once(logger):
    global _password_announced
    if _password_announced:
        return

    password_hash, source = get_auth_password_hash()

    if source == "configured_hash":
        logger.info("Web authentication enabled with ASGC_WEB_PASSWORD_HASH.")
    elif source == "hash_file":
        logger.info(
            "Web authentication enabled with stored password hash file: %s",
            Config.AUTH_PASSWORD_HASH_FILE,
        )
    elif source == "persisted_from_plaintext_env":
        logger.warning(
            "ASGC_WEB_PASSWORD detected. Converted to hash and saved to %s. "
            "Prefer ASGC_WEB_PASSWORD_HASH or the hash file going forward.",
            Config.AUTH_PASSWORD_HASH_FILE,
        )
    elif source == "derived_from_plaintext_env":
        logger.warning(
            "ASGC_WEB_PASSWORD detected. Converted to in-memory hash for this run. "
            "Set ASGC_WEB_PASSWORD_HASH and remove plaintext password env usage."
        )
    elif source == "unconfigured":
        logger.warning(
            "No web login password configured yet. Open /setup in the browser to create one. "
            "Hash file path: %s",
            Config.AUTH_PASSWORD_HASH_FILE,
        )
    else:
        logger.warning("Web authentication is using a temporary runtime configuration.")

    if password_hash and not _is_password_hash_format_valid(password_hash):
        logger.error(
            "Configured login hash is invalid. Expected format: %s$<iterations>$<salt>$<digest>",
            Config.PASSWORD_HASH_ALGORITHM,
        )

    _password_announced = True


def check_password(candidate):
    if not candidate:
        return False
    password_hash, _ = get_auth_password_hash()
    if not _is_password_hash_format_valid(password_hash):
        return False
    return _verify_password_hash(str(candidate), password_hash)


def login_user():
    session[AUTH_SESSION_KEY] = True


def logout_user():
    session.clear()


def is_authenticated():
    try:
        return bool(session.get(AUTH_SESSION_KEY))
    except RuntimeError:
        return False


def is_public_request(req):
    if req.path in PUBLIC_PATHS:
        return True
    if any(req.path.startswith(prefix) for prefix in PUBLIC_PREFIXES):
        return True
    if req.path in {"/audio", "/motor", "/gpio"}:
        # WebSocket auth is handled inside their respective handlers.
        return True
    return False


def unauthorized_response():
    if request.path.startswith("/api/"):
        return jsonify({"error": "authentication required"}), 401
    return redirect(url_for("main.login", next=_request_path_with_query()))


def setup_required_response():
    if request.path.startswith("/api/"):
        return jsonify({"error": "password setup required", "setup_required": True}), 503
    return redirect(url_for("main.setup", next=_request_path_with_query()))


def _request_path_with_query():
    if request.query_string:
        return f"{request.path}?{request.query_string.decode()}"
    return request.path


def sanitize_next_url(next_url, fallback):
    if not next_url:
        return fallback

    parsed = urlparse(next_url)
    if parsed.scheme or parsed.netloc:
        return fallback
    if not next_url.startswith("/"):
        return fallback
    return next_url


def extract_request_password(req):
    bearer_token = req.headers.get("Authorization", "")
    if bearer_token.lower().startswith("bearer "):
        return bearer_token.split(" ", 1)[1].strip()

    header_token = req.headers.get("X-ASGC-Password", "").strip()
    if header_token:
        return header_token

    return req.args.get("auth", "").strip()


def is_origin_allowed(req):
    origin = req.headers.get("Origin")
    if not origin:
        return True

    parsed_origin = urlparse(origin)
    if parsed_origin.netloc == req.host:
        return True

    allowed_origins = Config.ALLOWED_ORIGINS
    if not allowed_origins:
        return False

    return origin in allowed_origins or parsed_origin.netloc in allowed_origins


def is_websocket_authenticated(req):
    if is_authenticated():
        return True

    token = extract_request_password(req)
    return check_password(token)
