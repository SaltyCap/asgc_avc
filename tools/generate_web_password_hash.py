#!/usr/bin/env python3
"""Generate ASGC web login password hashes (PBKDF2-SHA256)."""

import argparse
import base64
import getpass
import hashlib
import secrets

ALGORITHM = "pbkdf2_sha256"
DEFAULT_ITERATIONS = 260000


def _b64encode_no_padding(raw_bytes):
    return base64.urlsafe_b64encode(raw_bytes).decode("ascii").rstrip("=")


def generate_password_hash(password, iterations=DEFAULT_ITERATIONS):
    salt = secrets.token_bytes(16)
    derived_key = hashlib.pbkdf2_hmac(
        "sha256", password.encode("utf-8"), salt, int(iterations)
    )
    return (
        f"{ALGORITHM}"
        f"${int(iterations)}"
        f"${_b64encode_no_padding(salt)}"
        f"${_b64encode_no_padding(derived_key)}"
    )


def _read_password(args):
    if args.password:
        return args.password

    first = getpass.getpass("Enter web login password: ")
    second = getpass.getpass("Re-enter password: ")
    if first != second:
        raise SystemExit("Passwords did not match.")
    if not first:
        raise SystemExit("Password cannot be empty.")
    return first


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--password", help="Password to hash (avoid shell history in shared environments).")
    parser.add_argument(
        "--iterations",
        type=int,
        default=DEFAULT_ITERATIONS,
        help=f"PBKDF2 iterations (default: {DEFAULT_ITERATIONS}).",
    )
    args = parser.parse_args()

    password = _read_password(args)
    password_hash = generate_password_hash(password, iterations=args.iterations)
    print(password_hash)
    print()
    print(f'export ASGC_WEB_PASSWORD_HASH="{password_hash}"')


if __name__ == "__main__":
    main()
