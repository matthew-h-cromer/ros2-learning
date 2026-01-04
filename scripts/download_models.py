#!/usr/bin/env python3
"""Download Whisper models for offline use."""

import argparse
import os
import sys

# Add the speech_recognition package to path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, os.path.join(REPO_ROOT, "src", "speech_recognition"))

from speech_recognition.constants import DEFAULT_WHISPER_MODEL, MODELS_DIR
from speech_recognition.whisper_recognizer import WHISPER_MODEL_SIZES

# Set HuggingFace cache to repo-local directory
os.environ.setdefault("HF_HOME", os.path.join(MODELS_DIR, "huggingface"))


def download_whisper(model_name: str) -> None:
    """Download a Whisper model by loading it (triggers HuggingFace download)."""
    from faster_whisper import WhisperModel

    size_str = WHISPER_MODEL_SIZES.get(model_name, "unknown size")
    print(f"Downloading Whisper model: {model_name} ({size_str})")
    print("(Downloading from Hugging Face Hub...)")

    # Loading the model triggers the download if not cached
    WhisperModel(model_name, device="cpu", compute_type="int8")
    print(f"Whisper model ready: {model_name}")


def main():
    parser = argparse.ArgumentParser(
        description="Download Whisper models for offline use.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s              # Download default model (small)
  %(prog)s base         # Download base model
  %(prog)s tiny base    # Download multiple models

Available Whisper models:
  tiny    (~75 MB)
  base    (~140 MB)
  small   (~460 MB)
  medium  (~1.5 GB)
  large   (~3 GB)
""",
    )

    parser.add_argument(
        "models",
        metavar="MODEL",
        nargs="*",
        default=[DEFAULT_WHISPER_MODEL],
        help=f"Whisper model(s) to download (default: {DEFAULT_WHISPER_MODEL})",
    )

    args = parser.parse_args()

    for model_name in args.models:
        print("=" * 60)
        print(f"WHISPER MODEL: {model_name}")
        print("=" * 60)
        download_whisper(model_name)
        print()

    print("Done!")


if __name__ == "__main__":
    main()
