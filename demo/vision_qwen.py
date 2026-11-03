"""
Qwen2.5-VL classifier for the demo cards.

Loads the model on first use and asks for {"digit", "colour"} as JSON.
Anything it can't parse, or a digit/colour the board wasn't trained on,
comes back as None so vision_mnist falls back to the LeNet path.
"""
from __future__ import annotations

import json
import re
import time
from pathlib import Path

import numpy as np
import cv2


VALID_COLOURS = {"RED", "GREEN", "BLUE", "MAGENTA", "WHITE"}
VALID_DIGITS  = {1, 2, 3, 4, 5, 6, 7}

PROMPT = (
    "Look at this image of a card. Exactly ONE digit (1-7) is visible on "
    "a coloured background. The background is one of RED, GREEN, BLUE, "
    "MAGENTA, or WHITE.\n\n"
    "Respond with ONLY this JSON format, nothing else:\n"
    '{"digit": N, "colour": "X"}\n\n'
    "where N is the digit number (1-7) and X is the colour name. "
    "Make your best guess for the digit - any digit 1-7 is valid. "
    "Please dont confuse numbers with other similarly looking mnist numbers- accuracy is everything here. "
    "Do not use null; pick the most likely digit even if unsure."
)

class QwenClassifier:
    """Qwen2.5-VL wrapper. Weights load on the first classify() call."""

    def __init__(self, model_path: str, device: str = "auto"):
        """
        model_path: folder written by qwen_model/download_qwen.py
        device: 'auto', 'cuda' or 'cpu'
        """
        self.model_path = model_path
        self.device = device
        self._model = None
        self._processor = None
        self._loaded = False
        self._load_error = None

    def _lazy_load(self):
        if self._loaded or self._load_error is not None:
            return
        try:
            import torch
            from transformers import (
                Qwen2_5_VLForConditionalGeneration,
                AutoProcessor,
                BitsAndBytesConfig,
            )

            path = Path(self.model_path)
            if not path.exists():
                raise FileNotFoundError(
                    f"VLM weights not found at {path}. "
                    f"Run the download script first.")

            if self.device == "auto":
                self.device = "cuda" if torch.cuda.is_available() else "cpu"

            print(f"[vlm] loading weights on {self.device} (bnb-int4) ...")
            t0 = time.time()

            if self.device == "cuda":
                bnb_config = BitsAndBytesConfig(
                    load_in_4bit=True,
                    bnb_4bit_quant_type="nf4",
                    bnb_4bit_compute_dtype=torch.float16,
                    bnb_4bit_use_double_quant=True,
                )
                self._model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
                    str(path),
                    device_map=self.device,
                    quantization_config=bnb_config,
                )
            else:
                self._model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
                    str(path),
                    torch_dtype=torch.float16,
                    device_map="cpu",
                )

            self._processor = AutoProcessor.from_pretrained(str(path))
            self._model.eval()
            self._torch = torch
            self._loaded = True
            print(f"[vlm] loaded in {time.time()-t0:.1f}s")
        except Exception as e:
            self._load_error = e
            print(f"[vlm] load failed: {e}")

    def classify(self, patch_bgr: np.ndarray) -> dict | None:
        """Returns colour, digit, confidence, raw_text and latency_ms,
        or None if the model didn't load."""
        self._lazy_load()
        if not self._loaded:
            return None

        t0 = time.time()

        rgb = cv2.cvtColor(patch_bgr, cv2.COLOR_BGR2RGB)
        from PIL import Image
        pil_img = Image.fromarray(rgb)

        messages = [{
            "role": "user",
            "content": [
                {"type": "image", "image": pil_img},
                {"type": "text", "text": PROMPT},
            ],
        }]

        text = self._processor.apply_chat_template(
            messages, tokenize=False, add_generation_prompt=True)

        inputs = self._processor(
            text=[text],
            images=[pil_img],
            padding=True,
            return_tensors="pt",
        ).to(self.device)
        with self._torch.no_grad():
            generated_ids = self._model.generate(
                **inputs,
                max_new_tokens=32,            # the JSON answer is ~20 tokens
                do_sample=False,
                pad_token_id=self._processor.tokenizer.eos_token_id,
            )
        trimmed = generated_ids[:, inputs.input_ids.shape[1]:]
        raw_text = self._processor.batch_decode(
            trimmed, skip_special_tokens=True,
            clean_up_tokenization_spaces=False)[0]

        latency_ms = (time.time() - t0) * 1000.0
        parsed = _parse_response(raw_text)
        parsed["raw_text"] = raw_text.strip()
        parsed["latency_ms"] = latency_ms

        # free the activations and KV cache, otherwise local PC GPU runs out - FYI 
        # I ran it on my local GPU- depending on your XPU you can change settings...
        # after 2-3 captures
        if self.device == "cuda":
            del inputs, generated_ids, trimmed
            self._torch.cuda.empty_cache()

        return parsed


def _parse_response(raw_text: str) -> dict:
    """Pull colour and digit out of Qwen's text. Copes with code fences,
    extra prose, "color" vs "colour" and digits given as strings."""
    out = {"colour": None, "digit": None, "confidence": 0.0}

    if not raw_text:
        return out

    cleaned = re.sub(r"```(?:json)?\s*", "", raw_text)
    cleaned = re.sub(r"\s*```", "", cleaned)

    match = re.search(r"\{[^{}]*\}", cleaned, re.DOTALL)
    if not match:
        return out

    try:
        data = json.loads(match.group(0))
    except json.JSONDecodeError:
        return out

    colour_raw = data.get("colour", data.get("color"))
    if isinstance(colour_raw, str):
        c = colour_raw.strip().upper()
        if c in VALID_COLOURS:
            out["colour"] = c

    d_raw = data.get("digit")
    try:
        if d_raw is not None:
            d = int(d_raw)
            if d in VALID_DIGITS:
                out["digit"] = f"D{d}"
    except (ValueError, TypeError):
        pass

    # 0.5 when only one of the two fields came through
    if out["colour"] is None and out["digit"] is None:
        out["confidence"] = 0.0
    elif out["colour"] is None or out["digit"] is None:
        out["confidence"] = 0.5
    else:
        out["confidence"] = 1.0

    return out


if __name__ == "__main__":
    # parser check, no model needed
    tests = [
        '{"digit": 5, "colour": "MAGENTA"}',
        '```json\n{"digit": 3, "color": "blue"}\n```',
        '  Sure! Here you go: {"digit":"7","colour":"RED"}',
        '{"digit": null, "colour": "GREEN"}',
        '{"digit": 9, "colour": "PURPLE"}',
        'no json here',
        '',
    ]
    for t in tests:
        print(repr(t), "->", _parse_response(t))
