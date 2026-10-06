#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

import os
import sys
import json
import queue
import re
import threading
import signal
import sounddevice as sd
from vosk import Model, KaldiRecognizer

WORD_TO_NUM = {
    'zero': 0, 'one': 1, 'won': 1, 'two': 2, 'to': 2, 'too': 2,
    'three': 3, 'four': 4, 'for': 4, 'five': 5, 'six': 6,
    'seven': 7, 'eight': 8, 'ate': 8, 'nine': 9, 'ten': 10,
    'eleven': 11, 'twelve': 12, 'thirteen': 13, 'fourteen': 14,
    'fifteen': 15, 'sixteen': 16, 'seventeen': 17, 'eighteen': 18,
    'nineteen': 19, 'twenty': 20, 'thirty': 30
}

class VoiceTrackerNode(Node):
    def __init__(self):
        super().__init__('voice_tracker_node')
        self.publisher = self.create_publisher(Int16, '/tracker_id', 10)

        model_path = os.path.expanduser('~/.vosk_model')
        if not os.path.exists(model_path):
            self.get_logger().error(f"Vosk model not found at {model_path}!")
            sys.exit(1)

        self.get_logger().info("Loading speech model...")
        self.model = Model(model_path)
        self.audio_queue = queue.Queue()
        self.running = True
        self.stream = None

        self.listen_thread = threading.Thread(target=self.audio_processing_loop, daemon=True)
        self.listen_thread.start()

        self.get_logger().info("Voice Commands Ready:")
        self.get_logger().info(" -> 'Robot, follow [ID]' / 'Switch to [ID]' / 'ID [ID]' (changes target)")
        self.get_logger().info(" -> 'Robot, stop' / 'halt' (stops immediately)")
        self.get_logger().info(" -> 'Robot, resume' / 'continue' (resumes tracking)")

    def audio_callback(self, indata, frames, time_info, status):
        if self.running:
            self.audio_queue.put(bytes(indata))

    def extract_number(self, text):
        # 1. Match direct digits (e.g., "follow 2", "id 15")
        digits = re.findall(r'\b\d+\b', text)
        if digits:
            return int(digits[0])
        # 2. Match spoken words (e.g., "follow two", "id five")
        for word, val in sorted(WORD_TO_NUM.items(), key=lambda x: -len(x[0])):
            if re.search(rf'\b{word}\b', text):
                return val
        return None

    def parse_command(self, text, recognizer):
        text = text.lower().strip()
        if not text:
            return

        self.get_logger().info(f"? Heard: '{text}'")

        # 1. EMERGENCY STOP COMMANDS
        if any(w in text for w in ["stop", "halt", "pause", "wait", "freeze"]):
            self.get_logger().info("? Action: STOP (-1)")
            self.publisher.publish(Int16(data=-1))
            recognizer.Reset()
            return

        # 2. TARGET ID SWITCH COMMANDS (Checked BEFORE general follow)
        num = self.extract_number(text)
        if num is not None:
            self.get_logger().info(f"? Action: Switch to Target ID {num}")
            self.publisher.publish(Int16(data=num))
            recognizer.Reset()
            return

        # 3. RESUME / AUTO-LOCK (Only if no specific number was spoken)
        if any(w in text for w in ["resume", "continue", "start", "follow", "track", "go"]):
            self.get_logger().info("?? Action: RESUME (-2)")
            self.publisher.publish(Int16(data=-2))
            recognizer.Reset()
            return

    def audio_processing_loop(self):
        sample_rate = 16000
        recognizer = KaldiRecognizer(self.model, sample_rate)

        try:
            self.stream = sd.RawInputStream(
                samplerate=sample_rate,
                blocksize=4000,
                dtype='int16',
                channels=1,
                callback=self.audio_callback
            )
            with self.stream:
                while self.running and rclpy.ok():
                    try:
                        # Non-blocking get allows thread to notice shutdown cleanly
                        data = self.audio_queue.get(timeout=0.2)
                    except queue.Empty:
                        continue

                    if recognizer.AcceptWaveform(data):
                        res = json.loads(recognizer.Result())
                        spoken_text = res.get("text", "")
                        if spoken_text:
                            self.parse_command(spoken_text, recognizer)
        except Exception as e:
            if self.running:
                self.get_logger().error(f"Audio stream error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceTrackerNode()

    # Clean signal exit handler for Ctrl+C
    def sigint_handler(sig, frame):
        node.running = False
        try:
            if node.stream is not None:
                node.stream.stop()
                node.stream.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
        os._exit(0)

    signal.signal(signal.SIGINT, sigint_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        sigint_handler(None, None)

if __name__ == '__main__':
    main()
