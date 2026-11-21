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
import subprocess
import time
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
        self.is_speaking = False
        self.stream = None

        self.listen_thread = threading.Thread(target=self.audio_processing_loop, daemon=True)
        self.listen_thread.start()

        # Announce system startup after audio initializes
        threading.Thread(target=self.initial_announcement, daemon=True).start()

        self.get_logger().info("Voice Commands Ready:")
        self.get_logger().info(" -> 'Robot' / 'Hey Robot' (wakes up and acknowledges)")
        self.get_logger().info(" -> 'Robot, follow [ID]' / 'Track [ID]' (locks target)")
        self.get_logger().info(" -> 'Robot, stop' (halts tracking)")
        self.get_logger().info(" -> 'Robot, resume' (resumes tracking)")

    def speak(self, text, recognizer=None):
        """Speaks using espeak-ng while muting the microphone to avoid self-triggering."""
        def run_speak():
            self.is_speaking = True
            # Clear old queued audio packets
            while not self.audio_queue.empty():
                try:
                    self.audio_queue.get_nowait()
                except queue.Empty:
                    break

            # -v en-us: American English, -s 160: Natural speaking speed
            subprocess.run(['espeak-ng', '-v', 'en-us', '-s', '160', text], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(0.15)
            if recognizer is not None:
                recognizer.Reset()
            self.is_speaking = False

        threading.Thread(target=run_speak, daemon=True).start()

    def initial_announcement(self):
        time.sleep(1.0)
        self.speak("Perception system ready. Awaiting voice command.")

    def audio_callback(self, indata, frames, time_info, status):
        # Ignore microphone input while the robot is actively speaking
        if self.running and not self.is_speaking:
            self.audio_queue.put(bytes(indata))

    def extract_number(self, text):
        digits = re.findall(r'\b\d+\b', text)
        if digits:
            return int(digits[0])
        for word, val in sorted(WORD_TO_NUM.items(), key=lambda x: -len(x[0])):
            if re.search(rf'\b{word}\b', text):
                return val
        return None

    def parse_command(self, text, recognizer):
        text = text.lower().strip()
        if not text:
            return

        self.get_logger().info(f"? Heard: '{text}'")

        # 1. STOP COMMAND
        if any(w in text for w in ["stop", "halt", "pause", "wait", "freeze"]):
            self.get_logger().info("? Action: STOP (-1)")
            self.publisher.publish(Int16(data=-1))
            self.speak("Tracking halted.", recognizer)
            return

        # 2. TARGET ID COMMAND
        num = self.extract_number(text)
        if num is not None:
            self.get_logger().info(f"? Action: Switch to Target ID {num}")
            self.publisher.publish(Int16(data=num))
            self.speak(f"Tracking target {num}.", recognizer)
            return

        # 3. RESUME COMMAND
        if any(w in text for w in ["resume", "continue", "start", "keep going"]):
            self.get_logger().info("?? Action: RESUME (-2)")
            self.publisher.publish(Int16(data=-2))
            self.speak("Resuming tracking.", recognizer)
            return

        # 4. WAKE WORD ONLY ("Robot", "Hey robot", "Hello robot")
        if any(w in text for w in ["robot", "hello", "hey", "wake up"]):
            self.get_logger().info("? Wake word recognized.")
            self.speak("I am listening.", recognizer)
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
