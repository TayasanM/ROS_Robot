#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import json
import queue
import threading
import signal
import subprocess
import time
import sounddevice as sd
from vosk import Model, KaldiRecognizer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

OBJECT_KEYWORDS = [
    'person', 'bottle', 'cup', 'chair', 'backpack',
    'laptop', 'cell phone', 'book', 'tv', 'suitcase',
    'apple', 'orange', 'banana', 'bowl', 'potted plant'
]


class VoiceMissionNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')
        self.mission_pub = self.create_publisher(String, '/voice_mission_cmd', 10)
        self.create_subscription(String, '/speech_tts', self.tts_callback, 10)

        model_path = os.path.expanduser('~/.vosk_model')
        if not os.path.exists(model_path):
            self.get_logger().error(f"Vosk model not found at {model_path}!")
            sys.exit(1)

        self.model = Model(model_path)
        self.audio_queue = queue.Queue()
        self.running = True
        self.is_speaking = False
        self.stream = None
        self.recognizer = None

        threading.Thread(target=self.audio_processing_loop, daemon=True).start()
        threading.Thread(target=self.initial_announcement, daemon=True).start()

        self.get_logger().info("Offline Voice Mission Ready:")
        self.get_logger().info(" -> 'Go to room one and find bottle'")
        self.get_logger().info(" -> 'Go to kitchen and check what is there'")
        self.get_logger().info(" -> 'Return to charging position' / 'Stop'")

    def tts_callback(self, msg: String):
        if msg.data:
            self.speak(msg.data, self.recognizer)

    def speak(self, text, recognizer=None):
        def run_speak():
            self.is_speaking = True
            while not self.audio_queue.empty():
                try:
                    self.audio_queue.get_nowait()
                except queue.Empty:
                    break
            subprocess.run(
                ['espeak-ng', '-v', 'en-us', '-s', '155', text],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            time.sleep(0.20)
            if recognizer is not None:
                recognizer.Reset()
            self.is_speaking = False

        threading.Thread(target=run_speak, daemon=True).start()

    def initial_announcement(self):
        time.sleep(1.0)
        self.speak("Mission system ready. Awaiting voice command.")

    def audio_callback(self, indata, frames, time_info, status):
        if self.running and not self.is_speaking:
            self.audio_queue.put(bytes(indata))

    def extract_object(self, text):
        for obj in OBJECT_KEYWORDS:
            if obj in text:
                return obj
        if any(w in text for w in ['human', 'man', 'woman', 'someone']):
            return 'person'
        return 'any'

    def parse_command(self, text, recognizer):
        text = text.lower().strip()
        if not text:
            return
        self.get_logger().info(f"Heard: '{text}'")

        # 1. STOP
        if any(w in text for w in ['stop', 'halt', 'cancel', 'freeze']):
            self.mission_pub.publish(String(data='STOP'))
            self.speak("Mission halted.", recognizer)
            return

        # 2. RETURN TO CHARGING POSITION / HOME
        if any(w in text for w in ['charging', 'charger', 'home', 'base']):
            self.mission_pub.publish(String(data='RETURN_HOME'))
            self.speak("Returning to charging position.", recognizer)
            return

        # 3. GO TO ROOM1
        if any(w in text for w in ['room one', 'room 1', 'room']):
            target_obj = self.extract_object(text)
            self.mission_pub.publish(String(data=f'MISSION:room1:{target_obj}'))
            if target_obj == 'any':
                self.speak("Remembering current position. Going to room one to inspect.", recognizer)
            else:
                self.speak(f"Remembering current position. Going to room one to find {target_obj}.", recognizer)
            return

        # 4. GO TO KITCHEN
        if 'kitchen' in text:
            target_obj = self.extract_object(text)
            self.mission_pub.publish(String(data=f'MISSION:kitchen:{target_obj}'))
            if target_obj == 'any':
                self.speak("Remembering current position. Going to kitchen to inspect.", recognizer)
            else:
                self.speak(f"Remembering current position. Going to kitchen to find {target_obj}.", recognizer)
            return

        # 5. WAKE WORD
        if any(w in text for w in ['robot', 'hello', 'hey']):
            self.speak("I am listening.", recognizer)

    def audio_processing_loop(self):
        sample_rate = 16000
        self.recognizer = KaldiRecognizer(self.model, sample_rate)
        try:
            self.stream = sd.RawInputStream(
                samplerate=sample_rate, blocksize=4000,
                dtype='int16', channels=1, callback=self.audio_callback
            )
            with self.stream:
                while self.running and rclpy.ok():
                    try:
                        data = self.audio_queue.get(timeout=0.2)
                    except queue.Empty:
                        continue
                    if self.recognizer.AcceptWaveform(data):
                        res = json.loads(self.recognizer.Result())
                        spoken_text = res.get("text", "")
                        if spoken_text:
                            self.parse_command(spoken_text, self.recognizer)
        except Exception as e:
            if self.running:
                self.get_logger().error(f"Audio stream error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = VoiceMissionNode()

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
