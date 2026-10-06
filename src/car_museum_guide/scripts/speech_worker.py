#!/usr/bin/env python3
"""
Speech Recognition Worker Process
"""

import sys
import os
import struct
import time
from pathlib import Path
from typing import Optional, List
from dataclasses import dataclass

# Constants
DEFAULT_WHISPER_MODEL = 'tiny'
DEFAULT_WHISPER_DEVICE = 'cpu'
DEFAULT_SAMPLE_RATE = 16000
DEFAULT_RECORDING_DURATION = 5.0
DEFAULT_WAKE_RECORDING_DURATION = 6.0
DEFAULT_ENERGY_THRESHOLD = 300
DEFAULT_MIN_CONFIDENCE = 0.5
DEFAULT_WAKE_WORD = 'computer'
DEFAULT_WAKE_WORD_SENSITIVITY = 0.5
SUPPORTED_WAKE_WORDS = ['computer', 'alexa', 'jarvis', 'americano', 'blueberry', 
                        'bumblebee', 'grapefruit', 'grasshopper', 'hey google', 
                        'hey siri', 'ok google', 'picovoice', 'porcupine', 'terminator']
READY_SIGNAL = "READY"
WAKE_EVENT = "EVENT:WAKE"
COMMAND_EVENT_PREFIX = "EVENT:COMMAND:"


class SpeechWorkerError(Exception):
    """Custom exception for speech worker errors"""
    pass


@dataclass
class SpeechConfig:
    """Configuration container for speech recognition"""
    porcupine_key: str
    whisper_model: str
    whisper_device: str
    wake_word_enabled: bool
    wake_word: str
    wake_word_sensitivity: float
    energy_threshold: int
    min_confidence: float
    sample_rate: int = DEFAULT_SAMPLE_RATE
    recording_duration: float = DEFAULT_RECORDING_DURATION
    wake_recording_duration: float = DEFAULT_WAKE_RECORDING_DURATION
    
    def __post_init__(self):
        """Validate configuration"""
        if self.wake_word_enabled and not self.porcupine_key:
            raise ValueError("Wake word enabled but no Porcupine key provided")
        
        if self.wake_word not in SUPPORTED_WAKE_WORDS:
            print(f"WARN: Wake word '{self.wake_word}' not supported", file=sys.stderr)
            print(f"WARN: Using 'computer' instead", file=sys.stderr)
            self.wake_word = DEFAULT_WAKE_WORD
        
        if not 0.0 <= self.wake_word_sensitivity <= 1.0:
            raise ValueError("Wake word sensitivity must be in [0, 1]")
        
        if not 0.0 <= self.min_confidence <= 1.0:
            raise ValueError("Min confidence must be in [0, 1]")


class ConfigLoader:
    """Loads configuration from environment"""
    
    @staticmethod
    def load() -> SpeechConfig:
        """
        Load configuration from environment variables.
        
        Returns:
            SpeechConfig instance
        """
        # Try to load .env file
        try:
            from dotenv import load_dotenv
            env_path = Path(__file__).resolve().parents[3] / '.env'
            if env_path.exists():
                load_dotenv(env_path)
                print(f"LOG: Loaded .env from {env_path}", file=sys.stderr, flush=True)
        except ImportError:
            pass
        
        return SpeechConfig(
            porcupine_key=os.getenv('PORCUPINE_ACCESS_KEY', ''),
            whisper_model=os.getenv('WHISPER_MODEL', DEFAULT_WHISPER_MODEL),
            whisper_device=os.getenv('WHISPER_DEVICE', DEFAULT_WHISPER_DEVICE),
            wake_word_enabled=os.getenv('WAKE_WORD_ENABLED', 'true').lower() == 'true',
            wake_word=os.getenv('WAKE_WORD', DEFAULT_WAKE_WORD),
            wake_word_sensitivity=float(os.getenv('WAKE_WORD_SENSITIVITY', str(DEFAULT_WAKE_WORD_SENSITIVITY))),
            energy_threshold=int(os.getenv('ENERGY_THRESHOLD', str(DEFAULT_ENERGY_THRESHOLD))),
            min_confidence=float(os.getenv('MIN_CONFIDENCE', str(DEFAULT_MIN_CONFIDENCE)))
        )


class WhisperTranscriber:
    """Handles speech-to-text using Faster Whisper"""
    
    def __init__(self, model_name: str, device: str):
        """
        Initialize Whisper transcriber.
        
        Args:
            model_name: Whisper model name (e.g., 'tiny', 'base')
            device: Device to run on ('cpu' or 'cuda')
        """
        try:
            from faster_whisper import WhisperModel
        except ImportError as e:
            raise SpeechWorkerError(f"Faster Whisper not available: {e}")
        
        print("LOG: Lade Whisper-Modell für Deutsch...", file=sys.stderr, flush=True)
        
        try:
            compute_type = 'int8' if device == 'cpu' else None
            self._model = WhisperModel(model_name, device=device, compute_type=compute_type)
            print(f"LOG: Whisper '{model_name}' geladen auf {device}", 
                  file=sys.stderr, flush=True)
        except Exception as e:
            raise SpeechWorkerError(f"Failed to load Whisper: {e}")
    
    def transcribe(self, audio_file: str) -> tuple[str, float]:
        """
        Transcribe audio file to text.
        
        Args:
            audio_file: Path to audio file
            
        Returns:
            Tuple of (transcribed_text, confidence)
        """
        try:
            segments, info = self._model.transcribe(
                audio_file,
                beam_size=5,
                language='de',
                condition_on_previous_text=False
            )
            
            text = ' '.join([seg.text for seg in segments]).strip()
            confidence = getattr(info, 'language_probability', 1.0)
            
            return text, confidence
        except Exception as e:
            raise SpeechWorkerError(f"Transcription failed: {e}")


class WakeWordDetector:
    """Handles wake word detection using Porcupine"""
    
    def __init__(self, access_key: str, keyword: str, sensitivity: float):
        """
        Initialize wake word detector.
        
        Args:
            access_key: Porcupine access key
            keyword: Wake word to detect
            sensitivity: Detection sensitivity (0-1)
        """
        try:
            import pvporcupine
            import pyaudio
        except ImportError as e:
            raise SpeechWorkerError(f"Porcupine dependencies not available: {e}")
        
        print("LOG: Initialisiere Weckwort-Erkennung...", file=sys.stderr, flush=True)
        
        try:
            self._porcupine = pvporcupine.create(
                access_key=access_key,
                keywords=[keyword],
                sensitivities=[sensitivity]
            )
            
            self._pa = pyaudio.PyAudio()
            self._stream = self._pa.open(
                rate=self._porcupine.sample_rate,
                channels=1,
                format=pyaudio.paInt16,
                input=True,
                frames_per_buffer=self._porcupine.frame_length
            )
            
            print(f"LOG: Weckwort-Erkennung aktiv (Wort: '{keyword}')", 
                  file=sys.stderr, flush=True)
        except Exception as e:
            raise SpeechWorkerError(f"Failed to initialize Porcupine: {e}")
    
    def detect(self) -> bool:
        """
        Check for wake word in audio stream.
        
        Returns:
            True if wake word detected, False otherwise
        """
        try:
            pcm = self._stream.read(self._porcupine.frame_length, exception_on_overflow=False)
            pcm = struct.unpack_from("h" * self._porcupine.frame_length, pcm)
            keyword_index = self._porcupine.process(pcm)
            return keyword_index >= 0
        except Exception as e:
            print(f"ERROR: Wake word detection error: {e}", file=sys.stderr, flush=True)
            return False
    
    def pause(self) -> None:
        """Pause audio stream"""
        if self._stream:
            self._stream.stop_stream()
    
    def resume(self) -> None:
        """Resume audio stream"""
        if self._stream:
            self._stream.start_stream()
    
    def cleanup(self) -> None:
        """Clean up resources"""
        if hasattr(self, '_stream') and self._stream:
            try:
                self._stream.close()
            except Exception:
                pass
        
        if hasattr(self, '_porcupine') and self._porcupine:
            try:
                self._porcupine.delete()
            except Exception:
                pass
        
        if hasattr(self, '_pa') and self._pa:
            try:
                self._pa.terminate()
            except Exception:
                pass


class AudioRecorder:
    """Handles audio recording"""
    
    def __init__(self, sample_rate: int = DEFAULT_SAMPLE_RATE):
        """
        Initialize audio recorder.
        
        Args:
            sample_rate: Sample rate in Hz
        """
        try:
            import sounddevice as sd
            import soundfile as sf
            import numpy as np
            import tempfile
        except ImportError as e:
            raise SpeechWorkerError(f"Audio dependencies not available: {e}")
        
        self._sd = sd
        self._sf = sf
        self._np = np
        self._tempfile = tempfile
        self._sample_rate = sample_rate
    
    def record(self, duration: float) -> tuple[str, float]:
        """
        Record audio for specified duration.
        
        Args:
            duration: Recording duration in seconds
            
        Returns:
            Tuple of (temp_file_path, audio_energy)
        """
        print(f"LOG: Nehme auf für {duration}s...", file=sys.stderr, flush=True)
        
        # Record audio
        recording = self._sd.rec(
            int(duration * self._sample_rate),
            samplerate=self._sample_rate,
            channels=1,
            dtype='int16'
        )
        self._sd.wait()
        
        # Calculate energy
        audio_array = recording.flatten()
        energy = float(self._np.sqrt(self._np.mean(audio_array.astype(float)**2)))
        
        # Save to temporary file
        temp_file = self._tempfile.NamedTemporaryFile(suffix='.wav', delete=False)
        temp_path = temp_file.name
        temp_file.close()
        
        self._sf.write(temp_path, recording, self._sample_rate, subtype='PCM_16')
        
        return temp_path, energy


class SpeechRecognitionWorker:
    """Main speech recognition worker"""
    
    def __init__(self):
        """Initialize speech recognition worker"""
        self._config = ConfigLoader.load()
        self._transcriber = WhisperTranscriber(
            self._config.whisper_model,
            self._config.whisper_device
        )
        self._recorder = AudioRecorder(self._config.sample_rate)
        
        # Initialize wake word detector if enabled
        self._wake_detector: Optional[WakeWordDetector] = None
        if self._config.wake_word_enabled and self._config.porcupine_key:
            try:
                self._wake_detector = WakeWordDetector(
                    self._config.porcupine_key,
                    self._config.wake_word,
                    self._config.wake_word_sensitivity
                )
            except SpeechWorkerError as e:
                print(f"WARN: {e}", file=sys.stderr, flush=True)
                print("LOG: Fahre ohne Weckwort fort (Dauerhörmodus)", 
                      file=sys.stderr, flush=True)
        else:
            print("LOG: Weckwort deaktiviert - Dauerhörmodus", file=sys.stderr, flush=True)
        
        self._listening_for_command = False
    
    def _process_audio(self) -> None:
        """Process recorded audio and publish results"""
        # Determine recording duration
        duration = (self._config.wake_recording_duration 
                   if self._listening_for_command 
                   else self._config.recording_duration)
        
        # Record audio
        temp_path, energy = self._recorder.record(duration)
        
        # Check energy threshold
        if energy < self._config.energy_threshold:
            print(f"LOG: Audio-Energie zu niedrig ({energy:.1f} < {self._config.energy_threshold})",
                  file=sys.stderr, flush=True)
            
            # Reset state
            if self._listening_for_command:
                self._listening_for_command = False
                if self._wake_detector:
                    self._wake_detector.resume()
            
            # Cleanup
            try:
                os.unlink(temp_path)
            except OSError:
                pass
            
            if not self._wake_detector:
                time.sleep(0.2)
            
            return
        
        print(f"LOG: Audio-Energie: {energy:.1f}", file=sys.stderr, flush=True)
        print("LOG: Transkribiere Audio...", file=sys.stderr, flush=True)
        
        # Transcribe
        try:
            text, confidence = self._transcriber.transcribe(temp_path)
        except SpeechWorkerError as e:
            print(f"ERROR: {e}", file=sys.stderr, flush=True)
            text = ""
            confidence = 0.0
        finally:
            # Cleanup temp file
            try:
                os.unlink(temp_path)
            except OSError:
                pass
        
        # Check confidence
        print(f"LOG: Sprache: de (Konfidenz: {confidence:.2f})", file=sys.stderr, flush=True)
        
        if confidence < self._config.min_confidence:
            print(f"LOG: Niedrige Konfidenz, abgelehnt ({confidence:.2f} < {self._config.min_confidence})",
                  file=sys.stderr, flush=True)
            text = ""
        
        # Publish command if valid
        if text and len(text) > 2:
            print(f"LOG: Transkribiert: {text}", file=sys.stderr, flush=True)
            print(f"{COMMAND_EVENT_PREFIX}{text}", flush=True)
        else:
            print("LOG: Keine Sprache erkannt", file=sys.stderr, flush=True)
        
        # Reset state
        if self._listening_for_command:
            self._listening_for_command = False
            if self._wake_detector:
                self._wake_detector.resume()
        
        # Delay in continuous mode
        if not self._wake_detector:
            time.sleep(0.2)
    
    def run(self) -> None:
        """Main event loop"""
        print(READY_SIGNAL, flush=True)
        
        try:
            while True:
                # Wake word mode
                if self._wake_detector and not self._listening_for_command:
                    if self._wake_detector.detect():
                        print(f"LOG: Weckwort '{self._config.wake_word}' erkannt!", 
                              file=sys.stderr, flush=True)
                        print(WAKE_EVENT, flush=True)
                        self._listening_for_command = True
                        self._wake_detector.pause()
                
                # Command recording mode
                elif self._listening_for_command or not self._wake_detector:
                    try:
                        self._process_audio()
                    except Exception as e:
                        print(f"ERROR: Aufnahme-Fehler: {e}", file=sys.stderr, flush=True)
                        self._listening_for_command = False
                        if self._wake_detector:
                            try:
                                self._wake_detector.resume()
                            except Exception:
                                pass
                        time.sleep(0.5)
        
        except KeyboardInterrupt:
            print("LOG: Interrupt-Signal empfangen", file=sys.stderr, flush=True)
        except Exception as e:
            print(f"FATAL: Worker abgestürzt: {e}", file=sys.stderr, flush=True)
            import traceback
            traceback.print_exc(file=sys.stderr)
        finally:
            if self._wake_detector:
                self._wake_detector.cleanup()
            print("LOG: Sprach-Worker beendet", file=sys.stderr, flush=True)


def main() -> None:
    """Entry point"""
    try:
        worker = SpeechRecognitionWorker()
        worker.run()
    except SpeechWorkerError as e:
        print(f"FATAL: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
