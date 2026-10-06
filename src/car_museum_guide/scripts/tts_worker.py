#!/usr/bin/env python3
"""
TTS Worker using gTTS
"""

import sys
import os
import tempfile
import time
import hashlib
import subprocess
from typing import Optional, Dict
from pathlib import Path

# Constants
DEFAULT_LANGUAGE = 'de'
DEFAULT_TTS_SPEED = 'normal'
DEFAULT_VOLUME_BOOST = 4
DEFAULT_SYSTEM_VOLUME = '80'
DEFAULT_CACHE_SIZE = 50
DEFAULT_CACHE_TEXT_LENGTH = 150
MPG123_GAIN_MAX = 10
MPG123_GAIN_MIN = 0
VOLUME_BOOST_MULTIPLIER = 2
TTS_TIMEOUT_SECONDS = 30
READY_SIGNAL = "READY"
DONE_SIGNAL = "DONE"
SHUTDOWN_SIGNAL = "SHUTDOWN"


class TTSWorkerError(Exception):
    """Custom exception for TTS worker errors"""
    pass


class TextSanitizer:
    """Utility class for text sanitization"""
    
    @staticmethod
    def sanitize(text: str) -> str:
        """
        Remove problematic characters from text for TTS processing.
        
        Args:
            text: Input text string
            
        Returns:
            Sanitized text string
        """
        if not text:
            return ""
        
        # Remove various quotation marks and special characters
        replacements = {
            '"': '', '"': '', '"': '',
            '«': '', '»': '',
            ''': '', ''': '',
            "'": '', '`': '', '~': '',
            '…': '...'
        }
        
        for old, new in replacements.items():
            text = text.replace(old, new)
        
        return text.strip()


class AudioCache:
    """Simple LRU-like cache for audio files"""
    
    def __init__(self, max_size: int = DEFAULT_CACHE_SIZE, max_text_length: int = DEFAULT_CACHE_TEXT_LENGTH):
        """
        Initialize audio cache.
        
        Args:
            max_size: Maximum number of cached audio files
            max_text_length: Maximum text length to cache
        """
        self._cache: Dict[str, str] = {}
        self._max_size = max_size
        self._max_text_length = max_text_length
    
    def get(self, text_hash: str) -> Optional[str]:
        """
        Retrieve cached audio file path.
        
        Args:
            text_hash: MD5 hash of text
            
        Returns:
            File path or None if not cached
        """
        return self._cache.get(text_hash)
    
    def add(self, text_hash: str, file_path: str, text_length: int) -> bool:
        """
        Add audio file to cache if conditions are met.
        
        Args:
            text_hash: MD5 hash of text
            file_path: Path to audio file
            text_length: Length of original text
            
        Returns:
            True if cached, False otherwise
        """
        if text_length >= self._max_text_length:
            return False
        
        if len(self._cache) >= self._max_size:
            return False
        
        self._cache[text_hash] = file_path
        return True
    
    def clear(self) -> None:
        """Clear cache and delete all cached files"""
        for mp3_file in self._cache.values():
            try:
                os.unlink(mp3_file)
            except OSError:
                pass
        self._cache.clear()
    
    def __contains__(self, text_hash: str) -> bool:
        """Check if text is cached"""
        return text_hash in self._cache


class AudioPlayer:
    """Handles audio playback using mpg123"""
    
    def __init__(self, gain: int, audio_device: str = 'hw:1,0'):
        """
        Initialize audio player.
        
        Args:
            gain: Audio gain (0-10)
            audio_device: ALSA device identifier
        """
        self._gain = max(MPG123_GAIN_MIN, min(MPG123_GAIN_MAX, gain))
        self._audio_device = audio_device
        self._validate_mpg123()
    
    @staticmethod
    def _validate_mpg123() -> None:
        """Validate that mpg123 is available"""
        try:
            result = subprocess.run(
                ['mpg123', '--version'],
                capture_output=True,
                timeout=2,
                check=False
            )
            if result.returncode != 0:
                raise TTSWorkerError("mpg123 returned error code")
        except (subprocess.TimeoutExpired, FileNotFoundError) as e:
            raise TTSWorkerError(f"mpg123 not available: {e}")
    
    def play(self, mp3_file: str) -> bool:
        """
        Play audio file.
        
        Args:
            mp3_file: Path to MP3 file
            
        Returns:
            True if playback successful, False otherwise
        """
        try:
            cmd = [
                'mpg123',
                '-q',
                '--gain', str(self._gain),
                '-a', self._audio_device,
                mp3_file
            ]
            result = subprocess.run(
                cmd,
                capture_output=True,
                timeout=TTS_TIMEOUT_SECONDS,
                check=False
            )
            return result.returncode == 0
        except subprocess.TimeoutExpired:
            print(f"ERROR: Playback timeout after {TTS_TIMEOUT_SECONDS}s", 
                  file=sys.stderr, flush=True)
            return False
        except Exception as e:
            print(f"ERROR: Playback failed: {e}", file=sys.stderr, flush=True)
            return False


class SystemVolumeController:
    """Controls system volume using amixer"""
    
    @staticmethod
    def set_volume(volume_percent: str) -> None:
        """
        Set system volume.
        
        Args:
            volume_percent: Volume level as percentage string (e.g., '80')
        """
        commands = [
            ['amixer', 'sset', 'Master', f'{volume_percent}%'],
            ['amixer', 'sset', 'PCM', f'{volume_percent}%'],
            ['amixer', '-c', '1', 'sset', 'Speaker', f'{volume_percent}%']
        ]
        
        for cmd in commands:
            try:
                subprocess.run(cmd, capture_output=True, timeout=1, check=False)
            except (subprocess.TimeoutExpired, FileNotFoundError):
                pass


class GTTSSynthesizer:
    """Handles text-to-speech synthesis using gTTS"""
    
    def __init__(self, language: str = DEFAULT_LANGUAGE, slow: bool = False):
        """
        Initialize TTS synthesizer.
        
        Args:
            language: Language code (e.g., 'de' for German)
            slow: Whether to use slow speech rate
        """
        try:
            from gtts import gTTS
            self._gtts_class = gTTS
        except ImportError as e:
            raise TTSWorkerError(f"gTTS not available: {e}")
        
        self._language = language
        self._slow = slow
    
    def synthesize(self, text: str, output_file: str) -> None:
        """
        Synthesize text to audio file.
        
        Args:
            text: Text to synthesize
            output_file: Path to output MP3 file
            
        Raises:
            TTSWorkerError: If synthesis fails
        """
        try:
            tts = self._gtts_class(text=text, lang=self._language, slow=self._slow)
            tts.save(output_file)
        except Exception as e:
            raise TTSWorkerError(f"Synthesis failed: {e}")


class TTSWorker:
    """Main TTS worker class"""
    
    def __init__(self):
        """Initialize TTS worker with configuration from environment"""
        self._load_environment()
        self._init_components()
        self._audio_cache = AudioCache()
    
    def _load_environment(self) -> None:
        """Load configuration from environment variables"""
        try:
            from dotenv import load_dotenv
            env_path = Path(__file__).resolve().parents[3] / '.env'
            if env_path.exists():
                load_dotenv(env_path)
        except ImportError:
            pass
        
        self._language = os.getenv('LANGUAGE', DEFAULT_LANGUAGE)
        self._tts_speed = os.getenv('TTS_SPEED', DEFAULT_TTS_SPEED)
        self._volume_boost = int(os.getenv('TTS_VOLUME_BOOST', str(DEFAULT_VOLUME_BOOST)))
        self._system_volume = os.getenv('SYSTEM_VOLUME', DEFAULT_SYSTEM_VOLUME)
    
    def _init_components(self) -> None:
        """Initialize TTS components"""
        # Calculate mpg123 gain
        mpg123_gain = min(
            MPG123_GAIN_MAX,
            max(MPG123_GAIN_MIN, int(self._volume_boost * VOLUME_BOOST_MULTIPLIER))
        )
        
        # Initialize components
        self._player = AudioPlayer(gain=mpg123_gain)
        self._synthesizer = GTTSSynthesizer(
            language=self._language,
            slow=(self._tts_speed == 'slow')
        )
        
        # Set system volume
        SystemVolumeController.set_volume(self._system_volume)
        
        # Log configuration
        print(f"LOG: Language: {self._language}", file=sys.stderr, flush=True)
        print(f"LOG: Speed: {self._tts_speed}", file=sys.stderr, flush=True)
        print(f"LOG: Volume gain: {mpg123_gain}/{MPG123_GAIN_MAX}", file=sys.stderr, flush=True)
        print(f"LOG: System volume: {self._system_volume}%", file=sys.stderr, flush=True)
    
    def _process_text(self, text: str) -> None:
        """
        Process and speak text.
        
        Args:
            text: Text to process
        """
        start_time = time.time()
        
        # Sanitize text
        text = TextSanitizer.sanitize(text)
        if not text:
            print("LOG: Empty text after sanitization", file=sys.stderr, flush=True)
            return
        
        # Check cache
        text_hash = hashlib.md5(text.encode()).hexdigest()
        
        if text_hash in self._audio_cache:
            print(f"LOG: Cache hit ({len(text)} chars)", file=sys.stderr, flush=True)
            mp3_file = self._audio_cache.get(text_hash)
            success = self._player.play(mp3_file)
            
            if not success:
                print("ERROR: Playback failed", file=sys.stderr, flush=True)
            
            total_time = time.time() - start_time
            print(f"LOG: Total: {total_time:.1f}s (cached)", file=sys.stderr, flush=True)
            return
        
        # Generate new audio
        print(f"LOG: Synthesizing {len(text)} chars with gTTS...", file=sys.stderr, flush=True)
        
        with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as f:
            tmp_mp3 = f.name
        
        try:
            self._synthesizer.synthesize(text, tmp_mp3)
            synth_time = time.time() - start_time
            print(f"LOG: Synth: {synth_time:.2f}s", file=sys.stderr, flush=True)
        except TTSWorkerError as e:
            print(f"ERROR: {e}", file=sys.stderr, flush=True)
            try:
                os.unlink(tmp_mp3)
            except OSError:
                pass
            return
        
        # Cache if appropriate
        cached = self._audio_cache.add(text_hash, tmp_mp3, len(text))
        if cached:
            print("LOG: Cached", file=sys.stderr, flush=True)
        
        # Play audio
        play_start = time.time()
        success = self._player.play(tmp_mp3)
        
        if not success:
            print("ERROR: Playback failed", file=sys.stderr, flush=True)
        
        play_time = time.time() - play_start
        
        # Cleanup if not cached
        if not cached:
            try:
                os.unlink(tmp_mp3)
            except OSError:
                pass
        
        total_time = time.time() - start_time
        print(f"LOG: Total: {total_time:.1f}s (synth: {synth_time:.2f}s, play: {play_time:.1f}s)",
              file=sys.stderr, flush=True)
    
    def run(self) -> None:
        """Main event loop"""
        # Redirect stdout to stderr for logging
        original_stdout = sys.stdout
        sys.stdout = sys.stderr
        
        print("LOG: Initializing gTTS TTS Worker...", file=sys.stderr, flush=True)
        
        try:
            # Signal ready
            sys.stdout = original_stdout
            print(READY_SIGNAL, flush=True)
            
            # Main loop
            while True:
                line = sys.stdin.readline()
                
                if not line:
                    break
                
                text = line.strip()
                
                if not text:
                    continue
                
                if text == SHUTDOWN_SIGNAL:
                    print("LOG: Shutdown requested", file=sys.stderr, flush=True)
                    break
                
                try:
                    self._process_text(text)
                    print(DONE_SIGNAL, flush=True)
                except Exception as e:
                    print(f"ERROR: {e}", file=sys.stderr, flush=True)
                    import traceback
                    traceback.print_exc(file=sys.stderr)
                    print(DONE_SIGNAL, flush=True)
            
            # Cleanup
            print("LOG: Cleaning up cache...", file=sys.stderr, flush=True)
            self._audio_cache.clear()
            print("LOG: Shutting down TTS worker", file=sys.stderr, flush=True)
        
        except KeyboardInterrupt:
            print("LOG: Interrupt", file=sys.stderr, flush=True)
        except Exception as e:
            print(f"FATAL: {e}", file=sys.stderr, flush=True)
            import traceback
            traceback.print_exc(file=sys.stderr)
            sys.exit(1)
        finally:
            sys.stdout = original_stdout


def main() -> None:
    """Entry point"""
    try:
        worker = TTSWorker()
        worker.run()
    except TTSWorkerError as e:
        print(f"FATAL: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
