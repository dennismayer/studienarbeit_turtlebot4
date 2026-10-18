from pathlib import Path
from collections import OrderedDict
import soundfile as sf
import os
import hashlib
from threading import Lock
import functools
from rclpy.impl.rcutils_logger import RcutilsLogger



class CacheError(Exception):
    pass


def _lock_function(func):
    @functools.wraps(func)
    def inner_func(self, *args, **kwargs):
        with self.dict_lock:
            return func(self, *args, **kwargs)
    return inner_func


class FileCache:
    def __init__(self, audio_dir: Path, logger: RcutilsLogger, cache_size: int = 30):
        # create dictionary
        self.dict = OrderedDict()
        self.dict_lock = Lock()
        self.cache_size = cache_size
        self.audio_dir = audio_dir
        self.logger = logger
        
        if not os.path.isdir(self.audio_dir):
            raise CacheError(f"No directory: '{self.audio_dir}'")
        
        list_file_names = os.listdir(self.audio_dir)
        
        unsorted_list = []
        
        for file_name in list_file_names:
            if file_name.endswith(".wav"):
                mtime = os.path.getmtime(self.audio_dir / file_name)
                
                try:
                    duration = sf.info(audio_dir / file_name).duration
                except sf.LibsndfileError as e:
                    self.logger.warning(f"Deleting unreadable cache file '{file_name}': {e}")
                    (self.audio_dir / file_name).unlink()
                    continue

                if duration == 0.0:
                    self.logger.warning(f"Deleting empty cache file '{file_name}'")
                    (self.audio_dir / file_name).unlink()
                    continue
                    
                key = file_name[:-4]
                unsorted_list.append({"key": key, "mtime": mtime, "duration": duration})
            else:
                try:
                    (self.audio_dir / file_name).unlink()
                    self.logger.info(f"Deleted non-wav file '{file_name}' from cache directory")
                except IsADirectoryError:
                    self.logger.warning(f"Ignoring directory '{file_name}' in cache directory")

        unsorted_list.sort(key=lambda entry: entry["mtime"])

        for entry in unsorted_list:
            self.dict[entry["key"]] = {"duration": entry["duration"]}

        self.__remove_entries_over_threshold()
        self.logger.info(f"Cache loaded with {len(self.dict)}/{self.cache_size} entries from '{self.audio_dir}'")
    
    @_lock_function
    def lookup(self, text: str):
        key = FileCache.get_hash(text)
        
        if key not in self.dict:
            self.logger.debug(f"Cache miss: {key}")
            return None
        
        file_path = self.audio_dir / (key + ".wav")

        if not os.path.exists(file_path):
            self.logger.warning(f"Cache entry '{key}' has no file on disk, removing entry")
            self.dict.pop(key)
            return None

        file_path.touch()
        self.logger.debug(f"Cache hit: {key}")

        entry = self.dict[key]
        self.dict.move_to_end(key)
        return {"file_path": str(file_path), "duration": entry["duration"]}
    
    @_lock_function
    def store(self, file_name: str, duration = None):
        file_path = self.audio_dir / file_name
        
        # check if file exists
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"No file in cache with name: {file_name}")
        
        # check if wav file
        if not file_name.endswith(".wav"):
            raise ValueError(f"Not a .wav file: {file_name}")
        
        key = file_name[:-4]
        # check if already exists
        if key in self.dict:
            self.logger.debug(f"Entry '{key}' already cached, marking as recently used")
            self.dict.move_to_end(key)
            return
        
        try:
            if duration is None:
                duration = sf.info(file_path).duration
        except sf.LibsndfileError as e:
            file_path.unlink(missing_ok=True)
            raise CacheError(f"Header information not valid!") from e
        
        if duration == 0.0:
            file_path.unlink(missing_ok=True)
            raise CacheError("File empty!")
        
        self.dict[key] = {"duration": duration}

        self.__remove_entries_over_threshold()
        self.logger.debug(f"Stored '{key}' ({duration:.2f} s), {len(self.dict)}/{self.cache_size} entries")
        
    @staticmethod
    def get_hash(text: str):
        text = text.strip()
        return hashlib.sha256(text.encode()).hexdigest()
        
    def __remove_entries_over_threshold(self):
        while len(self.dict) > self.cache_size:
            key,  _ = self.dict.popitem(last=False)
            key: str = key
            file_path = self.audio_dir / (key + ".wav")
            self.logger.info(f"Cache full, evicting least recently used entry '{key}'")
            try:
                file_path.unlink(missing_ok=True)
            except Exception as e:
                raise CacheError(f"Error while removing old entries: {e}")