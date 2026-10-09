import openwakeword
from openwakeword.model import Model

import pyaudio
import numpy as np
from openwakeword.model import Model
import argparse




FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = 1280
audio = pyaudio.PyAudio()
mic_stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)

model = Model(
    wakeword_models=["/home/ubuntu/.venv/turtlebot_venv/lib/python3.12/site-packages/openwakeword/resources/models/hey_mycroft_v0.1.onnx"],
    inference_framework="onnx",
)

n_models = len(model.models.keys())

# Run capture loop continuosly, checking for wakewords
if __name__ == "__main__":
    # Generate output string header
    print("\n\n")
    print("#"*100)
    print("Listening for wakewords...")
    print("#"*100)
    print("\n"*(n_models*3))

    while True:
        # Get audio
        audio = np.frombuffer(mic_stream.read(CHUNK, exception_on_overflow=False), dtype=np.int16)

        # Feed to openWakeWord model
        prediction = model.predict(audio)

        # Column titles
        n_spaces = 16
        output_string_header = """
            Model Name         | Score | Wakeword Status
            --------------------------------------
            """

        for mdl in model.prediction_buffer.keys():
            # Add scores in formatted table
            scores = list(model.prediction_buffer[mdl])
            curr_score = format(scores[-1], '.20f').replace("-", "")

            output_string_header += f"""{mdl}{" "*(n_spaces - len(mdl))}   | {curr_score[0:5]} | {"--"+" "*20 if scores[-1] <= 0.5 else "Wakeword Detected!"}
            """

        # Print results table
        print("\033[F"*(4*n_models+1))
        print(output_string_header, "                             ", end='\r')