import time
from pocketsphinx import LiveSpeech, get_model_path
import os

# Set up the decoder
model_path = get_model_path()

speech = LiveSpeech(
    verbose=False,
    sampling_rate=16000,
    buffer_size=2048,
    no_search=False,
    full_utt=False,
    hmm=os.path.join(model_path, 'en-us'),
    lm=os.path.join(model_path, 'en-us.lm.bin'),
    dict=os.path.join(model_path, 'demo.dict'),
    jsgf=os.path.join(model_path, 'confirmation.jsgf')
)

# Loop through the audio input and decode speech
for phrase in speech:
    print(phrase)

    # Add a delay to give the decoder more time to process audio input
    time.sleep(0.5)
