#!/usr/bin/env python3

import speech_recognition as sr
import numpy as np
import soundfile as sf
import tempfile
import os

def transcribe_spanish(audio_buffer, sample_rate=16000, processing_method='frombuffer'):
    """
    Transcribe audio buffer to Spanish text using Google Cloud Speech Recognition
    
    Args:
        audio_buffer (list or numpy.array): Audio data buffer
        sample_rate (int): Sample rate of the audio (default: 16000 Hz)
        processing_method (str): 'frombuffer' or 'array'
    
    Returns:
        str: Transcribed text in Spanish, or "None" if transcription fails
    """
    # Initialize Google recognizer
    recognizer = sr.Recognizer()
    
    if processing_method == 'frombuffer':
        # Convert buffer of bytes into a numpy array of 16-bit integers
        audio_array = np.frombuffer(bytes(audio_buffer), dtype=np.int16)
    elif processing_method == 'array':
        # Convert buffer to numpy array if needed (original incorrect method)
        audio_array = np.array(audio_buffer)
        audio_array = audio_array.astype(float)
        audio_array = np.asarray(audio_array, dtype=np.int16)
    else:
        raise ValueError("Invalid processing_method. Choose 'frombuffer' or 'array'")
    
    # Create temporary WAV file
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_audio:
        temp_path = temp_audio.name
        sf.write(temp_path, audio_array, sample_rate)
    
    transcription = "None"
    
    try:
        # Load audio file and transcribe
        with sr.AudioFile(temp_path) as source:
            audio = recognizer.record(source)
            
        # Transcribe using Google Speech Recognition with Spanish language
        transcription = recognizer.recognize_google(audio, language="es-ES")
        
    except sr.UnknownValueError:
        print("Google Speech Recognition no pudo entender el audio")
        transcription = "None"
        
    except sr.RequestError as e:
        print(f"Error en el servicio de Google Speech Recognition: {e}")
        transcription = "None"
        
    finally:
        # Clean up temporary file
        #if os.path.exists(temp_path):
        #    os.remove(temp_path)
        pass
    
    return transcription
