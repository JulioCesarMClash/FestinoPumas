#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool
import pyaudio
from vosk import Model, KaldiRecognizer
import json
from vosk_speech_recognition.srv import speech_recog, speech_recogResponse  

class VoskSpeechRecognizer:
    def __init__(self):
        rospy.init_node('vosk_speech_recognizer', anonymous=True)
        
        self.language = rospy.get_param('~language', 'en')  
        self.model_path = rospy.get_param('~model_path', 'model') 
        self.sample_rate = rospy.get_param('~sample_rate', 16000)
        self.service_name = rospy.get_param('~speech_service', 'speech_recognition')
        
        try:
            self.model = Model(self.model_path)
        except:
            rospy.logerr(f"Error loading model from {self.model_path}")
            rospy.signal_shutdown("Error Vosk Model")
            return
        
        self.recognizer = KaldiRecognizer(self.model, self.sample_rate)
        self.recognizer.SetWords(True) 
        
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=4096,
            input_device_index=None
        )
        
        self.service = rospy.Service(self.service_name, speech_recog, self.handle_speech_service)
        rospy.loginfo("Speech recognition service ready.")
    
    def handle_speech_service(self, req):
        if req.is_speecg_recog_enabled:
            rospy.loginfo("Speech recognition enabled.")
            result_text = self.recognize_speech()
            return speech_recogResponse(success=True, text_recog=result_text)
        else:
            rospy.loginfo("Speech recognition disabled.")
            return speech_recogResponse(success=True, text_recog="")
    
    def recognize_speech(self):
        rospy.loginfo("Listening...")
        while not rospy.is_shutdown():
            data = self.stream.read(4096, exception_on_overflow=False)
            
            if self.recognizer.AcceptWaveform(data):
                result = json.loads(self.recognizer.Result())
                if 'text' in result and result['text']:
                    rospy.loginfo(f"Recognized: {result['text']}")
                    return result['text']
    
    def shutdown(self):
        rospy.loginfo("Closing speech recognition...")
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()

if __name__ == '__main__':
    try:
        recognizer = VoskSpeechRecognizer()
        rospy.on_shutdown(recognizer.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
