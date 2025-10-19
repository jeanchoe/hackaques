cat > ~/pi-falldetector/notifier.py <<'PY'
import os
from time import sleep

# Optional Twilio
try:
    from twilio.rest import Client
except Exception:
    Client = None

class Notifier:
    def __init__(self, cfg):
        self.cfg = cfg
        ncfg = cfg.get("notifiers", {})
        self.voice = bool(ncfg.get("local_voice", True))
        self.siren = bool(ncfg.get("local_siren", True))
        self.sms = bool(ncfg.get("sms", False))
        self.call = bool(ncfg.get("call", False))

        # Twilio env vars (optional)
        self.tw_sid = os.getenv("TWILIO_ACCOUNT_SID")
        self.tw_tok = os.getenv("TWILIO_AUTH_TOKEN")
        self.tw_from = os.getenv("TWILIO_FROM")
        self.sms_to = os.getenv("ALERT_SMS_TO")
        self.call_to = os.getenv("ALERT_CALL_TO")

        self.tw_client = None
        if (self.sms or self.call) and Client and self.tw_sid and self.tw_tok:
            try:
                self.tw_client = Client(self.tw_sid, self.tw_tok)
            except Exception:
                self.tw_client = None

        # Try pyttsx3 first; if it fails we'll fall back to espeak-ng CLI
        self.tts = None
        if self.voice:
            try:
                import pyttsx3
                self.tts = pyttsx3.init(driverName='espeak')
                # Select a safe voice if available
                voices = self.tts.getProperty('voices')
                if voices:
                    self.tts.setProperty('voice', voices[0].id)
                self.tts.setProperty('rate', 175)
            except Exception:
                self.tts = None

    def _speak(self, text):
        if not self.voice:
            return
        # 1) Try pyttsx3
        if self.tts:
            try:
                self.tts.say(text)
                self.tts.runAndWait()
                return
            except Exception:
                pass
        # 2) Fallback: espeak-ng CLI (robust on Pi)
        safe = text.replace('"', r'\"')
        os.system(f'espeak-ng -ven+f3 -s 175 "{safe}" >/dev/null 2>&1 &')

    def _beep(self):
        # quick triple beep using ffmpeg
        try:
            os.system('ffplay -nodisp -autoexit -loglevel quiet -f lavfi -i "sine=frequency=900:duration=0.6" >/dev/null 2>&1 &')
            sleep(0.7)
        except Exception:
            pass

    def _siren(self):
        for _ in range(3):
            self._beep()

    def _sms(self, body):
        if not (self.tw_client and self.sms and self.tw_from and self.sms_to):
            return
        try:
            self.tw_client.messages.create(from_=self.tw_from, to=self.sms_to, body=body)
        except Exception:
            pass

    def _call(self, text):
        if not (self.tw_client and self.call and self.tw_from and self.call_to):
            return
        try:
            self.tw_client.calls.create(
                to=self.call_to,
                from_=self.tw_from,
                twiml=f'<Response><Say>{text}</Say></Response>'
            )
        except Exception:
            pass

    def alert_all(self, speak_text, sms_text):
        if self.siren:
            self._siren()
        self._speak(speak_text)
        self._sms(sms_text)
        self._call(speak_text)
PY
