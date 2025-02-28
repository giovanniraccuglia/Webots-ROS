#!/usr/bin/env python3

import tiago_pkg.utils as Utils

class Speaker(object):

    def __init__(self, language='it-IT'):
        Utils.call_service('speaker', 'set_language', language)

    def speak(self, text, volume=100):
        Utils.call_service('speaker', 'speak', text, volume/100)

    def is_speaking(self):
        resp = Utils.call_service('speaker', 'is_speaking', True)
        return resp.value

    def play(self, sound, loop=1, volume=100, pitch=1.0, balance=0):
        Utils.call_service('speaker', 'speak', sound, volume/100, pitch, balance, loop)
