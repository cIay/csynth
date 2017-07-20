Csynth is a simple polyphonic sound synthesizer (bandlimited sine, square, sawtooth and triangle waves with ADSR envelopes) that supports networking for real-time group collaboration. Csynth uses the PortAudio library for audio output and SDL for cross-platform multithreading, keyboard input and graphics. In addition, the Kiss FFT library is used for fast Fourier transforms.

The host IP and port may be set in the settings.ini file. Leaving the host IP blank means that you are host. Attack, decay, and release times can be specifed here as well in milliseconds. The option 'sustain_percent' represents where the decay ends and sustain begins. It is a percentage of the maximum attack amplitude (e.g. 50% means the sustain is half the maximum attack and the decay is the transition between 100% and 50%, 100% means the sustain is equal the maximum attack and therefore there is no decay transition).


