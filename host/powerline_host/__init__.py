"""Host-side tooling for the powerline analyzer.

    protocol.py   the frame format spoken by the firmware
    link.py       serial transport, stall detection, reconnect
    meters.py     Vrms, mains frequency, effective sample rate
    writers.py    optional CSV / WAV capture outputs
    session.py    the capture loop and its integrity reporting
    cli.py        argument parsing and wiring
"""
