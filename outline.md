Files: 

stabilization.py - ROV Receives – For thruster control 

new_ar.py - ROV Receives – For arm control 

arm_sender.py - Topside Sends – For arm control 

thrust_sender.py - Topside Sends – For thruster control 

 

Requests: 

 

Ui Stages: 

Initial Launch Screen: 

Priority 1: Launches 2 files on topside: arm_sender.py & thrust_sender.py 

Priority 1: SH into Onboard Pi:  

Launches 2 files on ROV: stabilization.py & new_ar.py 

There should be two buttons. One for “Start onboard program” and “Start art topside program” After everything is started there should be a next button 

Control Screen: 

Priority 2: 2 Camera View 

Priority 2: Overlay with direction 

Priority 3: Overlay with full telemetry  

Priority 2: Button to toggle mosfet  

Priority 3: Mode toggle: 

Stabilization 

Drive/Armed 

Disarmed 

Priority 3: Start colmap buttons 

Priority 3: Start crabs button 

Priority 4: Battery voltage and amps and other info at the bottom 

 

 

 