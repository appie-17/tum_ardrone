# constants
uint32 PTAM_IDLE = 0           # PTAM not running.
uint32 PTAM_INITIALIZING = 1   # initialization (trails)
uint32 PTAM_LOST = 2           # ptam is running, but lost
uint32 PTAM_GOOD = 3           # tracking quality OK
uint32 PTAM_BEST = 4           # tracking quality best
uint32 PTAM_TOOKKF = 5         # just took a new KF (equivalent to PTAM_BEST)
uint32 PTAM_FALSEPOSITIVE = 6  # ptam thinks it is good, but its estimate is rejected.

# header
Header      header

# --------------------- other values ---------------------------
float32     scale         # ptam scale factor (PTAMpos * scale = WORLDpos).
uint32      state
float32     scaleAccuracy # if scale is very inaccurate, this is about 0.5, and grows up to 1 (=very accurate).
